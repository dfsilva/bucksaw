use std::collections::HashMap;
use std::sync::mpsc::Sender;

use blackbox_log::frame::Frame;
use blackbox_log::frame::FrameDef;
use blackbox_log::headers::DebugMode;
use blackbox_log::headers::Firmware;
use blackbox_log::headers::PwmProtocol;
use blackbox_log::units::FlagSet;

use crate::gui::blackbox_ui_ext::*;

#[allow(dead_code)]
#[derive(Clone)]
pub struct FlightData {
    pub index: usize,
    pub firmware: Firmware,
    pub firmware_date: Option<String>,
    pub board_info: Option<String>,
    pub craft_name: Option<String>,
    pub debug_mode: DebugMode,
    pub features: Vec<String>,
    pub esc_protocol: PwmProtocol,
    pub unknown_headers: HashMap<String, String>,
    pub times: Vec<f64>,
    pub main_values: HashMap<String, Vec<f32>>,
    pub main_units: HashMap<String, String>,
}

impl FlightData {
    pub async fn parse(
        index: usize,
        headers: blackbox_log::headers::Headers<'_>,
        progress_sender: Sender<f32>,
    ) -> Result<Self, ()> {
        let mut parser = headers.data_parser();

        let main_frame_defs: Vec<_> = parser.main_frame_def().iter().collect();

        let main_units = main_frame_defs
            .iter()
            .filter_map(|def| {
                let unit = match def.unit {
                    blackbox_log::frame::MainUnit::Amperage => Some("A"),
                    blackbox_log::frame::MainUnit::Voltage => Some("V"),
                    blackbox_log::frame::MainUnit::Acceleration => Some("m/s²"),
                    blackbox_log::frame::MainUnit::Rotation => Some("°/s"),
                    blackbox_log::frame::MainUnit::Unitless => None,
                };
                unit.map(|u| (def.name.to_string(), u.to_string()))
            })
            .collect();

        let mut times = Vec::new();
        let mut main_values: HashMap<String, Vec<f32>> = HashMap::new();
        let mut i = 0;

        while let Some(frame) = parser.next() {
            if let blackbox_log::ParserEvent::Main(frame) = frame {
                if frame.time().value < times.last().copied().unwrap_or_default() {
                    continue;
                }

                times.push(frame.time().value);

                for (def, value) in main_frame_defs.iter().zip(frame.iter()) {
                    let float = match value {
                        blackbox_log::frame::MainValue::Amperage(val) => val.value as f32,
                        blackbox_log::frame::MainValue::Voltage(val) => val.value as f32,
                        blackbox_log::frame::MainValue::Acceleration(val) => val.value as f32,
                        blackbox_log::frame::MainValue::Rotation(val) => {
                            val.value.to_degrees() as f32
                        }
                        blackbox_log::frame::MainValue::Unsigned(val) => val as f32,
                        blackbox_log::frame::MainValue::Signed(val) => val as f32,
                    };

                    let entry = main_values.entry(def.name.to_owned()).or_default();

                    entry.push(float);
                }
            }

            if i == 0 {
                progress_sender.send(parser.stats().progress).unwrap();
                #[cfg(target_arch = "wasm32")]
                gloo_timers::future::TimeoutFuture::new(0).await;
            }
            i = (i + 1) % 1000;
        }

        progress_sender.send(1.0).unwrap();

        Ok(Self {
            index,
            firmware: headers.firmware(),
            firmware_date: headers
                .firmware_date()
                .and_then(|r| r.ok())
                .map(|dt| format!("{}", dt)),
            board_info: headers.board_info().map(|x| x.to_string()),
            craft_name: headers.craft_name().map(|x| x.to_string()),
            debug_mode: headers.debug_mode(),
            features: headers
                .features()
                .as_names()
                .iter()
                .map(|x| x.to_string())
                .collect(),
            esc_protocol: headers.pwm_protocol(),
            unknown_headers: headers
                .unknown()
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect(),
            times,
            main_values,
            main_units,
        })
    }

    fn get_vector_series<const N: usize>(&self, series_name: &str) -> Option<[&Vec<f32>; N]> {
        (0..N)
            .map(|i| self.main_values.get(&format!("{}[{}]", series_name, i)))
            .collect::<Option<Vec<_>>>()
            .and_then(|v| v.try_into().ok())
    }

    /// Get motor pole count from headers (used for eRPM to Hz conversion)
    /// Returns None if not found in headers, caller should default to 14
    pub fn motor_poles(&self) -> Option<u8> {
        self.unknown_headers
            .get("motor_poles")
            .or_else(|| self.unknown_headers.get("motorPoles"))
            .and_then(|s| s.parse().ok())
    }

    pub fn gyro_unfiltered(&self) -> Option<[&Vec<f32>; 3]> {
        self.get_vector_series("gyroUnfilt")
    }

    pub fn gyro_filtered(&self) -> Option<[&Vec<f32>; 3]> {
        self.get_vector_series("gyroADC")
    }

    pub fn accel(&self) -> Option<[&Vec<f32>; 3]> {
        self.get_vector_series("accSmooth")
    }

    pub fn rc_command(&self) -> Option<[&Vec<f32>; 4]> {
        self.get_vector_series("rcCommand")
    }

    pub fn setpoint(&self) -> Option<[&Vec<f32>; 4]> {
        self.get_vector_series("setpoint")
    }

    pub fn p(&self) -> Option<[&Vec<f32>; 3]> {
        self.get_vector_series("axisP")
    }

    pub fn i(&self) -> Option<[&Vec<f32>; 3]> {
        self.get_vector_series("axisI")
    }

    // Note the type signature change here, we might not have D gains for all axes
    pub fn d(&self) -> [Option<&Vec<f32>>; 3] {
        (0..3)
            .map(|i| self.main_values.get(&format!("axisD[{}]", i)))
            .collect::<Vec<_>>()
            .try_into()
            .unwrap()
    }

    pub fn f(&self) -> Option<[&Vec<f32>; 3]> {
        self.get_vector_series("axisF")
    }

    /// Calculate PID sum (P + I + D + F) for each axis
    pub fn pid_sum(&self) -> Option<[Vec<f32>; 3]> {
        let p = self.p()?;
        let i = self.i()?;
        let d = self.d();
        let f = self.f();

        let len = p[0].len();
        let mut result: [Vec<f32>; 3] = [
            Vec::with_capacity(len),
            Vec::with_capacity(len),
            Vec::with_capacity(len),
        ];

        for axis in 0..3 {
            for idx in 0..len {
                let p_val = p[axis].get(idx).copied().unwrap_or(0.0);
                let i_val = i[axis].get(idx).copied().unwrap_or(0.0);
                let d_val = d[axis].and_then(|d| d.get(idx).copied()).unwrap_or(0.0);
                let f_val = f
                    .as_ref()
                    .and_then(|f| f[axis].get(idx).copied())
                    .unwrap_or(0.0);
                result[axis].push(p_val + i_val + d_val + f_val);
            }
        }

        Some(result)
    }

    /// Get pre-filtered (raw) D-term if available
    /// This checks for dedicated fields first, then falls back to:
    /// 1. Debug channels when debug_mode is D_LPF
    /// 2. Calculating derivative of filtered gyro (which approximates D-term pre-filter)
    pub fn d_unfiltered(&self) -> [Option<&Vec<f32>>; 3] {
        // First try dedicated fields (newer firmware)
        let from_dedicated: [Option<&Vec<f32>>; 3] = (0..3)
            .map(|i| {
                self.main_values
                    .get(&format!("axisDRaw[{}]", i))
                    .or_else(|| self.main_values.get(&format!("axisD_UnFilt[{}]", i)))
            })
            .collect::<Vec<_>>()
            .try_into()
            .unwrap();

        // If any axis has dedicated data, use that
        if from_dedicated.iter().any(|opt| opt.is_some()) {
            return from_dedicated;
        }

        // Fall back to debug channels for D_LPF debug mode (mode 6)
        match self.debug_mode {
            DebugMode::DLpf => {
                [
                    self.main_values.get("debug[0]"), // Roll pre-filter
                    self.main_values.get("debug[2]"), // Pitch pre-filter
                    self.main_values.get("debug[4]"), // Yaw pre-filter (if available)
                ]
            }
            _ => [None, None, None],
        }
    }

    /// Calculate D-term pre-filter from filtered gyro derivative
    /// This approximates what PIDToolbox shows as "Dterm prefilt"
    pub fn d_unfiltered_calculated(&self) -> Option<[Vec<f32>; 3]> {
        let gyro = self.gyro_filtered()?;
        let sample_rate = self.sample_rate() as f32;
        let dt = 1.0 / sample_rate;

        Some([
            Self::calculate_derivative(gyro[0], dt),
            Self::calculate_derivative(gyro[1], dt),
            Self::calculate_derivative(gyro[2], dt),
        ])
    }

    /// Calculate the derivative of a signal (approximating D-term before filtering)
    fn calculate_derivative(data: &[f32], dt: f32) -> Vec<f32> {
        if data.len() < 2 {
            return vec![];
        }
        let mut result = Vec::with_capacity(data.len());
        result.push(0.0); // First sample has no derivative
        for i in 1..data.len() {
            result.push((data[i] - data[i - 1]) / dt);
        }
        result
    }

    /// Get debug channel data (up to 8 channels depending on debug mode)
    pub fn debug(&self) -> Option<Vec<&Vec<f32>>> {
        let debug_count = self
            .main_values
            .keys()
            .filter(|k| k.starts_with("debug["))
            .count();
        if debug_count == 0 {
            return None;
        }
        (0..debug_count)
            .map(|i| self.main_values.get(&format!("debug[{}]", i)))
            .collect::<Option<Vec<_>>>()
    }

    pub fn motor(&self) -> Option<Vec<&Vec<f32>>> {
        let motor_count = self
            .main_values
            .keys()
            .filter(|k| k.contains("motor"))
            .count();
        (0..motor_count)
            .map(|i| self.main_values.get(&format!("motor[{}]", i)))
            .collect::<Option<Vec<_>>>()
    }

    pub fn electrical_rpm(&self) -> Option<Vec<&Vec<f32>>> {
        let erpm_count = self
            .main_values
            .keys()
            .filter(|k| k.contains("eRPM"))
            .count();
        (0..erpm_count)
            .map(|i| self.main_values.get(&format!("eRPM[{}]", i)))
            .collect::<Option<Vec<_>>>()
    }

    pub fn battery_voltage(&self) -> Option<&Vec<f32>> {
        self.main_values.get("vbatLatest")
    }

    pub fn amperage(&self) -> Option<&Vec<f32>> {
        self.main_values.get("amperageLatest")
    }

    pub fn rssi(&self) -> Option<&Vec<f32>> {
        self.main_values.get("rssi")
    }

    // TODO: there's gotta be a better way to do this
    pub fn sample_rate(&self) -> f64 {
        const NUM_SAMPLES: usize = 100;
        let mut samples: Vec<u32> = self
            .times
            .windows(2)
            .map(|w| ((w[1] - w[0]) * 1_000_000.0) as u32)
            .take(NUM_SAMPLES)
            .collect();
        samples.sort();
        let sample_interval = samples[NUM_SAMPLES / 2];
        let rate = 1_000_000.0 / (sample_interval as f64);
        (rate / 100.0).round() * 100.0
    }

    pub fn show(&self, ui: &mut egui::Ui) -> bool {
        egui::Grid::new(ui.next_auto_id())
            .num_columns(2)
            .striped(true)
            .show(ui, |ui| {
                ui.label("FW");
                self.firmware.show(ui);
                ui.end_row();

                ui.label("Board");
                ui.label(self.board_info.clone().unwrap_or_default());
                ui.end_row();

                ui.label("Craft");
                ui.label(self.craft_name.clone().unwrap_or_default());
                ui.end_row();

                ui.label("Duration");
                if let Some(duration) = self
                    .times
                    .first()
                    .and_then(|f| self.times.last().map(|l| l - f))
                {
                    let freq = (self.times.len() as f64) / duration;
                    ui.label(format!("{:.3}s (~{:.0}Hz)", duration, freq));
                } else {
                    ui.label("");
                }
                ui.end_row();
            });

        false
    }
}

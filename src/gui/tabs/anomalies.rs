use egui::{Color32, RichText};
use egui_plot::{Plot, PlotPoints, Polygon};
use std::sync::Arc;

use crate::analytics;
use crate::flight_data::FlightData;

/// Action to navigate to a different tab with optional time range focus
#[derive(Clone, Debug)]
pub struct NavigationAction {
    pub target_time_start: f64,
    pub target_time_end: f64,
}

impl NavigationAction {
    pub fn new(start: f64, end: f64) -> Self {
        Self {
            target_time_start: start,
            target_time_end: end,
        }
    }

    /// Expand the time range for better context around the event
    pub fn with_context(start: f64, end: f64, total_duration: f64) -> Self {
        let margin = (end - start).max(0.5); // At least 0.5s context
        Self {
            target_time_start: (start - margin).max(0.0),
            target_time_end: (end + margin).min(total_duration),
        }
    }
}

#[allow(dead_code)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum AnomalyType {
    Desync,
    MotorSaturation,
    MidThrottleOscillation,
    HighVibration,
    RadioFailsafe,
    GyroNoiseSpike,
    // New anomaly types
    BatteryVoltageSag,
    PropWash,
    MotorImbalance,
    ITermWindup,
}

impl AnomalyType {
    pub fn color(&self) -> Color32 {
        match self {
            Self::Desync => Color32::RED,
            Self::MotorSaturation => Color32::from_rgb(255, 165, 0), // Orange
            Self::MidThrottleOscillation => Color32::YELLOW,
            Self::HighVibration => Color32::from_rgb(255, 100, 255), // Magenta
            Self::RadioFailsafe => Color32::from_rgb(255, 0, 0),     // Red
            Self::GyroNoiseSpike => Color32::from_rgb(0, 255, 255),  // Cyan
            // New colors
            Self::BatteryVoltageSag => Color32::from_rgb(255, 80, 80), // Light red
            Self::PropWash => Color32::from_rgb(180, 100, 255),        // Purple
            Self::MotorImbalance => Color32::from_rgb(255, 200, 100),  // Light orange
            Self::ITermWindup => Color32::from_rgb(100, 200, 255),     // Light blue
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            Self::Desync => "Desync / Motor Stall",
            Self::MotorSaturation => "Motor Saturation",
            Self::MidThrottleOscillation => "Mid-Throttle Oscillation",
            Self::HighVibration => "High Vibration Event",
            Self::RadioFailsafe => "Radio Failsafe / Signal Loss",
            Self::GyroNoiseSpike => "Gyro Noise Spike",
            // New labels
            Self::BatteryVoltageSag => "Battery Voltage Sag",
            Self::PropWash => "Propwash Oscillation",
            Self::MotorImbalance => "Motor Output Imbalance",
            Self::ITermWindup => "I-Term Windup",
        }
    }
}

pub struct AnomalyEvent {
    #[allow(dead_code)]
    pub start_idx: usize,
    #[allow(dead_code)]
    pub end_idx: usize,
    pub start_time: f64,
    pub end_time: f64,
    pub anomaly_type: AnomalyType,
    pub description: String,
    #[allow(dead_code)]
    pub severity: f32, // 0.0 to 1.0
}

struct AnomalyDetector;

impl AnomalyDetector {
    fn detect(fd: &FlightData) -> Vec<AnomalyEvent> {
        let mut events = Vec::new();

        let times = &fd.times;
        let count = times.len();
        if count == 0 {
            return events;
        }

        // 1. Detect Motor Saturation (> 1950 or > 95%)
        if let Some(motors) = fd.motor() {
            // Determine max motor value to guess scale
            let mut max_val = 0.0;
            for m in &motors {
                for v in m.iter() {
                    if *v > max_val {
                        max_val = *v;
                    }
                }
            }

            let saturation_threshold = if max_val > 1000.0 {
                1990.0 // Raw DShot/PWM
            } else {
                0.99 // Normalized or Percent
            };

            let mut start_idx = None;
            for i in 0..count {
                let is_saturated = motors
                    .iter()
                    .any(|m| m.get(i).copied().unwrap_or(0.0) >= saturation_threshold);

                if is_saturated {
                    if start_idx.is_none() {
                        start_idx = Some(i);
                    }
                } else if let Some(start) = start_idx {
                    if (times[i] - times[start]).abs() > 0.1 {
                        // Only count if > 100ms
                        events.push(AnomalyEvent {
                            start_idx: start,
                            end_idx: i,
                            start_time: times[start],
                            end_time: times[i],
                            anomaly_type: AnomalyType::MotorSaturation,
                            description: format!(
                                "Motors saturated for {:.2}s",
                                (times[i] - times[start]).abs()
                            ),
                            severity: 0.8,
                        });
                    }
                    start_idx = None;
                }
            }
        }

        // 2. Detect Gyro Noise Spikes
        if let Some(gyro) = fd.gyro_unfiltered() {
            // Check for extreme values > 2000 deg/s
            let mut start_idx = None;
            for i in 0..count {
                let is_spike = gyro
                    .iter()
                    .any(|axis| axis.get(i).copied().unwrap_or(0.0).abs() > 2000.0);

                if is_spike {
                    if start_idx.is_none() {
                        start_idx = Some(i);
                    }
                } else if let Some(start) = start_idx {
                    events.push(AnomalyEvent {
                        start_idx: start,
                        end_idx: i,
                        start_time: times[start],
                        end_time: times[i],
                        anomaly_type: AnomalyType::GyroNoiseSpike,
                        description: "Gyro spike > 2000 deg/s".to_string(),
                        severity: 0.6,
                    });
                    start_idx = None;
                }
            }
        }

        // 3. Detect High Vibration (Accelerometer)
        if let Some(acc) = fd.accel() {
            // Magnitude > 4G is usually a crash or extreme vibration
            let mut start_idx = None;
            for i in 0..count {
                let x = acc[0][i];
                let y = acc[1][i];
                let z = acc[2][i];
                let mag = (x * x + y * y + z * z).sqrt();

                if mag > 4.0 {
                    // 4G threshold
                    if start_idx.is_none() {
                        start_idx = Some(i);
                    }
                } else if let Some(start) = start_idx {
                    if (times[i] - times[start]).abs() > 0.05 {
                        events.push(AnomalyEvent {
                            start_idx: start,
                            end_idx: i,
                            start_time: times[start],
                            end_time: times[i],
                            anomaly_type: AnomalyType::HighVibration,
                            description: "High G-Force/Vibration (>4G)".to_string(),
                            severity: 0.7,
                        });
                    }
                    start_idx = None;
                }
            }
        }

        // 4. Detect Battery Voltage Sag (sudden drops)
        if let Some(voltage) = fd.battery_voltage() {
            if voltage.len() > 100 {
                let mut start_idx = None;
                for i in 10..voltage.len() {
                    // Check for >0.5V drop in last 10 samples
                    let current = voltage[i];
                    let recent_max = voltage[i.saturating_sub(10)..i]
                        .iter()
                        .cloned()
                        .fold(0.0f32, f32::max);

                    let drop = recent_max - current;
                    if drop > 0.5 && current > 10.0 {
                        // >0.5V sag, battery > 10V (not noise)
                        if start_idx.is_none() {
                            start_idx = Some(i);
                        }
                    } else if let Some(start) = start_idx {
                        if (times[i] - times[start]).abs() > 0.05 {
                            events.push(AnomalyEvent {
                                start_idx: start,
                                end_idx: i,
                                start_time: times[start],
                                end_time: times[i],
                                anomaly_type: AnomalyType::BatteryVoltageSag,
                                description: format!(
                                    "Voltage sag detected ({:.1}V drop)",
                                    recent_max - current
                                ),
                                severity: 0.6,
                            });
                        }
                        start_idx = None;
                    }
                }
            }
        }

        // 5. Detect Propwash (oscillations during descent / low throttle with high gyro activity)
        if let (Some(motors), Some(gyro)) = (fd.motor(), fd.gyro_filtered()) {
            if motors.len() >= 4 && !motors[0].is_empty() {
                let mut start_idx = None;
                for i in 20..count.min(motors[0].len()).min(gyro[0].len()) {
                    // Calculate average throttle (normalized)
                    let avg_motor: f32 = motors.iter().filter_map(|m| m.get(i)).sum::<f32>() / 4.0;
                    let throttle_pct = avg_motor / 20.0; // Rough % for DShot

                    // Check for low/descending throttle with oscillating gyro
                    let is_low_throttle = throttle_pct < 40.0;

                    // Gyro oscillation: check variance in recent samples
                    let window_start = i.saturating_sub(20);
                    let gyro_window: Vec<f32> = gyro[0][window_start..i].to_vec();
                    if gyro_window.len() >= 10 {
                        let mean: f32 = gyro_window.iter().sum::<f32>() / gyro_window.len() as f32;
                        let variance: f32 =
                            gyro_window.iter().map(|v| (v - mean).powi(2)).sum::<f32>()
                                / gyro_window.len() as f32;
                        let std_dev = variance.sqrt();

                        let is_oscillating = std_dev > 150.0; // High gyro variance

                        if is_low_throttle && is_oscillating {
                            if start_idx.is_none() {
                                start_idx = Some(i);
                            }
                        } else if let Some(start) = start_idx {
                            if (times[i] - times[start]).abs() > 0.1 {
                                events.push(AnomalyEvent {
                                    start_idx: start,
                                    end_idx: i,
                                    start_time: times[start],
                                    end_time: times[i],
                                    anomaly_type: AnomalyType::PropWash,
                                    description: "Propwash oscillation during descent".to_string(),
                                    severity: 0.5,
                                });
                            }
                            start_idx = None;
                        }
                    }
                }
            }
        }

        // 6. Detect Motor Imbalance (large difference between motor outputs)
        if let Some(motors) = fd.motor() {
            if motors.len() >= 4 {
                let mut start_idx = None;
                for i in 0..count.min(motors[0].len()) {
                    let motor_vals: Vec<f32> =
                        motors.iter().filter_map(|m| m.get(i).copied()).collect();

                    if motor_vals.len() == 4 {
                        let max = motor_vals.iter().cloned().fold(f32::MIN, f32::max);
                        let min = motor_vals.iter().cloned().fold(f32::MAX, f32::min);
                        let avg = motor_vals.iter().sum::<f32>() / 4.0;

                        // Significant imbalance: >30% difference when not at idle
                        let diff_pct = if avg > 200.0 {
                            (max - min) / avg * 100.0
                        } else {
                            0.0
                        };

                        if diff_pct > 30.0 {
                            if start_idx.is_none() {
                                start_idx = Some(i);
                            }
                        } else if let Some(start) = start_idx {
                            if (times[i] - times[start]).abs() > 0.2 {
                                events.push(AnomalyEvent {
                                    start_idx: start,
                                    end_idx: i,
                                    start_time: times[start],
                                    end_time: times[i],
                                    anomaly_type: AnomalyType::MotorImbalance,
                                    description: format!(
                                        "Motor outputs differ by >{:.0}%",
                                        diff_pct
                                    ),
                                    severity: 0.6,
                                });
                            }
                            start_idx = None;
                        }
                    }
                }
            }
        }

        // 7. Detect I-Term Windup (I-term growing while gyro error persists)
        if let Some(i_terms) = fd.i() {
            for axis in 0..3 {
                let i_data = &i_terms[axis];
                if i_data.len() > 50 {
                    let mut start_idx = None;
                    for i in 50..i_data.len() {
                        // Check if I-term is growing significantly
                        let current_i = i_data[i].abs();
                        let prev_i = i_data[i.saturating_sub(50)].abs();

                        let i_growing = current_i > prev_i * 1.5 && current_i > 200.0;

                        if i_growing {
                            if start_idx.is_none() {
                                start_idx = Some(i);
                            }
                        } else if let Some(start) = start_idx {
                            if (times[i] - times[start]).abs() > 0.3 {
                                let axis_name = ["Roll", "Pitch", "Yaw"][axis];
                                events.push(AnomalyEvent {
                                    start_idx: start,
                                    end_idx: i,
                                    start_time: times[start],
                                    end_time: times[i],
                                    anomaly_type: AnomalyType::ITermWindup,
                                    description: format!("{} I-term accumulating", axis_name),
                                    severity: 0.5,
                                });
                            }
                            start_idx = None;
                        }
                    }
                }
            }
        }

        events
    }
}

pub struct AnomaliesTab {
    fd: Arc<FlightData>,
    events: Vec<AnomalyEvent>,
}

impl AnomaliesTab {
    pub fn new(fd: Arc<FlightData>) -> Self {
        let events = AnomalyDetector::detect(&fd);
        Self { fd, events }
    }

    pub fn show(&mut self, ui: &mut egui::Ui) -> Option<NavigationAction> {
        let mut navigation_request: Option<NavigationAction> = None;

        ui.heading("🚨 Anomaly Detection");
        ui.add_space(8.0);

        if self.events.is_empty() {
            ui.label("No anomalies detected in this flight log.");
            ui.label(
                "This scan checks for motor saturation, gyro spikes, and high vibration events.",
            );
            return None;
        }

        let total_time = self.fd.times.last().copied().unwrap_or(1.0);

        egui::ScrollArea::vertical().show(ui, |ui| {
            // Timeline view
            ui.horizontal(|ui| {
                ui.label(RichText::new("Timeline").strong());
                ui.add_space(16.0);
                ui.label(
                    RichText::new("💡 Click an event below to jump to that time in the Plot tab")
                        .weak()
                        .small(),
                );
            });

            Plot::new("anomaly_timeline")
                .height(100.0)
                .include_x(0.0)
                .include_x(total_time)
                .include_y(0.0)
                .include_y(1.5)
                .show_axes([true, false])
                .link_cursor("global_timeseries", true, false)
                .show(ui, |plot_ui| {
                    for event in &self.events {
                        let color = event.anomaly_type.color();

                        // Use Polygon for filled block
                        let points = PlotPoints::new(vec![
                            [event.start_time, 0.2],
                            [event.end_time, 0.2],
                            [event.end_time, 0.8],
                            [event.start_time, 0.8],
                        ]);

                        plot_ui.polygon(
                            Polygon::new(points)
                                .fill_color(color.gamma_multiply(0.5))
                                .stroke(egui::Stroke::new(1.0, color))
                                .name(event.anomaly_type.label()),
                        );
                    }
                });

            ui.add_space(16.0);
            ui.separator();
            ui.add_space(8.0);

            // Event List
            ui.horizontal(|ui| {
                ui.label(RichText::new("Detected Anomalies").strong().size(16.0));
                ui.label(RichText::new(format!("({} events)", self.events.len())).weak());
            });
            ui.add_space(8.0);

            for (i, event) in self.events.iter().enumerate() {
                egui::Frame::none()
                    .fill(ui.style().visuals.extreme_bg_color)
                    .rounding(6.0)
                    .inner_margin(8.0)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            // Jump to button - clickable!
                            if ui
                                .button("🎯 Jump")
                                .on_hover_text(format!(
                                    "View this event at {:.1}s in Plot tab",
                                    event.start_time
                                ))
                                .clicked()
                            {
                                analytics::log_anomaly_jump(event.anomaly_type.label());
                                navigation_request = Some(NavigationAction::with_context(
                                    event.start_time,
                                    event.end_time,
                                    total_time,
                                ));
                            }

                            // Time range
                            let time_str =
                                format!("{:.1}s - {:.1}s", event.start_time, event.end_time);
                            ui.label(RichText::new(time_str).monospace());

                            ui.add_space(8.0);

                            // Severity indicator
                            ui.colored_label(
                                event.anomaly_type.color(),
                                event.anomaly_type.label(),
                            );

                            ui.add_space(8.0);

                            // Description
                            ui.label(&event.description);
                        });
                    });

                if i < self.events.len() - 1 {
                    ui.add_space(4.0);
                }
            }
        });

        navigation_request
    }
}

use std::sync::Arc;

use egui_oszi::{TimeseriesGroup, TimeseriesLine, TimeseriesPlot, TimeseriesPlotMemory};
use egui_plot::{Corner, Legend};

use crate::flight_data::FlightData;
use crate::gui::colors::Colors;

use super::PLOT_HEIGHT;

/// Smoothing level for time-series plots (like PIDtoolbox)
#[derive(Clone, Copy, PartialEq, Default)]
pub enum PlotSmoothingLevel {
    #[default]
    Off,
    Light,   // 3-point moving average
    Medium,  // 7-point moving average
    Heavy,   // 15-point moving average
    Maximum, // 31-point moving average
}

impl PlotSmoothingLevel {
    pub const ALL: [PlotSmoothingLevel; 5] = [
        PlotSmoothingLevel::Off,
        PlotSmoothingLevel::Light,
        PlotSmoothingLevel::Medium,
        PlotSmoothingLevel::Heavy,
        PlotSmoothingLevel::Maximum,
    ];

    fn window_size(self) -> usize {
        match self {
            PlotSmoothingLevel::Off => 1,
            PlotSmoothingLevel::Light => 3,
            PlotSmoothingLevel::Medium => 7,
            PlotSmoothingLevel::Heavy => 15,
            PlotSmoothingLevel::Maximum => 31,
        }
    }
}

impl std::fmt::Display for PlotSmoothingLevel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PlotSmoothingLevel::Off => write!(f, "Off"),
            PlotSmoothingLevel::Light => write!(f, "Light (3)"),
            PlotSmoothingLevel::Medium => write!(f, "Medium (7)"),
            PlotSmoothingLevel::Heavy => write!(f, "Heavy (15)"),
            PlotSmoothingLevel::Maximum => write!(f, "Max (31)"),
        }
    }
}

/// Apply moving average smoothing to data
fn apply_smoothing(data: impl Iterator<Item = f32>, window: usize) -> Vec<f32> {
    if window <= 1 {
        return data.collect();
    }

    let data: Vec<f32> = data.collect();
    if data.len() < window {
        return data;
    }

    let half = window / 2;
    let mut result = Vec::with_capacity(data.len());

    for i in 0..data.len() {
        let start = i.saturating_sub(half);
        let end = (i + half + 1).min(data.len());
        let sum: f32 = data[start..end].iter().sum();
        result.push(sum / (end - start) as f32);
    }

    result
}

pub struct PlotTab {
    gyro_plot: TimeseriesPlotMemory<f64, f32>,
    acc_plot: TimeseriesPlotMemory<f64, f32>,
    rc_plot: TimeseriesPlotMemory<f64, f32>,
    battery_plot: TimeseriesPlotMemory<f64, f32>,
    rssi_plot: TimeseriesPlotMemory<f64, f32>,
    motor_plot: TimeseriesPlotMemory<f64, f32>,
    erpm_plot: TimeseriesPlotMemory<f64, f32>,
    pid_sum_plot: TimeseriesPlotMemory<f64, f32>,
    d_unfilt_plot: TimeseriesPlotMemory<f64, f32>,
    debug_plot: TimeseriesPlotMemory<f64, f32>,
    // Cached PID sum since it's computed, not just a reference
    pid_sum_data: Option<[Vec<f32>; 3]>,
    fd: Arc<FlightData>,
    // Display settings
    smoothing: PlotSmoothingLevel,
    line_width: f32,
}

impl PlotTab {
    pub fn new(fd: Arc<FlightData>) -> Self {
        // Pre-compute PID sum
        let pid_sum_data = fd.pid_sum();

        Self {
            gyro_plot: TimeseriesPlotMemory::new("gyro"),
            acc_plot: TimeseriesPlotMemory::new("acc"),
            rc_plot: TimeseriesPlotMemory::new("rc"),
            battery_plot: TimeseriesPlotMemory::new("battery"),
            rssi_plot: TimeseriesPlotMemory::new("rssi"),
            motor_plot: TimeseriesPlotMemory::new("motors"),
            erpm_plot: TimeseriesPlotMemory::new("erpm"),
            pid_sum_plot: TimeseriesPlotMemory::new("pid_sum"),
            d_unfilt_plot: TimeseriesPlotMemory::new("d_unfilt"),
            debug_plot: TimeseriesPlotMemory::new("debug"),
            pid_sum_data,
            fd,
            smoothing: PlotSmoothingLevel::default(),
            line_width: 1.5,
        }
    }

    /// Helper to apply current smoothing to data
    fn smooth(&self, data: impl Iterator<Item = f32>) -> Vec<f32> {
        apply_smoothing(data, self.smoothing.window_size())
    }

    pub fn show(&mut self, ui: &mut egui::Ui, timeseries_group: &mut TimeseriesGroup) {
        let times = &self.fd.times;
        let legend = Legend::default().position(Corner::LeftTop);
        let colors = Colors::get(ui);

        // Track previous smoothing level to detect changes
        let prev_smoothing = self.smoothing;

        // Display controls
        ui.horizontal(|ui| {
            ui.label("Line Smoothing:");
            egui::ComboBox::from_id_source("plot_smoothing")
                .selected_text(self.smoothing.to_string())
                .show_ui(ui, |ui| {
                    for level in PlotSmoothingLevel::ALL {
                        ui.selectable_value(&mut self.smoothing, level, level.to_string());
                    }
                });

            ui.separator();

            ui.label("Line Width:");
            ui.add(egui::Slider::new(&mut self.line_width, 0.5..=5.0).step_by(0.5));
        });

        // If smoothing level changed, clear all plot caches so they rebuild with new data
        if prev_smoothing != self.smoothing {
            self.gyro_plot.clear_caches();
            self.acc_plot.clear_caches();
            self.rc_plot.clear_caches();
            self.battery_plot.clear_caches();
            self.rssi_plot.clear_caches();
            self.motor_plot.clear_caches();
            self.erpm_plot.clear_caches();
            self.pid_sum_plot.clear_caches();
            self.d_unfilt_plot.clear_caches();
            self.debug_plot.clear_caches();
        }

        ui.add_space(8.0);

        ui.heading("Gyroscope");
        let line_width = self.line_width;

        // Pre-compute smoothed gyro data
        let gyro_unfilt_smoothed: [Vec<f32>; 3] = [
            self.smooth(
                self.fd
                    .gyro_unfiltered()
                    .map(|s| s[0].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .gyro_unfiltered()
                    .map(|s| s[1].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .gyro_unfiltered()
                    .map(|s| s[2].iter().copied())
                    .unwrap_or_default(),
            ),
        ];
        let gyro_filt_smoothed: [Vec<f32>; 3] = [
            self.smooth(
                self.fd
                    .gyro_filtered()
                    .map(|s| s[0].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .gyro_filtered()
                    .map(|s| s[1].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .gyro_filtered()
                    .map(|s| s[2].iter().copied())
                    .unwrap_or_default(),
            ),
        ];

        ui.add(
            TimeseriesPlot::new(&mut self.gyro_plot)
                .group(timeseries_group)
                .legend(legend.clone())
                .height(PLOT_HEIGHT)
                .line(
                    TimeseriesLine::new("gyroUnfilt[0]")
                        .color(colors.triple_secondary[0])
                        .width(line_width),
                    times
                        .iter()
                        .copied()
                        .zip(gyro_unfilt_smoothed[0].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("gyroUnfilt[1]")
                        .color(colors.triple_secondary[1])
                        .width(line_width),
                    times
                        .iter()
                        .copied()
                        .zip(gyro_unfilt_smoothed[1].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("gyroUnfilt[2]")
                        .color(colors.triple_secondary[2])
                        .width(line_width),
                    times
                        .iter()
                        .copied()
                        .zip(gyro_unfilt_smoothed[2].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("gyroADC[0]")
                        .color(colors.triple_primary[0])
                        .width(line_width),
                    times
                        .iter()
                        .copied()
                        .zip(gyro_filt_smoothed[0].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("gyroADC[1]")
                        .color(colors.triple_primary[1])
                        .width(line_width),
                    times
                        .iter()
                        .copied()
                        .zip(gyro_filt_smoothed[1].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("gyroADC[2]")
                        .color(colors.triple_primary[2])
                        .width(line_width),
                    times
                        .iter()
                        .copied()
                        .zip(gyro_filt_smoothed[2].iter().copied()),
                ),
        );

        ui.heading("Accelerometer");
        // Pre-compute smoothed accelerometer data
        let acc_smoothed: [Vec<f32>; 3] = [
            self.smooth(
                self.fd
                    .accel()
                    .map(|s| s[0].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .accel()
                    .map(|s| s[1].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .accel()
                    .map(|s| s[2].iter().copied())
                    .unwrap_or_default(),
            ),
        ];
        ui.add(
            TimeseriesPlot::new(&mut self.acc_plot)
                .group(timeseries_group)
                .legend(legend.clone())
                .height(PLOT_HEIGHT)
                .line(
                    TimeseriesLine::new("accSmooth[0]")
                        .color(colors.triple_primary[0])
                        .width(line_width),
                    times.iter().copied().zip(acc_smoothed[0].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("accSmooth[1]")
                        .color(colors.triple_primary[1])
                        .width(line_width),
                    times.iter().copied().zip(acc_smoothed[1].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("accSmooth[2]")
                        .color(colors.triple_primary[2])
                        .width(line_width),
                    times.iter().copied().zip(acc_smoothed[2].iter().copied()),
                ),
        );

        ui.heading("RC Commands");
        // Pre-compute smoothed RC command data
        let rc_smoothed: [Vec<f32>; 4] = [
            self.smooth(
                self.fd
                    .rc_command()
                    .map(|s| s[0].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .rc_command()
                    .map(|s| s[1].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .rc_command()
                    .map(|s| s[2].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .rc_command()
                    .map(|s| s[3].iter().copied())
                    .unwrap_or_default(),
            ),
        ];
        ui.add(
            TimeseriesPlot::new(&mut self.rc_plot)
                .group(timeseries_group)
                .legend(legend.clone())
                .height(PLOT_HEIGHT)
                .line(
                    TimeseriesLine::new("rcCommand[0]")
                        .color(colors.quad[0])
                        .width(line_width),
                    times.iter().copied().zip(rc_smoothed[0].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("rcCommand[1]")
                        .color(colors.quad[1])
                        .width(line_width),
                    times.iter().copied().zip(rc_smoothed[1].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("rcCommand[2]")
                        .color(colors.quad[2])
                        .width(line_width),
                    times.iter().copied().zip(rc_smoothed[2].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("rcCommand[3]")
                        .color(colors.quad[3])
                        .width(line_width),
                    times.iter().copied().zip(rc_smoothed[3].iter().copied()),
                ),
        );

        // PID Sum (P + I + D + F) - NEW
        ui.heading("PID Sum (P+I+D+F)");
        // Pre-compute smoothed PID sum data before the plot block
        let window = self.smoothing.window_size();
        let pid_sum_smoothed: Option<[Vec<f32>; 3]> = self.pid_sum_data.as_ref().map(|pid_sum| {
            [
                apply_smoothing(pid_sum[0].iter().copied(), window),
                apply_smoothing(pid_sum[1].iter().copied(), window),
                apply_smoothing(pid_sum[2].iter().copied(), window),
            ]
        });
        ui.add({
            let mut plot = TimeseriesPlot::new(&mut self.pid_sum_plot)
                .group(timeseries_group)
                .legend(legend.clone())
                .height(PLOT_HEIGHT);

            if let Some(pid_sum_smoothed) = &pid_sum_smoothed {
                for (i, axis) in ["Roll", "Pitch", "Yaw"].iter().enumerate() {
                    plot = plot.line(
                        TimeseriesLine::new(format!("PID Sum [{}]", axis))
                            .color(colors.triple_primary[i]),
                        times
                            .iter()
                            .copied()
                            .zip(pid_sum_smoothed[i].iter().copied()),
                    );
                }
            }
            plot
        });

        // D-term unfiltered (pre-filtered) - NEW
        ui.heading("D-term (Pre-filtered)");
        // Pre-compute smoothed D-term data before the plot block
        let d_unfilt = self.fd.d_unfiltered();
        let d_unfilt_smoothed: [Option<Vec<f32>>; 3] = [
            d_unfilt[0].map(|d| apply_smoothing(d.iter().copied(), window)),
            d_unfilt[1].map(|d| apply_smoothing(d.iter().copied(), window)),
            d_unfilt[2].map(|d| apply_smoothing(d.iter().copied(), window)),
        ];
        let d_filt = self.fd.d();
        let d_filt_smoothed: [Option<Vec<f32>>; 3] = [
            d_filt[0].map(|d| apply_smoothing(d.iter().copied(), window)),
            d_filt[1].map(|d| apply_smoothing(d.iter().copied(), window)),
            d_filt[2].map(|d| apply_smoothing(d.iter().copied(), window)),
        ];
        ui.add({
            let mut plot = TimeseriesPlot::new(&mut self.d_unfilt_plot)
                .group(timeseries_group)
                .legend(legend.clone())
                .height(PLOT_HEIGHT);

            for (i, axis) in ["Roll", "Pitch", "Yaw"].iter().enumerate() {
                if let Some(d_data) = &d_unfilt_smoothed[i] {
                    plot = plot.line(
                        TimeseriesLine::new(format!("D Unfilt [{}]", axis))
                            .color(colors.triple_secondary[i]),
                        times.iter().copied().zip(d_data.iter().copied()),
                    );
                }
            }

            for (i, axis) in ["Roll", "Pitch", "Yaw"].iter().enumerate() {
                if let Some(d_data) = &d_filt_smoothed[i] {
                    plot = plot.line(
                        TimeseriesLine::new(format!("D Filt [{}]", axis))
                            .color(colors.triple_primary[i]),
                        times.iter().copied().zip(d_data.iter().copied()),
                    );
                }
            }
            plot
        });

        // Debug Channels - NEW
        // Pre-compute smoothed debug data before the plot block
        let debug_smoothed: Option<Vec<Vec<f32>>> = self.fd.debug().map(|debug_data| {
            debug_data
                .iter()
                .map(|ch| apply_smoothing(ch.iter().copied(), window))
                .collect()
        });
        if let Some(debug_smoothed) = &debug_smoothed {
            ui.heading(format!("Debug Channels ({:?})", self.fd.debug_mode));
            ui.add({
                let mut plot = TimeseriesPlot::new(&mut self.debug_plot)
                    .group(timeseries_group)
                    .legend(legend.clone())
                    .height(PLOT_HEIGHT);

                for (i, ch) in debug_smoothed.iter().enumerate() {
                    plot = plot.line(
                        TimeseriesLine::new(format!("debug[{}]", i))
                            .color(colors.motors[i % colors.motors.len()]),
                        times.iter().copied().zip(ch.iter().copied()),
                    );
                }
                plot
            });
        }

        ui.heading("Motors");
        // Pre-compute smoothed motor data
        let motor_smoothed: [Vec<f32>; 4] = [
            self.smooth(
                self.fd
                    .motor()
                    .map(|s| s[0].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .motor()
                    .map(|s| s[1].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .motor()
                    .map(|s| s[2].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .motor()
                    .map(|s| s[3].iter().copied())
                    .unwrap_or_default(),
            ),
        ];
        ui.add(
            TimeseriesPlot::new(&mut self.motor_plot)
                .group(timeseries_group)
                .legend(legend.clone())
                .height(PLOT_HEIGHT)
                .line(
                    TimeseriesLine::new("motor[0]")
                        .color(colors.motors[0])
                        .width(line_width),
                    times.iter().copied().zip(motor_smoothed[0].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("motor[1]")
                        .color(colors.motors[1])
                        .width(line_width),
                    times.iter().copied().zip(motor_smoothed[1].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("motor[2]")
                        .color(colors.motors[2])
                        .width(line_width),
                    times.iter().copied().zip(motor_smoothed[2].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("motor[3]")
                        .color(colors.motors[3])
                        .width(line_width),
                    times.iter().copied().zip(motor_smoothed[3].iter().copied()),
                ),
        );

        ui.heading("eRPM");
        // Pre-compute smoothed eRPM data
        let erpm_smoothed: [Vec<f32>; 4] = [
            self.smooth(
                self.fd
                    .electrical_rpm()
                    .map(|s| s[0].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .electrical_rpm()
                    .map(|s| s[1].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .electrical_rpm()
                    .map(|s| s[2].iter().copied())
                    .unwrap_or_default(),
            ),
            self.smooth(
                self.fd
                    .electrical_rpm()
                    .map(|s| s[3].iter().copied())
                    .unwrap_or_default(),
            ),
        ];
        ui.add(
            TimeseriesPlot::new(&mut self.erpm_plot)
                .group(timeseries_group)
                .legend(legend.clone())
                .height(PLOT_HEIGHT)
                .line(
                    TimeseriesLine::new("eRPM[0]")
                        .color(colors.motors[0])
                        .width(line_width),
                    times.iter().copied().zip(erpm_smoothed[0].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("eRPM[1]")
                        .color(colors.motors[1])
                        .width(line_width),
                    times.iter().copied().zip(erpm_smoothed[1].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("eRPM[2]")
                        .color(colors.motors[2])
                        .width(line_width),
                    times.iter().copied().zip(erpm_smoothed[2].iter().copied()),
                )
                .line(
                    TimeseriesLine::new("eRPM[3]")
                        .color(colors.motors[3])
                        .width(line_width),
                    times.iter().copied().zip(erpm_smoothed[3].iter().copied()),
                ),
        );

        ui.heading("Battery");
        // Pre-compute smoothed battery data
        let voltage_smoothed = self.smooth(
            self.fd
                .battery_voltage()
                .map(|s| s.iter().copied())
                .unwrap_or_default(),
        );
        let amperage_smoothed = self.smooth(
            self.fd
                .amperage()
                .map(|s| s.iter().copied())
                .unwrap_or_default(),
        );
        ui.add(
            TimeseriesPlot::new(&mut self.battery_plot)
                .group(timeseries_group)
                .legend(legend.clone())
                .height(PLOT_HEIGHT)
                .line(
                    TimeseriesLine::new("vbatLatest")
                        .color(colors.voltage)
                        .width(line_width),
                    times.iter().copied().zip(voltage_smoothed.iter().copied()),
                )
                .line(
                    TimeseriesLine::new("amperageLatest")
                        .color(colors.current)
                        .width(line_width),
                    times.iter().copied().zip(amperage_smoothed.iter().copied()),
                ),
        );

        ui.heading("RSSI");
        // Pre-compute smoothed RSSI data
        let rssi_smoothed = self.smooth(
            self.fd
                .rssi()
                .map(|s| s.iter().copied())
                .unwrap_or_default(),
        );
        ui.add(
            TimeseriesPlot::new(&mut self.rssi_plot)
                .group(timeseries_group)
                .legend(legend.clone())
                .height(PLOT_HEIGHT)
                .line(
                    TimeseriesLine::new("rssi")
                        .color(colors.rssi)
                        .width(line_width),
                    times.iter().copied().zip(rssi_smoothed.iter().copied()),
                ),
        );
    }
}

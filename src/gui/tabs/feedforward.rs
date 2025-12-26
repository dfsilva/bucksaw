use crate::flight_data::FlightData;
use egui::{Color32, RichText};
use egui_plot::{Legend, Line, Plot, PlotPoints};
use std::sync::Arc;

pub struct FeedforwardTab {
    fd: Arc<FlightData>,
    // Analysis metrics
    ff_effectiveness: f32,
    setpoint_jitter: f32,
    max_stick_rate: [f32; 3],
    avg_delay: f32,
}

impl FeedforwardTab {
    pub fn new(fd: Arc<FlightData>) -> Self {
        let mut tab = Self {
            fd,
            ff_effectiveness: 0.0,
            setpoint_jitter: 0.0,
            max_stick_rate: [0.0; 3],
            avg_delay: 0.0,
        };
        tab.compute_metrics();
        tab
    }

    fn compute_metrics(&mut self) {
        let sample_rate = self.fd.sample_rate();
        // Re-implementing logic from suggestions.rs for standalone display
        if let Some(setpoint) = self.fd.setpoint() {
            // 1. Calculate Stick Rates & Jitter
            let mut total_jitter = 0.0;
            for axis in 0..3 {
                if setpoint[axis].len() > 10 {
                    let dt = 1.0 / sample_rate as f32;
                    let mut stick_rates = Vec::new();
                    for i in 1..setpoint[axis].len() {
                        let rate = (setpoint[axis][i] - setpoint[axis][i - 1]) / dt;
                        stick_rates.push(rate);
                        if rate.abs() > self.max_stick_rate[axis] {
                            self.max_stick_rate[axis] = rate.abs();
                        }
                    }

                    if stick_rates.len() > 10 {
                        let mean: f32 = stick_rates.iter().sum::<f32>() / stick_rates.len() as f32;
                        let variance: f32 =
                            stick_rates.iter().map(|r| (r - mean).powi(2)).sum::<f32>()
                                / stick_rates.len() as f32;
                        total_jitter += variance.sqrt() / 100.0;
                    }
                }
            }
            self.setpoint_jitter = total_jitter / 3.0;

            // 2. FF Effectiveness (Gyro vs Setpoint transition)
            if let Some(gyro) = self.fd.gyro_filtered() {
                let mut effective_transitions = 0;
                let mut total_transitions = 0;
                let threshold = 50.0;

                for axis in 0..3 {
                    let len = setpoint[axis].len().min(gyro[axis].len());
                    for i in 5..len.saturating_sub(5) {
                        let sp_change =
                            (setpoint[axis][i] - setpoint[axis][i.saturating_sub(3)]).abs();
                        if sp_change > threshold {
                            total_transitions += 1;
                            let gyro_change =
                                (gyro[axis][i] - gyro[axis][i.saturating_sub(3)]).abs();
                            if gyro_change > threshold * 0.5 {
                                effective_transitions += 1;
                            }
                        }
                    }
                }
                if total_transitions > 10 {
                    self.ff_effectiveness =
                        (effective_transitions as f32 / total_transitions as f32) * 100.0;
                }
            }
        }
    }

    pub fn show(&mut self, ui: &mut egui::Ui) {
        ui.heading("➤ Feedforward Analysis");
        ui.add_space(10.0);

        // Metrics Grid
        egui::Grid::new("ff_metrics")
            .striped(true)
            .spacing([40.0, 10.0])
            .show(ui, |ui| {
                // FF Effectiveness
                ui.label("FF Effectiveness Score:");
                let color = if self.ff_effectiveness > 80.0 {
                    Color32::GREEN
                } else if self.ff_effectiveness > 50.0 {
                    Color32::YELLOW
                } else {
                    Color32::RED
                };
                ui.label(
                    RichText::new(format!("{:.1}%", self.ff_effectiveness))
                        .color(color)
                        .strong(),
                );
                ui.label("How well the quad tracks rapid setpoint changes. >80% is ideal.");
                ui.end_row();

                // Setpoint Jitter
                ui.label("RC/Setpoint Jitter:");
                let jitter_color = if self.setpoint_jitter < 5.0 {
                    Color32::GREEN
                } else if self.setpoint_jitter < 10.0 {
                    Color32::YELLOW
                } else {
                    Color32::RED
                };
                ui.label(
                    RichText::new(format!("{:.1}", self.setpoint_jitter))
                        .color(jitter_color)
                        .strong(),
                );
                ui.label("Noise in RC signal. High values (>10) cause hot motors.");
                ui.end_row();

                // Max Stick Rate
                ui.label("Max Stick Rate (measured):");
                ui.label(format!(
                    "{:.0} / {:.0} / {:.0} deg/s",
                    self.max_stick_rate[0], self.max_stick_rate[1], self.max_stick_rate[2]
                ));
                ui.label("Highest commanded rotation rate on Roll/Pitch/Yaw.");
                ui.end_row();
            });

        ui.add_space(20.0);
        ui.separator();
        ui.add_space(10.0);

        ui.heading("Stick Input Analysis");
        ui.label("Visualizing setpoint transitions to evaluate feedforward performance.");

        // Simple Setpoint vs Gyro Plot (Sample - usually needs Zoom)
        // For now, just a placeholder plot or simple time domain of the first few seconds
        let height = ui.available_height().min(300.0);
        Plot::new("ff_plot")
            .height(height)
            .show_grid(true)
            .legend(Legend::default())
            .show(ui, |plot_ui| {
                if let (Some(setpoint), Some(gyro)) = (self.fd.setpoint(), self.fd.gyro_filtered())
                {
                    let times = &self.fd.times;
                    let limit = times.len().min(1000); // Show first 1000 samples for performance

                    // Roll Axis
                    let sp_points: PlotPoints = times
                        .iter()
                        .take(limit)
                        .zip(setpoint[0].iter().take(limit))
                        .map(|(t, v)| [*t, *v as f64])
                        .collect();
                    plot_ui.line(
                        Line::new(sp_points)
                            .name("Roll Setpoint")
                            .color(Color32::from_rgb(255, 100, 100)),
                    );

                    let gyro_points: PlotPoints = times
                        .iter()
                        .take(limit)
                        .zip(gyro[0].iter().take(limit))
                        .map(|(t, v)| [*t, *v as f64])
                        .collect();
                    plot_ui.line(
                        Line::new(gyro_points)
                            .name("Roll Gyro")
                            .color(Color32::from_rgb(100, 255, 100)),
                    );
                }
            });
    }
}

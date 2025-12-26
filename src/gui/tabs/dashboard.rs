//! Dashboard Tab - Quick flight health overview
//! Shows summary metrics and status from all analysis areas

use std::sync::Arc;

use egui::{Color32, RichText, Ui};
use egui_phosphor::regular as icons;

use crate::flight_data::FlightData;
use crate::gui::colors::Colors;

/// Health score levels
#[derive(Clone, Copy, PartialEq)]
enum HealthLevel {
    Excellent,
    Good,
    Warning,
    Poor,
}

impl HealthLevel {
    fn color(&self) -> Color32 {
        match self {
            HealthLevel::Excellent => Color32::from_rgb(0x83, 0xa5, 0x98), // Green
            HealthLevel::Good => Color32::from_rgb(0x8e, 0xc0, 0x7c),      // Light green
            HealthLevel::Warning => Color32::from_rgb(0xfa, 0xbd, 0x2f),   // Yellow
            HealthLevel::Poor => Color32::from_rgb(0xfb, 0x49, 0x34),      // Red
        }
    }

    fn label(&self) -> &'static str {
        match self {
            HealthLevel::Excellent => "Excellent",
            HealthLevel::Good => "Good",
            HealthLevel::Warning => "Needs Attention",
            HealthLevel::Poor => "Poor",
        }
    }

    fn icon(&self) -> &'static str {
        match self {
            HealthLevel::Excellent => icons::CHECK_CIRCLE,
            HealthLevel::Good => icons::CHECK,
            HealthLevel::Warning => icons::WARNING,
            HealthLevel::Poor => icons::X_CIRCLE,
        }
    }
}

/// Computed dashboard metrics
struct DashboardMetrics {
    // Overall scores
    overall_health: HealthLevel,
    overall_score: f32, // 0-100

    // Individual area scores
    noise_score: f32,
    tracking_score: f32,
    motor_score: f32,
    filter_score: f32,

    // Quick stats
    flight_duration_sec: f64,
    sample_count: usize,
    sample_rate: f64,

    // Noise metrics
    gyro_noise_rms: [f32; 3],
    dterm_noise_rms: [f32; 3],

    // Tracking metrics
    tracking_error_rms: [f32; 3],

    // Motor metrics
    motor_saturation_pct: f32,
    motor_balance: f32, // 0-100, higher is better

    // Anomalies
    anomaly_count: usize,
}

impl DashboardMetrics {
    fn compute(fd: &FlightData) -> Self {
        let flight_duration_sec =
            fd.times.last().copied().unwrap_or(0.0) - fd.times.first().copied().unwrap_or(0.0);
        let sample_count = fd.times.len();
        let sample_rate = fd.sample_rate();

        // Compute noise metrics
        let gyro_noise_rms = Self::compute_gyro_noise(fd);
        let dterm_noise_rms = Self::compute_dterm_noise(fd);

        // Compute tracking error
        let tracking_error_rms = Self::compute_tracking_error(fd);

        // Compute motor metrics
        let (motor_saturation_pct, motor_balance) = Self::compute_motor_metrics(fd);

        // Count anomalies (simplified detection)
        let anomaly_count = Self::count_anomalies(fd);

        // Calculate individual scores (0-100, higher is better)
        let noise_score = Self::calculate_noise_score(&gyro_noise_rms, &dterm_noise_rms);
        let tracking_score = Self::calculate_tracking_score(&tracking_error_rms);
        let motor_score = Self::calculate_motor_score(motor_saturation_pct, motor_balance);
        let filter_score = noise_score; // Simplified - same as noise for now

        // Overall score is weighted average
        let overall_score = (noise_score * 0.35 + tracking_score * 0.35 + motor_score * 0.30);

        let overall_health = if overall_score >= 85.0 {
            HealthLevel::Excellent
        } else if overall_score >= 70.0 {
            HealthLevel::Good
        } else if overall_score >= 50.0 {
            HealthLevel::Warning
        } else {
            HealthLevel::Poor
        };

        Self {
            overall_health,
            overall_score,
            noise_score,
            tracking_score,
            motor_score,
            filter_score,
            flight_duration_sec,
            sample_count,
            sample_rate,
            gyro_noise_rms,
            dterm_noise_rms,
            tracking_error_rms,
            motor_saturation_pct,
            motor_balance,
            anomaly_count,
        }
    }

    fn compute_gyro_noise(fd: &FlightData) -> [f32; 3] {
        let mut rms = [0.0; 3];
        if let Some(gyro) = fd.gyro_filtered() {
            for axis in 0..3 {
                if gyro[axis].len() > 10 {
                    let data = &gyro[axis];
                    let mean: f32 = data.iter().sum::<f32>() / data.len() as f32;
                    let _variance: f32 =
                        data.iter().map(|x| (x - mean).powi(2)).sum::<f32>() / data.len() as f32;
                    // High-pass estimate: use std dev of diff as noise proxy
                    let noise: f32 = data.windows(2).map(|w| (w[1] - w[0]).abs()).sum::<f32>()
                        / (data.len() - 1) as f32;
                    rms[axis] = noise * 5.0; // Scale factor
                }
            }
        }
        rms
    }

    fn compute_dterm_noise(fd: &FlightData) -> [f32; 3] {
        let mut rms = [0.0; 3];
        let d_terms = fd.d();
        for axis in 0..3 {
            if let Some(d_data) = d_terms[axis] {
                if d_data.len() > 10 {
                    let noise: f32 = d_data.windows(2).map(|w| (w[1] - w[0]).abs()).sum::<f32>()
                        / (d_data.len() - 1) as f32;
                    rms[axis] = noise * 5.0;
                }
            }
        }
        rms
    }

    fn compute_tracking_error(fd: &FlightData) -> [f32; 3] {
        let mut rms = [0.0; 3];
        if let (Some(setpoint), Some(gyro)) = (fd.setpoint(), fd.gyro_filtered()) {
            for axis in 0..3 {
                let n = setpoint[axis].len().min(gyro[axis].len());
                if n > 0 {
                    let error_sum: f32 = setpoint[axis]
                        .iter()
                        .zip(gyro[axis].iter())
                        .take(n)
                        .map(|(sp, gy)| (sp - gy).abs())
                        .sum();
                    rms[axis] = error_sum / n as f32;
                }
            }
        }
        rms
    }

    fn compute_motor_metrics(fd: &FlightData) -> (f32, f32) {
        let mut saturation_pct = 0.0;
        let mut balance = 100.0;

        if let Some(motors) = fd.motor() {
            if motors.len() >= 4 {
                let n = motors[0].len();
                if n > 0 {
                    // Saturation: count samples where any motor is near max
                    let mut sat_count = 0;
                    for i in 0..n {
                        let max_motor = motors
                            .iter()
                            .filter_map(|m| m.get(i))
                            .fold(0.0f32, |a, &b| a.max(b));
                        // Assume max is ~2000 for DShot, ~1.0 for normalized
                        let threshold = if max_motor > 100.0 { 1950.0 } else { 0.95 };
                        if max_motor >= threshold {
                            sat_count += 1;
                        }
                    }
                    saturation_pct = (sat_count as f32 / n as f32) * 100.0;

                    // Balance: measure variance between motors
                    let motor_avgs: Vec<f32> = motors
                        .iter()
                        .filter(|m| !m.is_empty()) // Guard against empty motor data
                        .map(|m| m.iter().sum::<f32>() / m.len() as f32)
                        .collect();

                    if !motor_avgs.is_empty() {
                        let avg_of_avgs = motor_avgs.iter().sum::<f32>() / motor_avgs.len() as f32;
                        if avg_of_avgs > 0.0 {
                            let variance = motor_avgs
                                .iter()
                                .map(|x| ((x - avg_of_avgs) / avg_of_avgs).powi(2))
                                .sum::<f32>()
                                / motor_avgs.len() as f32;
                            balance = (100.0 - variance.sqrt() * 100.0).clamp(0.0, 100.0);
                        }
                    }
                }
            }
        }

        (saturation_pct, balance)
    }

    fn count_anomalies(fd: &FlightData) -> usize {
        let mut count = 0;

        // Count motor saturation events
        if let Some(motors) = fd.motor() {
            if !motors.is_empty() {
                let threshold = if motors[0].first().copied().unwrap_or(0.0) > 100.0 {
                    1990.0
                } else {
                    0.99
                };
                let mut in_event = false;
                for i in 0..motors[0].len() {
                    let saturated = motors
                        .iter()
                        .any(|m| m.get(i).copied().unwrap_or(0.0) >= threshold);
                    if saturated && !in_event {
                        count += 1;
                        in_event = true;
                    } else if !saturated {
                        in_event = false;
                    }
                }
            }
        }

        // Count gyro spikes
        if let Some(gyro) = fd.gyro_unfiltered() {
            for axis_data in gyro {
                for &val in axis_data {
                    if val.abs() > 2000.0 {
                        count += 1;
                    }
                }
            }
        }

        count.min(99) // Cap for display
    }

    fn calculate_noise_score(gyro: &[f32; 3], dterm: &[f32; 3]) -> f32 {
        let avg_gyro = (gyro[0] + gyro[1] + gyro[2]) / 3.0;
        let avg_dterm = (dterm[0] + dterm[1] + dterm[2]) / 3.0;

        // Score based on noise levels (lower is better)
        // Typical good values: gyro < 10, dterm < 30
        let gyro_score = (100.0 - avg_gyro.clamp(0.0, 100.0)).max(0.0);
        let dterm_score = (100.0 - (avg_dterm / 2.0).clamp(0.0, 100.0)).max(0.0);

        (gyro_score * 0.6 + dterm_score * 0.4)
    }

    fn calculate_tracking_score(error: &[f32; 3]) -> f32 {
        let avg_error = (error[0] + error[1] + error[2]) / 3.0;
        // Good tracking: < 20 deg/s error
        (100.0 - (avg_error / 0.5).clamp(0.0, 100.0)).max(0.0)
    }

    fn calculate_motor_score(saturation: f32, balance: f32) -> f32 {
        let sat_score = (100.0 - saturation * 5.0).clamp(0.0, 100.0);
        sat_score * 0.7 + balance * 0.3
    }
}

pub struct DashboardTab {
    fd: Arc<FlightData>,
    metrics: DashboardMetrics,
}

impl DashboardTab {
    pub fn new(fd: Arc<FlightData>) -> Self {
        let metrics = DashboardMetrics::compute(&fd);
        Self { fd, metrics }
    }

    pub fn show(&mut self, ui: &mut Ui) {
        let colors = Colors::get(ui);

        egui::ScrollArea::vertical().show(ui, |ui| {
            // Header with overall health
            ui.horizontal(|ui| {
                ui.heading(format!("{} Flight Dashboard", icons::SQUARES_FOUR));
                ui.add_space(16.0);
                ui.label(
                    RichText::new(format!(
                        "{} Overall: {} ({:.0}/100)",
                        self.metrics.overall_health.icon(),
                        self.metrics.overall_health.label(),
                        self.metrics.overall_score
                    ))
                    .size(16.0)
                    .color(self.metrics.overall_health.color()),
                );
            });

            ui.add_space(12.0);

            // Flight info card
            self.show_flight_info_card(ui);

            ui.add_space(16.0);

            // Score cards grid
            ui.label(RichText::new("Analysis Scores").strong().size(15.0));
            ui.add_space(8.0);

            ui.horizontal(|ui| {
                self.show_score_card(
                    ui,
                    "∼ Noise",
                    self.metrics.noise_score,
                    "Lower is better. Based on gyro and D-term noise levels.",
                    colors.gyro_filtered,
                );
                ui.add_space(12.0);
                self.show_score_card(
                    ui,
                    "◎ Tracking",
                    self.metrics.tracking_score,
                    "How well gyro follows setpoint. Higher is better.",
                    colors.setpoint,
                );
                ui.add_space(12.0);
                self.show_score_card(
                    ui,
                    "↗ Motors",
                    self.metrics.motor_score,
                    "Motor saturation and balance. Higher is better.",
                    colors.motors[0],
                );
                ui.add_space(12.0);
                self.show_score_card(
                    ui,
                    "| Filtering",
                    self.metrics.filter_score,
                    "Filter effectiveness score.",
                    colors.d,
                );
            });

            ui.add_space(16.0);
            ui.separator();
            ui.add_space(8.0);

            // Quick metrics
            ui.label(RichText::new("Key Metrics").strong().size(15.0));
            ui.add_space(8.0);

            egui::Grid::new("dashboard_metrics")
                .num_columns(4)
                .spacing([20.0, 8.0])
                .striped(true)
                .show(ui, |ui| {
                    // Gyro noise
                    ui.label(RichText::new("Gyro Noise RMS").strong());
                    ui.label(format!("R: {:.1}", self.metrics.gyro_noise_rms[0]));
                    ui.label(format!("P: {:.1}", self.metrics.gyro_noise_rms[1]));
                    ui.label(format!("Y: {:.1}", self.metrics.gyro_noise_rms[2]));
                    ui.end_row();

                    // D-term noise
                    ui.label(RichText::new("D-term Noise RMS").strong());
                    ui.label(format!("R: {:.1}", self.metrics.dterm_noise_rms[0]));
                    ui.label(format!("P: {:.1}", self.metrics.dterm_noise_rms[1]));
                    ui.label(format!("Y: {:.1}", self.metrics.dterm_noise_rms[2]));
                    ui.end_row();

                    // Tracking error
                    ui.label(RichText::new("Tracking Error").strong());
                    ui.label(format!("R: {:.1}°/s", self.metrics.tracking_error_rms[0]));
                    ui.label(format!("P: {:.1}°/s", self.metrics.tracking_error_rms[1]));
                    ui.label(format!("Y: {:.1}°/s", self.metrics.tracking_error_rms[2]));
                    ui.end_row();

                    // Motors
                    ui.label(RichText::new("Motor Saturation").strong());
                    ui.label(format!("{:.1}%", self.metrics.motor_saturation_pct));
                    ui.label(RichText::new("Balance").strong());
                    ui.label(format!("{:.0}%", self.metrics.motor_balance));
                    ui.end_row();
                });

            ui.add_space(16.0);

            // Anomalies summary
            if self.metrics.anomaly_count > 0 {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("⚠ Anomalies Detected:").strong());
                    ui.colored_label(
                        Color32::from_rgb(0xfb, 0x49, 0x34),
                        format!("{} events", self.metrics.anomaly_count),
                    );
                    ui.label("→ Check Anomalies tab for details");
                });
            } else {
                ui.label(
                    RichText::new("✓ No anomalies detected")
                        .color(Color32::from_rgb(0x83, 0xa5, 0x98)),
                );
            }

            ui.add_space(16.0);

            // Quick navigation hints
            ui.separator();
            ui.add_space(8.0);
            ui.label(RichText::new("★ Quick Tips").strong());

            if self.metrics.noise_score < 70.0 {
                ui.label("• High noise detected → Check Filter tab for tuning suggestions");
            }
            if self.metrics.tracking_score < 70.0 {
                ui.label("• Tracking issues → Check Error tab for detailed analysis");
            }
            if self.metrics.motor_saturation_pct > 5.0 {
                ui.label("• Motor saturation → Check Suggestions tab for PID recommendations");
            }
            if self.metrics.noise_score >= 70.0
                && self.metrics.tracking_score >= 70.0
                && self.metrics.motor_saturation_pct <= 5.0
            {
                ui.label(
                    "• Flight looks well-tuned! Check Tune tab for fine-tuning opportunities.",
                );
            }
        });
    }

    fn show_flight_info_card(&self, ui: &mut Ui) {
        egui::Frame::none()
            .fill(ui.style().visuals.extreme_bg_color)
            .rounding(8.0)
            .inner_margin(12.0)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    // Flight duration
                    ui.vertical(|ui| {
                        ui.label(RichText::new("Duration").weak());
                        ui.label(
                            RichText::new(format!("{:.1}s", self.metrics.flight_duration_sec))
                                .strong()
                                .size(18.0),
                        );
                    });

                    ui.add_space(40.0);

                    // Sample rate
                    ui.vertical(|ui| {
                        ui.label(RichText::new("Sample Rate").weak());
                        ui.label(
                            RichText::new(format!("{:.0} Hz", self.metrics.sample_rate))
                                .strong()
                                .size(18.0),
                        );
                    });

                    ui.add_space(40.0);

                    // Total samples
                    ui.vertical(|ui| {
                        ui.label(RichText::new("Samples").weak());
                        ui.label(
                            RichText::new(format!("{}", self.metrics.sample_count))
                                .strong()
                                .size(18.0),
                        );
                    });

                    ui.add_space(40.0);

                    // Firmware
                    ui.vertical(|ui| {
                        ui.label(RichText::new("Firmware").weak());
                        ui.label(RichText::new(format!("{:?}", self.fd.firmware)).strong());
                    });

                    if let Some(ref name) = self.fd.craft_name {
                        ui.add_space(40.0);
                        ui.vertical(|ui| {
                            ui.label(RichText::new("Craft").weak());
                            ui.label(RichText::new(name).strong());
                        });
                    }
                });
            });
    }

    fn show_score_card(
        &self,
        ui: &mut Ui,
        label: &str,
        score: f32,
        tooltip: &str,
        _accent: Color32,
    ) {
        let health = if score >= 85.0 {
            HealthLevel::Excellent
        } else if score >= 70.0 {
            HealthLevel::Good
        } else if score >= 50.0 {
            HealthLevel::Warning
        } else {
            HealthLevel::Poor
        };

        egui::Frame::none()
            .fill(ui.style().visuals.extreme_bg_color)
            .rounding(8.0)
            .inner_margin(12.0)
            .show(ui, |ui| {
                ui.set_min_width(100.0);
                ui.vertical(|ui| {
                    ui.label(RichText::new(label).size(13.0));
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new(format!("{:.0}", score))
                                .strong()
                                .size(24.0)
                                .color(health.color()),
                        );
                        ui.label(RichText::new("/100").weak().size(12.0));
                    });
                    ui.label(
                        RichText::new(health.label())
                            .size(11.0)
                            .color(health.color()),
                    );
                });
            })
            .response
            .on_hover_text(tooltip);
    }
}

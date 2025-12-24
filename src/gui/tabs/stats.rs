use std::sync::Arc;

use egui::{Color32, RichText, Stroke};
use egui_plot::{Bar, BarChart, Corner, Legend, Line, Plot, PlotPoints};

use crate::flight_data::FlightData;
use crate::gui::colors::Colors;
use crate::gui::flex::FlexColumns;

use super::{MIN_WIDE_WIDTH, PLOT_HEIGHT};

/// Rate curve settings for one axis
#[derive(Clone, Default)]
struct RateCurveSettings {
    rc_rate: f32,    // RC Rate (0-255, typically 100-150)
    super_rate: f32, // Super Rate (0-100)
    rc_expo: f32,    // RC Expo (0-100)
    rate_limit: f32, // Max rate limit in deg/s (typically 1998)
}

impl RateCurveSettings {
    /// Calculate rate in deg/s for a given stick position (-1.0 to 1.0)
    /// Using Betaflight ACTUAL rates formula
    fn calculate_rate(&self, stick: f32) -> f32 {
        let stick_abs = stick.abs();

        // Apply expo curve
        let expo_factor = self.rc_expo / 100.0;
        let exp_stick =
            stick_abs * stick_abs * stick_abs * expo_factor + stick_abs * (1.0 - expo_factor);

        // Calculate center sensitivity (rc_rate determines this)
        let center_sensitivity = self.rc_rate / 100.0 * 200.0; // Convert to deg/s at center

        // Calculate max rate (super_rate boosts at full stick)
        let super_factor = self.super_rate / 100.0;
        let max_rate = center_sensitivity + (super_factor * 500.0); // Super rate adds up to 500 deg/s at full stick

        // Blend between center and max based on expo-curved stick
        let rate = center_sensitivity + (max_rate - center_sensitivity) * exp_stick;

        // Apply rate limit
        let limited_rate = rate.min(self.rate_limit);

        // Apply sign
        limited_rate * stick.signum()
    }

    /// Generate rate curve points for plotting
    fn generate_curve(&self, points: usize) -> Vec<[f64; 2]> {
        (0..=points)
            .map(|i| {
                let stick = (i as f32 / points as f32) * 2.0 - 1.0; // -1 to 1
                let rate = self.calculate_rate(stick);
                [stick as f64 * 100.0, rate as f64] // Convert stick to percentage
            })
            .collect()
    }
}

/// Parsed rate curves from flight log
#[derive(Clone, Default)]
struct RateCurves {
    roll: RateCurveSettings,
    pitch: RateCurveSettings,
    yaw: RateCurveSettings,
    rates_type: String,
    valid: bool,
}

impl RateCurves {
    /// Parse rate settings from FlightData headers
    fn parse_from_headers(fd: &FlightData) -> Self {
        let headers = &fd.unknown_headers;

        // Helper to parse header value
        let get_f32 = |key: &str| -> f32 {
            headers
                .get(key)
                .and_then(|v| v.parse::<f32>().ok())
                .unwrap_or(0.0)
        };

        let get_array = |key: &str| -> Vec<f32> {
            headers
                .get(key)
                .map(|v| {
                    v.split(',')
                        .filter_map(|s| s.trim().parse::<f32>().ok())
                        .collect()
                })
                .unwrap_or_default()
        };

        // Try to get rates_type first
        let rates_type = headers
            .get("rates_type")
            .cloned()
            .unwrap_or_else(|| "BETAFLIGHT".to_string());

        // Parse rates - try different header formats
        // Format 1: rollRate, pitchRate, yawRate (older)
        // Format 2: rates[0], rates[1], rates[2] (newer)
        // Format 3: roll_rate, pitch_rate, yaw_rate

        let roll_rate = get_f32("rollRate")
            .max(get_f32("roll_rate"))
            .max(get_array("rates").first().copied().unwrap_or(0.0));
        let pitch_rate = get_f32("pitchRate")
            .max(get_f32("pitch_rate"))
            .max(get_array("rates").get(1).copied().unwrap_or(0.0));
        let yaw_rate = get_f32("yawRate")
            .max(get_f32("yaw_rate"))
            .max(get_array("rates").get(2).copied().unwrap_or(0.0));

        // RC Rate
        let rc_rates = get_array("rc_rates");
        let roll_rc_rate = get_f32("rcRollRate")
            .max(get_f32("rc_roll_rate"))
            .max(rc_rates.first().copied().unwrap_or(0.0));
        let pitch_rc_rate = get_f32("rcPitchRate")
            .max(get_f32("rc_pitch_rate"))
            .max(rc_rates.get(1).copied().unwrap_or(0.0));
        let yaw_rc_rate = get_f32("rcYawRate")
            .max(get_f32("rc_yaw_rate"))
            .max(rc_rates.get(2).copied().unwrap_or(0.0));

        // Expo
        let roll_expo = get_f32("rcExpo")
            .max(get_f32("roll_expo"))
            .max(get_f32("rc_expo"));
        let pitch_expo = get_f32("rcExpo").max(get_f32("pitch_expo"));
        let yaw_expo = get_f32("rcYawExpo").max(get_f32("yaw_expo"));

        // Rate limit (default 1998)
        let rate_limit = get_f32("rateLimit")
            .max(get_array("rateLimits").first().copied().unwrap_or(0.0))
            .max(1998.0);

        let valid = roll_rate > 0.0 || roll_rc_rate > 0.0;

        Self {
            roll: RateCurveSettings {
                rc_rate: roll_rc_rate.max(100.0), // Default to 100 if not set
                super_rate: roll_rate,
                rc_expo: roll_expo,
                rate_limit,
            },
            pitch: RateCurveSettings {
                rc_rate: pitch_rc_rate.max(roll_rc_rate).max(100.0),
                super_rate: pitch_rate.max(roll_rate),
                rc_expo: pitch_expo.max(roll_expo),
                rate_limit,
            },
            yaw: RateCurveSettings {
                rc_rate: yaw_rc_rate.max(roll_rc_rate).max(100.0),
                super_rate: yaw_rate.max(roll_rate * 0.7), // Yaw typically lower
                rc_expo: yaw_expo.max(roll_expo),
                rate_limit,
            },
            rates_type,
            valid,
        }
    }

    /// Check if we have valid rate data
    fn is_valid(&self) -> bool {
        self.valid
    }
}

/// Statistics computed from flight data
struct FlightStats {
    /// RC command histograms: bins from -100% to +100% (or 0-100% for throttle)
    /// Each bin represents 2% range
    rc_roll_hist: Vec<f64>,
    rc_pitch_hist: Vec<f64>,
    rc_yaw_hist: Vec<f64>,
    rc_throttle_hist: Vec<f64>,

    /// PID term statistics: (mean, std_dev) for each axis
    gyro_stats: [(f32, f32); 3],
    p_stats: [(f32, f32); 3],
    i_stats: [(f32, f32); 3],
    d_stats: [(f32, f32); 2], // Roll, Pitch only
    f_stats: [(f32, f32); 3],

    /// Motor statistics: (mean, std_dev) for each motor
    motor_stats: Vec<(f32, f32)>,
}

impl FlightStats {
    fn compute(fd: &FlightData) -> Self {
        // Compute RC command histograms
        let rc_roll_hist =
            Self::compute_histogram(fd.rc_command().map(|r| r[0]), -500.0, 500.0, 50);
        let rc_pitch_hist =
            Self::compute_histogram(fd.rc_command().map(|r| r[1]), -500.0, 500.0, 50);
        let rc_yaw_hist = Self::compute_histogram(fd.rc_command().map(|r| r[2]), -500.0, 500.0, 50);
        let rc_throttle_hist =
            Self::compute_histogram(fd.setpoint().map(|r| r[3]), 0.0, 1000.0, 50);

        // Compute gyro statistics
        let gyro_stats = fd
            .gyro_filtered()
            .map(|gyro| {
                [
                    Self::compute_stats(gyro[0]),
                    Self::compute_stats(gyro[1]),
                    Self::compute_stats(gyro[2]),
                ]
            })
            .unwrap_or([(0.0, 0.0); 3]);

        // Compute P-term statistics
        let p_stats = fd
            .p()
            .map(|p| {
                [
                    Self::compute_stats(p[0]),
                    Self::compute_stats(p[1]),
                    Self::compute_stats(p[2]),
                ]
            })
            .unwrap_or([(0.0, 0.0); 3]);

        // Compute I-term statistics
        let i_stats = fd
            .i()
            .map(|i| {
                [
                    Self::compute_stats(i[0]),
                    Self::compute_stats(i[1]),
                    Self::compute_stats(i[2]),
                ]
            })
            .unwrap_or([(0.0, 0.0); 3]);

        // Compute D-term statistics (Roll, Pitch only)
        let d_terms = fd.d();
        let d_stats = [
            d_terms[0]
                .map(|d| Self::compute_stats(d))
                .unwrap_or((0.0, 0.0)),
            d_terms[1]
                .map(|d| Self::compute_stats(d))
                .unwrap_or((0.0, 0.0)),
        ];

        // Compute F-term statistics
        let f_stats = fd
            .f()
            .map(|f| {
                [
                    Self::compute_stats(f[0]),
                    Self::compute_stats(f[1]),
                    Self::compute_stats(f[2]),
                ]
            })
            .unwrap_or([(0.0, 0.0); 3]);

        // Compute motor statistics
        let motor_stats = fd
            .motor()
            .map(|motors| motors.iter().map(|m| Self::compute_stats(m)).collect())
            .unwrap_or_default();

        Self {
            rc_roll_hist,
            rc_pitch_hist,
            rc_yaw_hist,
            rc_throttle_hist,
            gyro_stats,
            p_stats,
            i_stats,
            d_stats,
            f_stats,
            motor_stats,
        }
    }

    fn compute_histogram(data: Option<&Vec<f32>>, min: f32, max: f32, num_bins: usize) -> Vec<f64> {
        let mut bins = vec![0.0; num_bins];
        let Some(data) = data else {
            return bins;
        };

        if data.is_empty() {
            return bins;
        }

        let bin_width = (max - min) / num_bins as f32;
        let total = data.len() as f64;

        for &val in data {
            let bin_idx = ((val - min) / bin_width).floor() as usize;
            let bin_idx = bin_idx.min(num_bins - 1);
            bins[bin_idx] += 1.0;
        }

        // Normalize to percentage
        for bin in &mut bins {
            *bin = (*bin / total) * 100.0;
        }

        bins
    }

    fn compute_stats(data: &[f32]) -> (f32, f32) {
        if data.is_empty() {
            return (0.0, 0.0);
        }

        let n = data.len() as f32;
        let mean = data.iter().map(|v| v.abs()).sum::<f32>() / n;
        let variance = data.iter().map(|v| (v.abs() - mean).powi(2)).sum::<f32>() / n;
        let std_dev = variance.sqrt();

        (mean, std_dev)
    }
}

pub struct StatsTab {
    #[allow(dead_code)]
    fd: Arc<FlightData>,
    stats: FlightStats,
    rate_curves: RateCurves,
}

impl StatsTab {
    pub fn new(fd: Arc<FlightData>) -> Self {
        let stats = FlightStats::compute(&fd);
        let rate_curves = RateCurves::parse_from_headers(&fd);
        Self {
            fd,
            stats,
            rate_curves,
        }
    }

    pub fn show(&mut self, ui: &mut egui::Ui) {
        let colors = Colors::get(ui);

        egui::ScrollArea::vertical().show(ui, |ui| {
            // RC Command Histograms Section
            ui.heading("Stick Usage Distribution");
            ui.add_space(8.0);

            FlexColumns::new(MIN_WIDE_WIDTH)
                .column(|ui| {
                    self.show_rc_histogram(
                        ui,
                        "Roll",
                        &self.stats.rc_roll_hist,
                        colors.triple_primary[0],
                        true,
                    )
                })
                .column(|ui| {
                    self.show_rc_histogram(
                        ui,
                        "Pitch",
                        &self.stats.rc_pitch_hist,
                        colors.triple_primary[1],
                        true,
                    )
                })
                .show(ui);

            FlexColumns::new(MIN_WIDE_WIDTH)
                .column(|ui| {
                    self.show_rc_histogram(
                        ui,
                        "Yaw",
                        &self.stats.rc_yaw_hist,
                        colors.triple_primary[2],
                        true,
                    )
                })
                .column(|ui| {
                    self.show_rc_histogram(
                        ui,
                        "Throttle",
                        &self.stats.rc_throttle_hist,
                        colors.quad[3],
                        false,
                    )
                })
                .show(ui);

            ui.add_space(16.0);
            ui.separator();
            ui.add_space(8.0);

            // PID Term Statistics Section
            ui.heading("PID Term Statistics");
            ui.add_space(8.0);

            FlexColumns::new(MIN_WIDE_WIDTH)
                .column(|ui| self.show_pid_stats(ui, "|Gyro|", &self.stats.gyro_stats, &colors))
                .column(|ui| self.show_pid_stats(ui, "|P-term|", &self.stats.p_stats, &colors))
                .column(|ui| self.show_pid_stats(ui, "|I-term|", &self.stats.i_stats, &colors))
                .show(ui);

            FlexColumns::new(MIN_WIDE_WIDTH)
                .column(|ui| self.show_d_stats(ui, &colors))
                .column(|ui| self.show_pid_stats(ui, "|F-term|", &self.stats.f_stats, &colors))
                .column(|ui| self.show_motor_stats(ui, &colors))
                .show(ui);

            ui.add_space(16.0);
            ui.separator();
            ui.add_space(8.0);

            // Cross-axis Trajectory Section
            ui.heading("Cross-Axis Trajectory");
            ui.add_space(8.0);

            FlexColumns::new(MIN_WIDE_WIDTH)
                .column(|ui| self.show_trajectory(ui, &colors))
                .show(ui);

            // Rate Curve Visualization Section (if we have rate data)
            if self.rate_curves.is_valid() {
                ui.add_space(16.0);
                ui.separator();
                ui.add_space(8.0);

                ui.heading("Rate Curve Visualization");
                ui.label(format!("Rates Type: {}", self.rate_curves.rates_type));
                ui.add_space(8.0);

                FlexColumns::new(MIN_WIDE_WIDTH)
                    .column(|ui| {
                        self.show_rate_curve(
                            ui,
                            "Roll",
                            &self.rate_curves.roll,
                            colors.triple_primary[0],
                        )
                    })
                    .column(|ui| {
                        self.show_rate_curve(
                            ui,
                            "Pitch",
                            &self.rate_curves.pitch,
                            colors.triple_primary[1],
                        )
                    })
                    .column(|ui| {
                        self.show_rate_curve(
                            ui,
                            "Yaw",
                            &self.rate_curves.yaw,
                            colors.triple_primary[2],
                        )
                    })
                    .show(ui);
            }
        });
    }

    fn show_rc_histogram(
        &self,
        ui: &mut egui::Ui,
        label: &str,
        data: &[f64],
        color: Color32,
        symmetric: bool,
    ) -> egui::Response {
        ui.vertical(|ui| {
            ui.label(RichText::new(label).strong());

            let (min_x, max_x) = if symmetric {
                (-100.0, 100.0)
            } else {
                (0.0, 100.0)
            };

            let bin_width = (max_x - min_x) / data.len() as f64;

            let bars: Vec<Bar> = data
                .iter()
                .enumerate()
                .map(|(i, &val)| {
                    let x = min_x + (i as f64 + 0.5) * bin_width;
                    Bar::new(x, val).width(bin_width * 0.9).fill(color)
                })
                .collect();

            let chart = BarChart::new(bars).name(label);

            Plot::new(format!("rc_hist_{}", label))
                .legend(Legend::default().position(Corner::RightTop))
                .height(PLOT_HEIGHT * 0.7)
                .include_y(0.0)
                .x_axis_label("% Stick")
                .y_axis_label("% of Flight")
                .show(ui, |plot_ui| {
                    plot_ui.bar_chart(chart);
                });
        })
        .response
    }

    fn show_pid_stats(
        &self,
        ui: &mut egui::Ui,
        label: &str,
        stats: &[(f32, f32); 3],
        colors: &Colors,
    ) -> egui::Response {
        ui.vertical(|ui| {
            ui.label(RichText::new(label).strong());

            let axis_labels = ["R", "P", "Y"];
            let axis_colors = [
                colors.triple_primary[0],
                colors.triple_primary[1],
                colors.triple_primary[2],
            ];

            let bars: Vec<Bar> = stats
                .iter()
                .enumerate()
                .map(|(i, &(mean, _std))| {
                    Bar::new(i as f64 + 1.0, mean as f64)
                        .width(0.6)
                        .fill(axis_colors[i])
                        .name(axis_labels[i])
                })
                .collect();

            let chart = BarChart::new(bars);

            // Error bars as lines
            let error_lines: Vec<_> = stats
                .iter()
                .enumerate()
                .map(|(i, &(mean, std))| {
                    let x = i as f64 + 1.0;
                    let points =
                        PlotPoints::new(vec![[x, (mean - std) as f64], [x, (mean + std) as f64]]);
                    Line::new(points)
                        .stroke(Stroke::new(2.0, Color32::BLACK))
                        .name(format!("{}±σ", axis_labels[i]))
                })
                .collect();

            Plot::new(format!("pid_stats_{}", label))
                .height(PLOT_HEIGHT * 0.7)
                .include_y(0.0)
                .x_axis_label("Axis")
                .y_axis_label("Mean |value|")
                .show(ui, |plot_ui| {
                    plot_ui.bar_chart(chart);
                    for line in error_lines {
                        plot_ui.line(line);
                    }
                });
        })
        .response
    }

    fn show_d_stats(&self, ui: &mut egui::Ui, colors: &Colors) -> egui::Response {
        ui.vertical(|ui| {
            ui.label(RichText::new("|D-term|").strong());

            let axis_labels = ["R", "P"];
            let axis_colors = [colors.triple_primary[0], colors.triple_primary[1]];

            let bars: Vec<Bar> = self
                .stats
                .d_stats
                .iter()
                .enumerate()
                .map(|(i, &(mean, _std))| {
                    Bar::new(i as f64 + 1.0, mean as f64)
                        .width(0.6)
                        .fill(axis_colors[i])
                        .name(axis_labels[i])
                })
                .collect();

            let chart = BarChart::new(bars);

            let error_lines: Vec<_> = self
                .stats
                .d_stats
                .iter()
                .enumerate()
                .map(|(i, &(mean, std))| {
                    let x = i as f64 + 1.0;
                    let points =
                        PlotPoints::new(vec![[x, (mean - std) as f64], [x, (mean + std) as f64]]);
                    Line::new(points)
                        .stroke(Stroke::new(2.0, Color32::BLACK))
                        .name(format!("{}±σ", axis_labels[i]))
                })
                .collect();

            Plot::new("pid_stats_d")
                .height(PLOT_HEIGHT * 0.7)
                .include_y(0.0)
                .x_axis_label("Axis")
                .y_axis_label("Mean |value|")
                .show(ui, |plot_ui| {
                    plot_ui.bar_chart(chart);
                    for line in error_lines {
                        plot_ui.line(line);
                    }
                });
        })
        .response
    }

    fn show_motor_stats(&self, ui: &mut egui::Ui, colors: &Colors) -> egui::Response {
        ui.vertical(|ui| {
            ui.label(RichText::new("Motors").strong());

            let motor_colors = &colors.motors;

            let bars: Vec<Bar> = self
                .stats
                .motor_stats
                .iter()
                .enumerate()
                .map(|(i, &(mean, _std))| {
                    Bar::new(i as f64 + 1.0, mean as f64)
                        .width(0.6)
                        .fill(motor_colors[i % motor_colors.len()])
                        .name(format!("M{}", i + 1))
                })
                .collect();

            let chart = BarChart::new(bars);

            let error_lines: Vec<_> = self
                .stats
                .motor_stats
                .iter()
                .enumerate()
                .map(|(i, &(mean, std))| {
                    let x = i as f64 + 1.0;
                    let points =
                        PlotPoints::new(vec![[x, (mean - std) as f64], [x, (mean + std) as f64]]);
                    Line::new(points)
                        .stroke(Stroke::new(2.0, Color32::BLACK))
                        .name(format!("M{}±σ", i + 1))
                })
                .collect();

            Plot::new("motor_stats")
                .height(PLOT_HEIGHT * 0.7)
                .include_y(0.0)
                .x_axis_label("Motor")
                .y_axis_label("Mean Output %")
                .show(ui, |plot_ui| {
                    plot_ui.bar_chart(chart);
                    for line in error_lines {
                        plot_ui.line(line);
                    }
                });
        })
        .response
    }

    /// Show cross-axis trajectory (Roll vs Pitch) with throttle coloring
    fn show_trajectory(&self, ui: &mut egui::Ui, colors: &Colors) -> egui::Response {
        ui.vertical(|ui| {
            ui.label(RichText::new("Roll vs Pitch Trajectory").strong());
            ui.label("Color: Blue=Low Throttle → Red=High Throttle");

            // Get gyro data for Roll and Pitch
            let gyro = self.fd.gyro_filtered();
            let setpoint = self.fd.setpoint();

            if gyro.is_none() {
                ui.label("Gyro data not available");
                return;
            }

            let gyro = gyro.unwrap();
            let roll_data = gyro[0];
            let pitch_data = gyro[1];

            // Subsample for performance (every 10th point)
            let subsample = 10;
            let len = roll_data.len().min(pitch_data.len());

            // Create colored line segments based on throttle
            Plot::new("trajectory_plot")
                .legend(Legend::default().position(Corner::RightTop))
                .height(PLOT_HEIGHT * 1.2)
                .data_aspect(1.0) // Equal scaling for both axes
                .x_axis_label("Roll Rate (deg/s)")
                .y_axis_label("Pitch Rate (deg/s)")
                .show(ui, |plot_ui| {
                    // Draw segments with color based on throttle
                    if let Some(sp) = setpoint {
                        let throttle_data = sp[3];
                        let max_throttle = throttle_data.iter().cloned().fold(0.0f32, f32::max);
                        let min_throttle = throttle_data.iter().cloned().fold(f32::MAX, f32::min);
                        let throttle_range = (max_throttle - min_throttle).max(1.0);

                        // Create segments with different colors
                        for i in (0..len.saturating_sub(1)).step_by(subsample) {
                            let thr_pct = if throttle_range > 0.0 {
                                (throttle_data[i] - min_throttle) / throttle_range
                            } else {
                                0.5
                            };

                            // Color gradient: blue (low throttle) -> red (high throttle)
                            let r = (thr_pct * 255.0) as u8;
                            let b = ((1.0 - thr_pct) * 255.0) as u8;
                            let color = Color32::from_rgb(r, 50, b);

                            let next_i = (i + subsample).min(len - 1);
                            let points = PlotPoints::new(vec![
                                [roll_data[i] as f64, pitch_data[i] as f64],
                                [roll_data[next_i] as f64, pitch_data[next_i] as f64],
                            ]);
                            plot_ui.line(Line::new(points).stroke(Stroke::new(1.5, color)));
                        }
                    } else {
                        // No throttle data, draw single color
                        let points: Vec<[f64; 2]> = (0..len)
                            .step_by(subsample)
                            .map(|i| [roll_data[i] as f64, pitch_data[i] as f64])
                            .collect();
                        plot_ui.line(
                            Line::new(PlotPoints::new(points))
                                .color(colors.triple_primary[0])
                                .name("Trajectory"),
                        );
                    }

                    // Add center reference point
                    plot_ui.points(
                        egui_plot::Points::new(vec![[0.0, 0.0]])
                            .radius(5.0)
                            .color(Color32::WHITE)
                            .name("Center"),
                    );
                });
        })
        .response
    }

    /// Display a rate curve plot for one axis
    fn show_rate_curve(
        &self,
        ui: &mut egui::Ui,
        label: &str,
        settings: &RateCurveSettings,
        color: Color32,
    ) -> egui::Response {
        ui.vertical(|ui| {
            ui.label(RichText::new(label).strong());

            // Show settings info
            ui.horizontal(|ui| {
                ui.label(format!(
                    "RC Rate: {:.0} | Super Rate: {:.0} | Expo: {:.0}",
                    settings.rc_rate, settings.super_rate, settings.rc_expo
                ));
            });

            // Calculate max rate at full stick for label
            let max_rate = settings.calculate_rate(1.0);
            ui.label(format!("Max Rate: {:.0}°/s", max_rate));

            // Generate curve points
            let curve_points = settings.generate_curve(100);

            Plot::new(format!("rate_curve_{}", label))
                .height(PLOT_HEIGHT)
                .x_axis_label("Stick %")
                .y_axis_label("Rate (°/s)")
                .include_x(-100.0)
                .include_x(100.0)
                .include_y(0.0)
                .include_y(max_rate as f64 * 1.1)
                .legend(Legend::default().position(Corner::LeftTop))
                .show(ui, |plot_ui| {
                    // Main rate curve
                    let line = Line::new(PlotPoints::new(curve_points.clone()))
                        .stroke(Stroke::new(2.0, color))
                        .name(format!("{} Rate", label));
                    plot_ui.line(line);

                    // Add horizontal reference lines at common max rates
                    for &ref_rate in &[500.0, 1000.0, 1500.0, 2000.0] {
                        if ref_rate < max_rate as f64 * 1.1 {
                            let ref_line = Line::new(PlotPoints::new(vec![
                                [-100.0, ref_rate],
                                [100.0, ref_rate],
                            ]))
                            .stroke(Stroke::new(0.5, Color32::from_gray(100)))
                            .name(format!("{}°/s", ref_rate));
                            plot_ui.line(ref_line);
                        }
                    }

                    // Add vertical center line
                    let center_line = Line::new(PlotPoints::new(vec![
                        [0.0, 0.0],
                        [0.0, max_rate as f64 * 1.1],
                    ]))
                    .stroke(Stroke::new(0.5, Color32::from_gray(100)));
                    plot_ui.line(center_line);
                });
        })
        .response
    }
}

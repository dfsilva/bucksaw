use std::sync::Arc;

use egui::{Color32, RichText, Stroke};
use egui_plot::{Bar, BarChart, Corner, Legend, Line, Plot, PlotPoints};

use crate::flight_data::FlightData;
use crate::gui::colors::Colors;
use crate::gui::flex::FlexColumns;

use super::{MIN_WIDE_WIDTH, PLOT_HEIGHT};

/// PID Error statistics and analysis data
struct PIDErrorStats {
    /// Error distribution histograms for each axis (normalized)
    roll_error_hist: Vec<f64>,
    pitch_error_hist: Vec<f64>,
    yaw_error_hist: Vec<f64>,

    /// Mean absolute error at each stick deflection level (10% buckets)
    roll_error_vs_stick: Vec<(f64, f64, f64)>, // (stick%, mean_error, std_error)
    pitch_error_vs_stick: Vec<(f64, f64, f64)>,
    yaw_error_vs_stick: Vec<(f64, f64, f64)>,

    /// Overall statistics
    roll_stats: (f32, f32), // (mean_abs_error, std)
    pitch_stats: (f32, f32),
    yaw_stats: (f32, f32),
}

impl PIDErrorStats {
    /// Default maximum deg/s threshold for filtering outliers (like PIDtoolbox)
    const DEFAULT_MAX_DEGSEC: f32 = 500.0;

    fn compute(fd: &FlightData) -> Option<Self> {
        Self::compute_with_threshold(fd, Self::DEFAULT_MAX_DEGSEC)
    }

    fn compute_with_threshold(fd: &FlightData, max_degsec: f32) -> Option<Self> {
        let setpoints = fd.setpoint()?;
        let gyro = fd.gyro_filtered()?;
        let rc_commands = fd.rc_command()?;

        // Compute PID errors (setpoint - gyro)
        let roll_errors: Vec<f32> = setpoints[0]
            .iter()
            .zip(gyro[0].iter())
            .map(|(sp, gy)| sp - gy)
            .collect();

        let pitch_errors: Vec<f32> = setpoints[1]
            .iter()
            .zip(gyro[1].iter())
            .map(|(sp, gy)| sp - gy)
            .collect();

        let yaw_errors: Vec<f32> = setpoints[2]
            .iter()
            .zip(gyro[2].iter())
            .map(|(sp, gy)| sp - gy)
            .collect();

        // Filter to samples where all errors are within threshold (like PIDtoolbox)
        let valid_mask: Vec<bool> = roll_errors
            .iter()
            .zip(pitch_errors.iter())
            .zip(yaw_errors.iter())
            .map(|((&r, &p), &y)| {
                r.abs() < max_degsec && p.abs() < max_degsec && y.abs() < max_degsec
            })
            .collect();

        let roll_filtered: Vec<f32> = roll_errors
            .iter()
            .zip(&valid_mask)
            .filter(|(_, &valid)| valid)
            .map(|(&e, _)| e)
            .collect();
        let pitch_filtered: Vec<f32> = pitch_errors
            .iter()
            .zip(&valid_mask)
            .filter(|(_, &valid)| valid)
            .map(|(&e, _)| e)
            .collect();
        let yaw_filtered: Vec<f32> = yaw_errors
            .iter()
            .zip(&valid_mask)
            .filter(|(_, &valid)| valid)
            .map(|(&e, _)| e)
            .collect();

        // Compute error distribution histograms (normalized to peak = 1.0 like PIDtoolbox)
        let roll_error_hist = Self::compute_histogram(&roll_filtered, -max_degsec, max_degsec, 100);
        let pitch_error_hist =
            Self::compute_histogram(&pitch_filtered, -max_degsec, max_degsec, 100);
        let yaw_error_hist = Self::compute_histogram(&yaw_filtered, -max_degsec, max_degsec, 100);

        // Compute error vs stick deflection
        let roll_rc: Vec<f32> = rc_commands[0]
            .iter()
            .zip(&valid_mask)
            .filter(|(_, &valid)| valid)
            .map(|(&v, _)| v)
            .collect();
        let pitch_rc: Vec<f32> = rc_commands[1]
            .iter()
            .zip(&valid_mask)
            .filter(|(_, &valid)| valid)
            .map(|(&v, _)| v)
            .collect();
        let yaw_rc: Vec<f32> = rc_commands[2]
            .iter()
            .zip(&valid_mask)
            .filter(|(_, &valid)| valid)
            .map(|(&v, _)| v)
            .collect();

        let roll_error_vs_stick = Self::compute_error_vs_stick(&roll_filtered, &roll_rc, 10);
        let pitch_error_vs_stick = Self::compute_error_vs_stick(&pitch_filtered, &pitch_rc, 10);
        let yaw_error_vs_stick = Self::compute_error_vs_stick(&yaw_filtered, &yaw_rc, 10);

        // Compute overall statistics
        let roll_stats = Self::compute_stats(&roll_filtered);
        let pitch_stats = Self::compute_stats(&pitch_filtered);
        let yaw_stats = Self::compute_stats(&yaw_filtered);

        Some(Self {
            roll_error_hist,
            pitch_error_hist,
            yaw_error_hist,
            roll_error_vs_stick,
            pitch_error_vs_stick,
            yaw_error_vs_stick,
            roll_stats,
            pitch_stats,
            yaw_stats,
        })
    }

    fn compute_histogram(data: &[f32], min: f32, max: f32, num_bins: usize) -> Vec<f64> {
        let mut bins = vec![0.0; num_bins];

        if data.is_empty() {
            return bins;
        }

        let bin_width = (max - min) / num_bins as f32;

        for &val in data {
            // Clamp to range
            let clamped = val.clamp(min, max - 0.001);
            let bin_idx = ((clamped - min) / bin_width).floor() as usize;
            let bin_idx = bin_idx.min(num_bins - 1);
            bins[bin_idx] += 1.0;
        }

        // Normalize to peak = 1.0 (like PIDtoolbox: yA/max(yA))
        let max_val = bins.iter().cloned().fold(0.0f64, f64::max);
        if max_val > 0.0 {
            for bin in &mut bins {
                *bin /= max_val;
            }
        }

        bins
    }

    fn compute_error_vs_stick(
        errors: &[f32],
        stick: &[f32],
        num_buckets: usize,
    ) -> Vec<(f64, f64, f64)> {
        // Bucket by stick deflection percentage (0-100%)
        let mut buckets: Vec<Vec<f32>> = vec![vec![]; num_buckets];

        // Find max stick deflection for percentage calculation (like PIDtoolbox)
        let max_stick = stick.iter().map(|v| v.abs()).fold(0.0f32, f32::max);
        if max_stick < 1.0 {
            return (0..num_buckets)
                .map(|i| ((i as f64 + 0.5) * (100.0 / num_buckets as f64), 0.0, 0.0))
                .collect();
        }

        for (error, &stick_val) in errors.iter().zip(stick.iter()) {
            let stick_pct = (stick_val.abs() / max_stick * 100.0).clamp(0.0, 99.9);
            let bucket_idx = (stick_pct / (100.0 / num_buckets as f32)).floor() as usize;
            let bucket_idx = bucket_idx.min(num_buckets - 1);
            buckets[bucket_idx].push(error.abs());
        }

        // Compute mean and STANDARD ERROR (std/sqrt(n)) like PIDtoolbox
        buckets
            .iter()
            .enumerate()
            .map(|(i, bucket)| {
                let stick_center = (i as f64 + 0.5) * (100.0 / num_buckets as f64);
                if bucket.is_empty() {
                    (stick_center, 0.0, 0.0)
                } else {
                    let n = bucket.len() as f32;
                    let mean = bucket.iter().sum::<f32>() / n;
                    let variance = bucket.iter().map(|e| (e - mean).powi(2)).sum::<f32>() / n;
                    let std = variance.sqrt();
                    // Standard error = std / sqrt(n) (like PIDtoolbox)
                    let std_error = std / n.sqrt();
                    (stick_center, mean as f64, std_error as f64)
                }
            })
            .collect()
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

pub struct ErrorTab {
    #[allow(dead_code)]
    fd: Arc<FlightData>,
    stats: Option<PIDErrorStats>,
}

impl ErrorTab {
    pub fn new(fd: Arc<FlightData>) -> Self {
        let stats = PIDErrorStats::compute(&fd);
        Self { fd, stats }
    }

    pub fn show(&mut self, ui: &mut egui::Ui) {
        let Some(stats) = &self.stats else {
            ui.label("PID error data not available. Ensure setpoint and gyro data are in the log.");
            return;
        };

        let colors = Colors::get(ui);

        egui::ScrollArea::vertical().show(ui, |ui| {
            // Section 1: Error Distribution Histograms
            ui.heading("PID Error Distributions");
            ui.label("Shows how tracking errors are distributed. Narrower = better tracking.");
            ui.add_space(8.0);

            FlexColumns::new(MIN_WIDE_WIDTH)
                .column(|ui| {
                    self.show_error_histogram(
                        ui,
                        "Roll Error",
                        &stats.roll_error_hist,
                        colors.triple_primary[0],
                        stats.roll_stats,
                    )
                })
                .column(|ui| {
                    self.show_error_histogram(
                        ui,
                        "Pitch Error",
                        &stats.pitch_error_hist,
                        colors.triple_primary[1],
                        stats.pitch_stats,
                    )
                })
                .column(|ui| {
                    self.show_error_histogram(
                        ui,
                        "Yaw Error",
                        &stats.yaw_error_hist,
                        colors.triple_primary[2],
                        stats.yaw_stats,
                    )
                })
                .show(ui);

            ui.add_space(16.0);
            ui.separator();
            ui.add_space(8.0);

            // Section 2: Error vs Stick Deflection
            ui.heading("Error vs Stick Deflection");
            ui.label("Shows mean absolute error at different stick positions. Lower = better.");
            ui.add_space(8.0);

            FlexColumns::new(MIN_WIDE_WIDTH)
                .column(|ui| {
                    self.show_error_vs_stick(
                        ui,
                        "Roll",
                        &stats.roll_error_vs_stick,
                        colors.triple_primary[0],
                    )
                })
                .column(|ui| {
                    self.show_error_vs_stick(
                        ui,
                        "Pitch",
                        &stats.pitch_error_vs_stick,
                        colors.triple_primary[1],
                    )
                })
                .column(|ui| {
                    self.show_error_vs_stick(
                        ui,
                        "Yaw",
                        &stats.yaw_error_vs_stick,
                        colors.triple_primary[2],
                    )
                })
                .show(ui);

            ui.add_space(16.0);
            ui.separator();
            ui.add_space(8.0);

            // Section 3: Summary Statistics
            ui.heading("Summary");
            ui.add_space(4.0);

            egui::Grid::new("error_summary")
                .num_columns(4)
                .spacing([20.0, 4.0])
                .show(ui, |ui| {
                    ui.label("");
                    ui.label(RichText::new("Roll").strong());
                    ui.label(RichText::new("Pitch").strong());
                    ui.label(RichText::new("Yaw").strong());
                    ui.end_row();

                    ui.label("Mean |Error| (deg/s):");
                    ui.label(format!("{:.1}", stats.roll_stats.0));
                    ui.label(format!("{:.1}", stats.pitch_stats.0));
                    ui.label(format!("{:.1}", stats.yaw_stats.0));
                    ui.end_row();

                    ui.label("Std Dev (deg/s):");
                    ui.label(format!("{:.1}", stats.roll_stats.1));
                    ui.label(format!("{:.1}", stats.pitch_stats.1));
                    ui.label(format!("{:.1}", stats.yaw_stats.1));
                    ui.end_row();
                });
        });
    }

    fn show_error_histogram(
        &self,
        ui: &mut egui::Ui,
        label: &str,
        data: &[f64],
        color: Color32,
        stats: (f32, f32),
    ) -> egui::Response {
        ui.vertical(|ui| {
            ui.label(RichText::new(label).strong());
            ui.label(format!("μ={:.1} σ={:.1} deg/s", stats.0, stats.1));

            let min_x = -100.0;
            let max_x = 100.0;
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

            Plot::new(format!("error_hist_{}", label))
                .legend(Legend::default().position(Corner::RightTop))
                .height(PLOT_HEIGHT * 0.7)
                .include_y(0.0)
                .x_axis_label("Error (deg/s)")
                .y_axis_label("Normalized Freq")
                .show(ui, |plot_ui| {
                    plot_ui.bar_chart(chart);
                });
        })
        .response
    }

    fn show_error_vs_stick(
        &self,
        ui: &mut egui::Ui,
        label: &str,
        data: &[(f64, f64, f64)],
        color: Color32,
    ) -> egui::Response {
        ui.vertical(|ui| {
            ui.label(RichText::new(label).strong());

            let bars: Vec<Bar> = data
                .iter()
                .map(|&(x, mean, _std)| Bar::new(x, mean).width(8.0).fill(color))
                .collect();

            let chart = BarChart::new(bars).name(format!("{} Error", label));

            // Error bars
            let error_lines: Vec<Line> = data
                .iter()
                .map(|&(x, mean, std)| {
                    let points = PlotPoints::new(vec![[x, mean - std], [x, mean + std]]);
                    Line::new(points).stroke(Stroke::new(2.0, Color32::DARK_GRAY))
                })
                .collect();

            Plot::new(format!("error_vs_stick_{}", label))
                .legend(Legend::default().position(Corner::RightTop))
                .height(PLOT_HEIGHT * 0.7)
                .include_y(0.0)
                .include_x(0.0)
                .include_x(100.0)
                .x_axis_label("Stick Deflection %")
                .y_axis_label("Mean |Error| (deg/s)")
                .show(ui, |plot_ui| {
                    plot_ui.bar_chart(chart);
                    for line in error_lines {
                        plot_ui.line(line);
                    }
                });
        })
        .response
    }
}

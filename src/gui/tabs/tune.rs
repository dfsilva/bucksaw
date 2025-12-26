use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;

use egui_oszi::{TimeseriesGroup, TimeseriesLine, TimeseriesPlot, TimeseriesPlotMemory};
use egui_plot::{Corner, Legend, PlotPoints};

use crate::analytics;
use crate::flight_data::FlightData;
use crate::gui::colors::Colors;
use crate::gui::flex::FlexColumns;
use crate::step_response::{
    calculate_step_response_with_config, generate_ideal_step_response, FilterType, SmoothingLevel,
    StepResponseConfig, StepResponseMetrics,
};
use crate::utils::execute_in_background;

use super::{MIN_WIDE_WIDTH, PLOT_HEIGHT};

struct StepResponses {
    roll_step_response: Vec<(f64, f64)>,
    pitch_step_response: Vec<(f64, f64)>,
    yaw_step_response: Vec<(f64, f64)>,
    // Metrics for each axis
    roll_metrics: StepResponseMetrics,
    pitch_metrics: StepResponseMetrics,
    yaw_metrics: StepResponseMetrics,
}

pub struct TuneTab {
    roll_plot: TimeseriesPlotMemory<f64, f32>,
    pitch_plot: TimeseriesPlotMemory<f64, f32>,
    yaw_plot: TimeseriesPlotMemory<f64, f32>,
    fd: Arc<FlightData>,
    step_responses: Option<StepResponses>,
    receiver: Option<Receiver<StepResponses>>,
    smoothing: SmoothingLevel,
    /// Minimum input threshold in deg/s for segment filtering
    min_input_threshold: f32,
    y_correction: f64,
    y_scale: f64,
    /// Overlay all axes on one plot for comparison
    overlay_mode: bool,
    /// Show ideal first-order response curve
    show_ideal_response: bool,
    /// Time constant for ideal response (ms)
    ideal_time_constant_ms: f64,
    /// Filter type for ideal response (PT1, PT2, PT3 - from Betaflight)
    ideal_filter_type: FilterType,
    /// Show step response metrics panel
    show_metrics: bool,
}

const AXIS_LABELS: [&str; 3] = ["Roll", "Pitch", "Yaw"];
const AXIS_COLORS: [[u8; 3]; 3] = [
    [0xfb, 0x49, 0x34], // Red for Roll
    [0x8e, 0xc0, 0x7c], // Green for Pitch
    [0x83, 0xa5, 0x98], // Blue/Cyan for Yaw
];

impl TuneTab {
    pub fn new(fd: Arc<FlightData>) -> Self {
        // calculate step response in background thread
        let (sender, receiver) = channel();

        Self::calculate_responses(fd.clone(), sender, SmoothingLevel::default(), 20.0);
        Self {
            roll_plot: TimeseriesPlotMemory::new("roll"),
            pitch_plot: TimeseriesPlotMemory::new("pitch"),
            yaw_plot: TimeseriesPlotMemory::new("yaw"),
            step_responses: None,
            receiver: Some(receiver),
            fd,
            smoothing: SmoothingLevel::default(),
            min_input_threshold: 20.0,
            y_correction: 0.0,
            y_scale: 1.5,
            overlay_mode: false,
            show_ideal_response: true,
            ideal_time_constant_ms: 20.0,
            ideal_filter_type: FilterType::default(),
            show_metrics: true,
        }
    }

    fn calculate_responses(
        fd: Arc<FlightData>,
        sender: Sender<StepResponses>,
        smoothing: SmoothingLevel,
        min_input_threshold: f32,
    ) {
        execute_in_background(async move {
            let empty_fallback = Vec::new();
            let setpoints = fd.setpoint().unwrap_or([&empty_fallback; 4]);
            let gyro = fd.gyro_filtered().unwrap_or([&empty_fallback; 3]);
            let sample_rate = fd.sample_rate();

            let config = StepResponseConfig {
                min_input_threshold,
                ..Default::default()
            };

            let roll_step_response = calculate_step_response_with_config(
                &fd.times,
                setpoints[0],
                gyro[0],
                sample_rate,
                smoothing,
                config,
            );
            let pitch_step_response = calculate_step_response_with_config(
                &fd.times,
                setpoints[1],
                gyro[1],
                sample_rate,
                smoothing,
                config,
            );
            let yaw_step_response = calculate_step_response_with_config(
                &fd.times,
                setpoints[2],
                gyro[2],
                sample_rate,
                smoothing,
                config,
            );

            // Calculate metrics for each axis
            let roll_metrics = StepResponseMetrics::analyze(&roll_step_response);
            let pitch_metrics = StepResponseMetrics::analyze(&pitch_step_response);
            let yaw_metrics = StepResponseMetrics::analyze(&yaw_step_response);

            let _ = sender.send(StepResponses {
                roll_step_response,
                pitch_step_response,
                yaw_step_response,
                roll_metrics,
                pitch_metrics,
                yaw_metrics,
            });
        });
    }

    fn recalculate(&mut self) {
        let (sender, receiver) = channel();
        self.receiver = Some(receiver);
        Self::calculate_responses(
            self.fd.clone(),
            sender,
            self.smoothing,
            self.min_input_threshold,
        );
    }

    fn check_receiver(&mut self) {
        if let Some(receiver) = &self.receiver {
            if let Ok(result) = receiver.try_recv() {
                self.step_responses = Some(result);
                self.receiver = None;
            }
        }
    }

    pub fn plot_step_response(
        ui: &mut egui::Ui,
        i: usize,
        step_response: &[(f64, f64)],
        total_width: f32,
        y_correction: f64,
        y_scale: f64,
        show_ideal: bool,
        ideal_time_constant_ms: f64,
        ideal_filter_type: FilterType,
        axis_color: Option<egui::Color32>,
    ) -> egui::Response {
        let height = if ui.available_width() < total_width {
            ui.available_height() / (3 - i) as f32
        } else {
            PLOT_HEIGHT
        };

        // Get max X value for ideal response
        let max_x = step_response.iter().map(|(x, _)| *x).fold(0.0f64, f64::max);

        egui_plot::Plot::new(ui.next_auto_id())
            .legend(Legend::default().position(Corner::RightBottom))
            .set_margin_fraction(egui::Vec2::new(0.0, 0.1))
            .show_grid(true)
            .allow_drag(false)
            .allow_zoom(false)
            .allow_scroll(false)
            .link_axis("global_timeseries", true, false)
            .link_cursor("global_timeseries", true, true)
            .y_axis_position(egui_plot::HPlacement::Right)
            .y_axis_width(3)
            .height(height)
            .include_y(-y_scale + 1.0)
            .include_y(y_scale + 1.0)
            .show(ui, |plot_ui| {
                // Show ideal response curve first (behind actual data)
                // Uses Betaflight-style filter simulation (PT1/PT2/PT3)
                if show_ideal && max_x > 0.0 {
                    // Use 1000 Hz as reference sample rate for smooth curves
                    let ideal = generate_ideal_step_response(
                        ideal_filter_type,
                        ideal_time_constant_ms,
                        max_x,
                        1000.0,
                    );
                    let ideal_points =
                        PlotPoints::new(ideal.iter().map(|(x, y)| [*x, *y]).collect());
                    let ideal_line = egui_plot::Line::new(ideal_points)
                        .name(format!("Ideal {} τ={}ms", ideal_filter_type, ideal_time_constant_ms as i32))
                        .color(egui::Color32::from_rgba_unmultiplied(0x83, 0xa5, 0x98, 180))
                        .width(1.5)
                        .style(egui_plot::LineStyle::dashed_loose());
                    plot_ui.line(ideal_line);
                }

                // Apply Y correction offset to each point
                let points = PlotPoints::new(
                    step_response
                        .iter()
                        .map(|(x, y)| [*x, *y + y_correction])
                        .collect(),
                );
                let color = axis_color.unwrap_or(egui::Color32::from_rgb(0xaf, 0x3a, 0x03));
                let egui_line = egui_plot::Line::new(points)
                    .name(format!("Step Response ({})", AXIS_LABELS[i]))
                    .color(color)
                    .width(2.0);
                plot_ui.line(egui_line);
            })
            .response
    }

    /// Render the step response metrics panel
    fn show_metrics_panel(ui: &mut egui::Ui, responses: &StepResponses) {
        let metrics = [
            ("Roll", &responses.roll_metrics, AXIS_COLORS[0]),
            ("Pitch", &responses.pitch_metrics, AXIS_COLORS[1]),
            ("Yaw", &responses.yaw_metrics, AXIS_COLORS[2]),
        ];

        egui::Grid::new("step_response_metrics")
            .striped(true)
            .min_col_width(60.0)
            .show(ui, |ui| {
                // Header row
                ui.label("");
                for (name, _, color) in &metrics {
                    ui.colored_label(
                        egui::Color32::from_rgb(color[0], color[1], color[2]),
                        egui::RichText::new(*name).strong(),
                    );
                }
                ui.end_row();

                // Rise time (10% to 90%)
                ui.label("Rise Time:")
                    .on_hover_text("Time to reach 90% of steady-state (faster = more responsive)");
                for (_, m, _) in &metrics {
                    ui.label(format!("{:.1} ms", m.rise_time_ms));
                }
                ui.end_row();

                // Overshoot
                ui.label("Overshoot:")
                    .on_hover_text("Maximum overshoot above target (lower = less oscillation)");
                for (_, m, color) in &metrics {
                    let text = format!("{:.1}%", m.overshoot_pct);
                    let text_color = if m.overshoot_pct > 20.0 {
                        egui::Color32::from_rgb(0xfb, 0x49, 0x34) // Red for high overshoot
                    } else if m.overshoot_pct > 10.0 {
                        egui::Color32::from_rgb(0xfa, 0xbd, 0x2f) // Yellow for moderate
                    } else {
                        egui::Color32::from_rgb(color[0], color[1], color[2])
                    };
                    ui.colored_label(text_color, text);
                }
                ui.end_row();

                // Settling time
                ui.label("Settling:")
                    .on_hover_text("Time to stay within 2% of target (faster = better tracking)");
                for (_, m, _) in &metrics {
                    ui.label(format!("{:.0} ms", m.settling_time_ms));
                }
                ui.end_row();

                // Peak time
                ui.label("Peak Time:")
                    .on_hover_text("Time to reach maximum response");
                for (_, m, _) in &metrics {
                    ui.label(format!("{:.1} ms", m.peak_time_ms));
                }
                ui.end_row();

                // Steady-state error
                ui.label("SS Error:")
                    .on_hover_text("Steady-state error from 1.0 (lower = better accuracy)");
                for (_, m, color) in &metrics {
                    let text = format!("{:.2}%", m.steady_state_error * 100.0);
                    let text_color = if m.steady_state_error > 0.05 {
                        egui::Color32::from_rgb(0xfb, 0x49, 0x34) // Red for high error
                    } else {
                        egui::Color32::from_rgb(color[0], color[1], color[2])
                    };
                    ui.colored_label(text_color, text);
                }
                ui.end_row();

                // Oscillation indicator
                ui.label("Oscillations:")
                    .on_hover_text("Indicates if sustained oscillations are present");
                for (_, m, _) in &metrics {
                    if m.has_oscillations {
                        ui.colored_label(
                            egui::Color32::from_rgb(0xfb, 0x49, 0x34),
                            "⚠ Yes",
                        );
                    } else {
                        ui.colored_label(
                            egui::Color32::from_rgb(0x8e, 0xc0, 0x7c),
                            "✓ No",
                        );
                    }
                }
                ui.end_row();
            });
    }

    pub fn show(&mut self, ui: &mut egui::Ui, timeseries_group: &mut TimeseriesGroup) {
        // Check for pending recalculation results
        self.check_receiver();

        // Show smoothing controls first (before FlexColumns to avoid borrow issues)
        let mut should_recalculate = false;
        ui.horizontal(|ui| {
            ui.label("Step Response Smoothing:");
            let prev_smoothing = self.smoothing;
            egui::ComboBox::from_id_source("smoothing_select")
                .selected_text(self.smoothing.to_string())
                .show_ui(ui, |ui| {
                    for level in SmoothingLevel::ALL {
                        ui.selectable_value(&mut self.smoothing, level, level.to_string());
                    }
                });
            if self.smoothing != prev_smoothing {
                should_recalculate = true;
                analytics::log_step_response_settings(&self.smoothing.to_string(), self.min_input_threshold);
            }

            ui.separator();

            // Min Input threshold control
            ui.label("Min Input:");
            let prev_min_input = self.min_input_threshold;
            ui.add(
                egui::Slider::new(&mut self.min_input_threshold, 10.0..=100.0)
                    .suffix("°/s")
                    .max_decimals(0),
            )
            .on_hover_text("Minimum setpoint threshold (deg/s) for segment filtering. Lower values include more data, higher values filter noisy segments.");
            if (self.min_input_threshold - prev_min_input).abs() > 0.1 {
                should_recalculate = true;
            }

            ui.separator();

            ui.label("Y Correction:");
            ui.add(egui::Slider::new(&mut self.y_correction, -0.5..=0.5).step_by(0.01));
            if ui
                .button("Auto")
                .on_hover_text("Automatically adjust Y offset to center at 1.0")
                .clicked()
            {
                // Calculate auto Y correction from step response data
                if let Some(responses) = &self.step_responses {
                    let mut total_correction = 0.0;
                    let mut count = 0;

                    for response in [
                        &responses.roll_step_response,
                        &responses.pitch_step_response,
                        &responses.yaw_step_response,
                    ] {
                        if response.len() > 10 {
                            // Use the steady-state portion (last 30% of the response)
                            let steady_start = (response.len() as f64 * 0.7) as usize;
                            let steady_values: Vec<f64> =
                                response[steady_start..].iter().map(|(_, y)| *y).collect();
                            if !steady_values.is_empty() {
                                let mean: f64 =
                                    steady_values.iter().sum::<f64>() / steady_values.len() as f64;
                                total_correction += 1.0 - mean; // Offset needed to bring mean to 1.0
                                count += 1;
                            }
                        }
                    }

                    if count > 0 {
                        self.y_correction = total_correction / count as f64;
                        // Clamp to valid range
                        self.y_correction = self.y_correction.clamp(-0.5, 0.5);
                    }
                }
            }

            ui.separator();

            ui.label("Y Scale:");
            ui.add(egui::Slider::new(&mut self.y_scale, 0.3..=2.0).step_by(0.1));

            ui.separator();

            // Overlay mode toggle
            ui.checkbox(&mut self.overlay_mode, "= Overlay")
                .on_hover_text("Compare all axes on a single plot");

            // Metrics panel toggle
            ui.checkbox(&mut self.show_metrics, "# Metrics")
                .on_hover_text("Show step response analysis metrics (rise time, overshoot, etc.)");

            // Ideal response controls
            ui.checkbox(&mut self.show_ideal_response, "~ Ideal")
                .on_hover_text("Show ideal step response curve based on Betaflight filter models");

            if self.show_ideal_response {
                // Filter type selector (PT1/PT2/PT3 from Betaflight)
                egui::ComboBox::from_id_source("filter_type_select")
                    .width(90.0)
                    .selected_text(format!("{}", self.ideal_filter_type))
                    .show_ui(ui, |ui| {
                        for ft in FilterType::ALL {
                            ui.selectable_value(&mut self.ideal_filter_type, ft, ft.to_string());
                        }
                    })
                    .response
                    .on_hover_text("Filter type for ideal response (matches Betaflight D-term lowpass types)");

                ui.label("τ:");
                ui.add(
                    egui::Slider::new(&mut self.ideal_time_constant_ms, 5.0..=50.0)
                        .suffix("ms")
                        .max_decimals(0),
                )
                .on_hover_text("Time constant (τ) for ideal response - time to reach 63.2% of step input");
            }
        });

        if should_recalculate {
            self.recalculate();
        }

        ui.separator();

        if let Some(step_responses) = &self.step_responses {
            let total_width = ui.available_width();
            let times = &self.fd.times;
            let colors = Colors::get(ui);

            // Clone step responses to avoid borrow issues
            let roll_response = step_responses.roll_step_response.clone();
            let pitch_response = step_responses.pitch_step_response.clone();
            let yaw_response = step_responses.yaw_step_response.clone();

            FlexColumns::new(MIN_WIDE_WIDTH)
                .column(|ui| {
                    ui.vertical(|ui| {
                        ui.heading("Time Domain");

                        let axes = [
                            &mut self.roll_plot,
                            &mut self.pitch_plot,
                            &mut self.yaw_plot,
                        ];
                        for (i, plot) in axes.into_iter().enumerate() {
                            let height = if ui.available_width() < total_width {
                                ui.available_height() / (3 - i) as f32
                            } else {
                                PLOT_HEIGHT
                            };

                            let label = AXIS_LABELS[i];
                            ui.add(
                                TimeseriesPlot::new(plot)
                                    .group(timeseries_group)
                                    .legend(Legend::default().position(Corner::LeftTop))
                                    .height(height)
                                    .line(
                                        TimeseriesLine::new(format!("Gyro ({}, unfilt.)", label))
                                            .color(colors.gyro_unfiltered),
                                        times.iter().copied().zip(
                                            self.fd
                                                .gyro_unfiltered()
                                                .map(|s| s[i].iter().copied())
                                                .unwrap_or_default(),
                                        ),
                                    )
                                    .line(
                                        TimeseriesLine::new(format!("Gyro ({})", label))
                                            .color(colors.gyro_filtered),
                                        times.iter().copied().zip(
                                            self.fd
                                                .gyro_filtered()
                                                .map(|s| s[i].iter().copied())
                                                .unwrap_or_default(),
                                        ),
                                    )
                                    .line(
                                        TimeseriesLine::new(format!("Setpoint ({})", label))
                                            .color(colors.setpoint),
                                        times.iter().copied().zip(
                                            self.fd
                                                .setpoint()
                                                .map(|s| s[i].iter().copied())
                                                .unwrap_or_default(),
                                        ),
                                    )
                                    .line(
                                        TimeseriesLine::new(format!("P ({})", label))
                                            .color(colors.p),
                                        times.iter().copied().zip(
                                            self.fd
                                                .p()
                                                .map(|s| s[i].iter().copied())
                                                .unwrap_or_default(),
                                        ),
                                    )
                                    .line(
                                        TimeseriesLine::new(format!("I ({})", label))
                                            .color(colors.i),
                                        times.iter().copied().zip(
                                            self.fd
                                                .i()
                                                .map(|s| s[i].iter().copied())
                                                .unwrap_or_default(),
                                        ),
                                    )
                                    .line(
                                        TimeseriesLine::new(format!("D ({})", label))
                                            .color(colors.d),
                                        times.iter().copied().zip(
                                            self.fd.d()[i]
                                                .map(|s| s.iter().copied())
                                                .unwrap_or_default(),
                                        ),
                                    )
                                    .line(
                                        TimeseriesLine::new(format!("F ({})", label))
                                            .color(colors.f),
                                        times.iter().copied().zip(
                                            self.fd
                                                .f()
                                                .map(|s| s[i].iter().copied())
                                                .unwrap_or_default(),
                                        ),
                                    ),
                            );
                        }
                    })
                    .response
                })
                .column(|ui| {
                    ui.vertical(|ui| {
                        ui.heading("Step Response");

                        let responses = [&roll_response, &pitch_response, &yaw_response];
                        for (i, axis) in responses.iter().enumerate() {
                            let axis_color = egui::Color32::from_rgb(
                                AXIS_COLORS[i][0],
                                AXIS_COLORS[i][1],
                                AXIS_COLORS[i][2],
                            );
                            Self::plot_step_response(
                                ui,
                                i,
                                axis,
                                total_width,
                                self.y_correction,
                                self.y_scale,
                                self.show_ideal_response,
                                self.ideal_time_constant_ms,
                                self.ideal_filter_type,
                                Some(axis_color),
                            );
                        }

                        // Show metrics panel if enabled
                        if self.show_metrics {
                            ui.add_space(8.0);
                            ui.separator();
                            ui.heading("# Step Response Metrics");
                            ui.add_space(4.0);
                            Self::show_metrics_panel(ui, step_responses);
                            
                            // Add tuning hints based on metrics
                            ui.add_space(8.0);
                            Self::show_tuning_hints(ui, step_responses);
                        }
                    })
                    .response
                })
                .show(ui);
        }
    }

    /// Show tuning recommendations based on step response metrics
    fn show_tuning_hints(ui: &mut egui::Ui, responses: &StepResponses) {
        let metrics = [
            &responses.roll_metrics,
            &responses.pitch_metrics,
            &responses.yaw_metrics,
        ];

        let mut hints: Vec<(&str, egui::Color32)> = Vec::new();

        // Check for high overshoot (suggests too much P or D)
        let max_overshoot = metrics.iter().map(|m| m.overshoot_pct).fold(0.0f32, f32::max);
        if max_overshoot > 25.0 {
            hints.push((
                "⚠ High overshoot detected. Consider reducing P gain or increasing D gain.",
                egui::Color32::from_rgb(0xfa, 0xbd, 0x2f),
            ));
        }

        // Check for oscillations
        let has_oscillations = metrics.iter().any(|m| m.has_oscillations);
        if has_oscillations {
            hints.push((
                "⚠ Oscillations detected. This usually indicates P gain is too high or filtering is too aggressive.",
                egui::Color32::from_rgb(0xfb, 0x49, 0x34),
            ));
        }

        // Check for slow response (high rise time)
        let max_rise_time = metrics.iter().map(|m| m.rise_time_ms).fold(0.0f32, f32::max);
        if max_rise_time > 100.0 {
            hints.push((
                "ℹ Slow response detected. Consider increasing P gain or feedforward.",
                egui::Color32::from_rgb(0x83, 0xa5, 0x98),
            ));
        }

        // Check for high steady-state error
        let max_ss_error = metrics
            .iter()
            .map(|m| m.steady_state_error)
            .fold(0.0f32, f32::max);
        if max_ss_error > 0.1 {
            hints.push((
                "ℹ High steady-state error. Consider increasing I gain.",
                egui::Color32::from_rgb(0x83, 0xa5, 0x98),
            ));
        }

        // Good tune indicator
        if max_overshoot < 15.0
            && !has_oscillations
            && max_rise_time < 50.0
            && max_ss_error < 0.05
        {
            hints.push((
                "✓ Tune looks good! Low overshoot, fast response, no oscillations.",
                egui::Color32::from_rgb(0x8e, 0xc0, 0x7c),
            ));
        }

        if !hints.is_empty() {
            ui.group(|ui| {
                ui.label(egui::RichText::new("* Tuning Hints").strong());
                for (hint, color) in hints {
                    ui.colored_label(color, hint);
                }
            });
        }
    }
}

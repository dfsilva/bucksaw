use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;

use egui_oszi::{TimeseriesGroup, TimeseriesLine, TimeseriesPlot, TimeseriesPlotMemory};
use egui_plot::{Corner, Legend, PlotPoints};

use crate::flight_data::FlightData;
use crate::gui::colors::Colors;
use crate::gui::flex::FlexColumns;
use crate::step_response::{calculate_step_response, SmoothingLevel};
use crate::utils::execute_in_background;

use super::{MIN_WIDE_WIDTH, PLOT_HEIGHT};

struct StepResponses {
    roll_step_response: Vec<(f64, f64)>,
    pitch_step_response: Vec<(f64, f64)>,
    yaw_step_response: Vec<(f64, f64)>,
}

pub struct TuneTab {
    roll_plot: TimeseriesPlotMemory<f64, f32>,
    pitch_plot: TimeseriesPlotMemory<f64, f32>,
    yaw_plot: TimeseriesPlotMemory<f64, f32>,
    fd: Arc<FlightData>,
    step_responses: Option<StepResponses>,
    receiver: Option<Receiver<StepResponses>>,
    smoothing: SmoothingLevel,
    y_correction: f64,
    y_scale: f64,
}

const AXIS_LABELS: [&str; 3] = ["Roll", "Pitch", "Yaw"];

impl TuneTab {
    pub fn new(fd: Arc<FlightData>) -> Self {
        // calculate step response in background thread
        let (sender, receiver) = channel();

        Self::calculate_responses(fd.clone(), sender, SmoothingLevel::default());
        Self {
            roll_plot: TimeseriesPlotMemory::new("roll"),
            pitch_plot: TimeseriesPlotMemory::new("pitch"),
            yaw_plot: TimeseriesPlotMemory::new("yaw"),
            step_responses: None,
            receiver: Some(receiver),
            fd,
            smoothing: SmoothingLevel::default(),
            y_correction: 0.0,
            y_scale: 1.5,
        }
    }

    fn calculate_responses(
        fd: Arc<FlightData>,
        sender: Sender<StepResponses>,
        smoothing: SmoothingLevel,
    ) {
        execute_in_background(async move {
            let empty_fallback = Vec::new();
            let setpoints = fd.setpoint().unwrap_or([&empty_fallback; 4]);
            let gyro = fd.gyro_filtered().unwrap_or([&empty_fallback; 3]);
            let sample_rate = fd.sample_rate();
            let roll_step_response =
                calculate_step_response(&fd.times, setpoints[0], gyro[0], sample_rate, smoothing);
            let pitch_step_response =
                calculate_step_response(&fd.times, setpoints[1], gyro[1], sample_rate, smoothing);
            let yaw_step_response =
                calculate_step_response(&fd.times, setpoints[2], gyro[2], sample_rate, smoothing);
            let _ = sender.send(StepResponses {
                roll_step_response,
                pitch_step_response,
                yaw_step_response,
            });
        });
    }

    fn recalculate(&mut self) {
        let (sender, receiver) = channel();
        self.receiver = Some(receiver);
        Self::calculate_responses(self.fd.clone(), sender, self.smoothing);
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
    ) -> egui::Response {
        let height = if ui.available_width() < total_width {
            ui.available_height() / (3 - i) as f32
        } else {
            PLOT_HEIGHT
        };

        egui_plot::Plot::new(ui.next_auto_id())
            .legend(Legend::default().position(Corner::RightBottom))
            .set_margin_fraction(egui::Vec2::new(0.0, 0.1))
            .show_grid(true)
            .allow_drag(false)
            .allow_zoom(false)
            .allow_scroll(false)
            .link_axis("step_response", true, true)
            .link_cursor("step_response", true, true)
            .y_axis_position(egui_plot::HPlacement::Right)
            .y_axis_width(3)
            .height(height)
            .include_y(-y_scale + 1.0)
            .include_y(y_scale + 1.0)
            .show(ui, |plot_ui| {
                // Apply Y correction offset to each point
                let points = PlotPoints::new(
                    step_response
                        .iter()
                        .map(|(x, y)| [*x, *y + y_correction])
                        .collect(),
                );
                let egui_line = egui_plot::Line::new(points)
                    .name(format!("Step Response ({})", AXIS_LABELS[i]))
                    .color(egui::Color32::from_rgb(0xaf, 0x3a, 0x03))
                    .width(2.0);
                plot_ui.line(egui_line);
            })
            .response
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
            }

            ui.separator();

            ui.label("Y Correction:");
            ui.add(egui::Slider::new(&mut self.y_correction, -0.5..=0.5).step_by(0.01));

            ui.separator();

            ui.label("Y Scale:");
            ui.add(egui::Slider::new(&mut self.y_scale, 0.3..=2.0).step_by(0.1));
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
                            Self::plot_step_response(
                                ui,
                                i,
                                axis,
                                total_width,
                                self.y_correction,
                                self.y_scale,
                            );
                        }
                    })
                    .response
                })
                .show(ui);
        }
    }
}

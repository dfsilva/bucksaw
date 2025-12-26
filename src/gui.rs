pub mod blackbox_ui_ext;
pub mod colors;
pub mod flex;
pub mod flight_view;
pub mod loading_modal;
pub mod open_file;
pub mod opened_file;
pub mod tabs;

use std::path::PathBuf;
use std::sync::Arc;

use egui::Layout;
use itertools::Itertools;

use crate::analytics;
use crate::gui::blackbox_ui_ext::*;
use crate::gui::colors::Colors;
use crate::gui::flight_view::*;
use crate::gui::open_file::*;
use crate::gui::opened_file::*;
use crate::gui::tabs::*;
use crate::log_file::*;

/// Selection state tracking both file and flight indices
#[derive(Default, Clone, Copy, PartialEq)]
pub struct Selection {
    pub file_index: usize,
    pub flight_index: usize,
}

pub struct App {
    open_file_dialog: Option<OpenFileDialog>,
    flight_view_tab: FlightViewTab,
    opened_files: Vec<OpenedFile>,
    selected: Selection,
    left_panel_open: bool,
}

impl App {
    pub fn new(cc: &eframe::CreationContext, path: Option<PathBuf>) -> Self {
        egui_extras::install_image_loaders(&cc.egui_ctx);

        let open_file_dialog = Some(OpenFileDialog::new(path));
        Self {
            open_file_dialog,
            flight_view_tab: FlightViewTab::Plot,
            opened_files: Vec::new(),
            selected: Selection::default(),
            left_panel_open: true,
        }
    }

    fn open_log(&mut self, ctx: &egui::Context, file_data: LogFile) {
        let flights: FlightDataAndView = file_data
            .flights
            .into_iter()
            .map(|f| match f {
                Ok(f) => {
                    let f = Arc::new(f);
                    (Ok(f.clone()), Some(FlightView::new(ctx, f)))
                }
                Err(e) => (Err(e), None),
            })
            .collect_vec();

        let file_name = file_data.file_name;
        let flight_count = flights.len();

        // Track file opened in analytics
        analytics::log_file_opened(&file_name, flight_count);

        let opened_file = OpenedFile::new(file_name, flights);
        self.opened_files.push(opened_file);

        // Select the last flight in the newly opened file
        self.selected = Selection {
            file_index: self.opened_files.len() - 1,
            flight_index: flight_count.saturating_sub(1),
        };

        self.open_file_dialog = None;

        // Keep panel open when we have files
        if !self.opened_files.is_empty() {
            self.left_panel_open = true;
        }
    }

    fn close_file(&mut self, file_index: usize) {
        if file_index < self.opened_files.len() {
            self.opened_files.remove(file_index);
            analytics::log_file_closed();

            // Adjust selection if needed
            if self.opened_files.is_empty() {
                self.selected = Selection::default();
            } else if self.selected.file_index >= self.opened_files.len() {
                // Select last file's first flight
                self.selected = Selection {
                    file_index: self.opened_files.len() - 1,
                    flight_index: 0,
                };
            } else if self.selected.file_index == file_index {
                // Closed the selected file, select first flight of same index or previous
                let new_file_index = file_index.min(self.opened_files.len() - 1);
                self.selected = Selection {
                    file_index: new_file_index,
                    flight_index: 0,
                };
            }
        }
    }

    fn get_selected_view_mut(&mut self) -> Option<&mut FlightView> {
        self.opened_files
            .get_mut(self.selected.file_index)
            .and_then(|file| file.flights.get_mut(self.selected.flight_index))
            .and_then(|(_, view)| view.as_mut())
    }
}

impl eframe::App for App {
    /// Main draw method of the application
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();
        #[cfg(feature = "profiling")]
        puffin::GlobalProfiler::lock().new_frame();
        #[cfg(feature = "profiling")]
        puffin_egui::profiler_window(ctx);

        if let Some(open_file_dialog) = self.open_file_dialog.as_mut() {
            match open_file_dialog.show(ctx) {
                Some(Some(result)) => {
                    self.open_log(ctx, result);
                }
                Some(None) => {
                    self.open_file_dialog = None;
                }
                None => {} // Not done yet.
            }
        }

        let enabled = self.open_file_dialog.is_none();

        let width = ctx.available_rect().width();
        let narrow = width < 400.0;

        egui::TopBottomPanel::top("menubar")
            .min_height(30.0)
            .max_height(30.0)
            .show(ctx, |ui| {
                ui.set_enabled(enabled);
                ui.horizontal_centered(|ui| {
                    if ui
                        .button(if self.left_panel_open { "⏴" } else { "☰" })
                        .clicked()
                    {
                        self.left_panel_open = !self.left_panel_open;
                        analytics::log_panel_toggled("left", self.left_panel_open);
                    }

                    // TODO: right panel (ℹ)

                    if ui
                        .button(if narrow { "🗁 " } else { "🗁  Open File" })
                        .clicked()
                    {
                        self.open_file_dialog = Some(OpenFileDialog::new(None));
                        ctx.request_repaint();
                    }

                    ui.separator();

                    const TABS: [FlightViewTab; 11] = [
                        FlightViewTab::Dashboard,
                        FlightViewTab::Plot,
                        FlightViewTab::Tune,
                        FlightViewTab::Vibe,
                        FlightViewTab::Stats,
                        FlightViewTab::Anomalies,
                        FlightViewTab::Error,
                        FlightViewTab::Setup,
                        FlightViewTab::Suggestions,
                        FlightViewTab::Filter,
                        FlightViewTab::Logs,
                    ];
                    for tab in TABS.into_iter() {
                        let label = if narrow {
                            tab.to_string().split(' ').next().unwrap().to_string()
                        } else {
                            tab.to_string()
                        };
                        let previous_tab = self.flight_view_tab;
                        ui.selectable_value(&mut self.flight_view_tab, tab, label);
                        // Track tab selection in analytics
                        if previous_tab != self.flight_view_tab && self.flight_view_tab == tab {
                            analytics::log_tab_selected(&tab.analytics_name());
                        }
                    }

                    ui.separator();

                    ui.with_layout(Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.hyperlink_to("", env!("CARGO_PKG_REPOSITORY"));
                        ui.separator();
                        egui::widgets::global_dark_light_mode_switch(ui);
                        ui.separator();
                    });
                });
            });

        // Track actions to perform after iterating (to avoid borrow issues)
        let mut file_to_close: Option<usize> = None;
        let mut new_selection: Option<Selection> = None;
        let mut toggle_expand: Option<usize> = None;

        if self.left_panel_open {
            let panel_draw = |ui: &mut egui::Ui| {
                ui.set_enabled(enabled);
                ui.set_width(ui.available_width());
                egui::ScrollArea::vertical().show(ui, |ui| {
                    ui.set_width(ui.available_width());

                    let colors = Colors::get(ui);

                    for (file_idx, opened_file) in self.opened_files.iter().enumerate() {
                        // File header
                        egui::Frame::none()
                            .fill(ui.visuals().faint_bg_color)
                            .rounding(4.0)
                            .inner_margin(4.0)
                            .show(ui, |ui| {
                                ui.horizontal(|ui| {
                                    // Expand/collapse button
                                    let expand_icon =
                                        if opened_file.expanded { "▼" } else { "▶" };
                                    if ui.small_button(expand_icon).clicked() {
                                        toggle_expand = Some(file_idx);
                                    }

                                    // File icon and name
                                    ui.label("📁");
                                    ui.label(&opened_file.file_name);

                                    // Close button on the right
                                    ui.with_layout(
                                        Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            if ui.small_button("✕").clicked() {
                                                file_to_close = Some(file_idx);
                                            }
                                        },
                                    );
                                });
                            });

                        // Flight list (if expanded)
                        if opened_file.expanded {
                            ui.indent(format!("file_{}_flights", file_idx), |ui| {
                                for (flight_idx, (parse_result, _)) in
                                    opened_file.flights.iter().enumerate()
                                {
                                    let is_selected = self.selected.file_index == file_idx
                                        && self.selected.flight_index == flight_idx;

                                    let bg_color = if is_selected {
                                        Some(ui.visuals().selection.bg_fill.gamma_multiply(0.5))
                                    } else if parse_result.is_err() {
                                        Some(colors.error.gamma_multiply(0.3))
                                    } else {
                                        None
                                    };

                                    egui::Frame::none()
                                        .fill(bg_color.unwrap_or(egui::Color32::TRANSPARENT))
                                        .rounding(2.0)
                                        .inner_margin(2.0)
                                        .show(ui, |ui| {
                                            ui.horizontal(|ui| {
                                                // Flight indicator
                                                if parse_result.is_ok() {
                                                    ui.label("  ");
                                                } else {
                                                    ui.label("⚠ ");
                                                }

                                                ui.label("Flight ");
                                                ui.monospace(format!("#{}", flight_idx + 1));

                                                if parse_result.is_ok() {
                                                    ui.with_layout(
                                                        Layout::right_to_left(egui::Align::Center),
                                                        |ui| {
                                                            if ui.small_button("➡").clicked() {
                                                                new_selection = Some(Selection {
                                                                    file_index: file_idx,
                                                                    flight_index: flight_idx,
                                                                });
                                                                analytics::log_flight_selected(
                                                                    flight_idx, file_idx,
                                                                );
                                                            }
                                                        },
                                                    );
                                                }
                                            });

                                            // Show flight metadata
                                            match parse_result {
                                                Ok(flight) => {
                                                    flight.show(ui);
                                                }
                                                Err(error) => {
                                                    error.show(ui);
                                                }
                                            }
                                        });
                                }
                            });
                        }

                        ui.add_space(8.0);
                    }
                });
            };
            if narrow {
                egui::CentralPanel::default().show(ctx, panel_draw);
            } else {
                egui::SidePanel::left("browserpanel")
                    .resizable(true)
                    .min_width(100.0)
                    .max_width(400.0)
                    .show(ctx, panel_draw);
            }
        }

        // Apply deferred actions
        if let Some(file_idx) = file_to_close {
            self.close_file(file_idx);
        }
        if let Some(selection) = new_selection {
            self.selected = selection;
        }
        if let Some(file_idx) = toggle_expand {
            if let Some(file) = self.opened_files.get_mut(file_idx) {
                file.expanded = !file.expanded;
            }
        }

        if !(self.left_panel_open && narrow) {
            // Track navigation action to handle after the borrow ends
            let mut nav_result: Option<NavigationAction> = None;
            let current_tab = self.flight_view_tab;

            egui::CentralPanel::default().show(ctx, |ui| {
                ui.set_enabled(enabled);

                if let Some(view) = self.get_selected_view_mut() {
                    // Check for navigation actions (e.g., from Anomalies tab "Jump" buttons)
                    if let Some(nav_action) = view.show(ui, current_tab) {
                        // Set the time range to focus on the anomaly
                        view.set_time_range(
                            nav_action.target_time_start,
                            nav_action.target_time_end,
                        );
                        log::info!(
                            "Navigating to time range: {:.1}s - {:.1}s",
                            nav_action.target_time_start,
                            nav_action.target_time_end
                        );
                        nav_result = Some(nav_action);
                    }
                }
            });

            // Handle navigation after the borrow ends
            if nav_result.is_some() {
                self.flight_view_tab = FlightViewTab::Plot;
            }
        }
    }
}

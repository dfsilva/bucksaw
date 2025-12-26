pub mod blackbox_ui_ext;
pub mod colors;
pub mod flex;
pub mod flight_view;
pub mod loading_modal;
pub mod open_file;
pub mod opened_file;
pub mod tabs;
pub mod toast;
pub mod welcome;

use std::path::PathBuf;
use std::sync::Arc;

use egui::{Layout, Key, RichText};
use egui_phosphor::regular as icons;
use itertools::Itertools;

use crate::analytics;
use crate::settings::AppSettings;
use crate::gui::blackbox_ui_ext::*;
use crate::gui::colors::Colors;
use crate::gui::flight_view::*;
use crate::gui::open_file::*;
use crate::gui::opened_file::*;
use crate::gui::tabs::*;
use crate::gui::toast::Toaster;
use crate::gui::welcome::{WelcomePanel, WelcomeAction};
use crate::log_file::*;

/// Selection state tracking both file and flight indices
#[derive(Default, Clone, Copy, PartialEq)]
pub struct Selection {
    pub file_index: usize,
    pub flight_index: usize,
}

/// Tab groups for the menu to prevent overflow
#[derive(Clone, Copy, PartialEq, Debug)]
enum TabGroup {
    Main,      // Dashboard, Plot, Tune
    Analysis,  // Vibe, Stats, Anomalies, Error
    Config,    // Setup, Filter, Suggestions
    More,      // Logs, Guide, Feedforward
}

impl TabGroup {
    fn label(&self) -> &'static str {
        match self {
            Self::Main => "View",
            Self::Analysis => "Analysis",
            Self::Config => "Config",
            Self::More => "More",
        }
    }

    fn tabs(&self) -> &[FlightViewTab] {
        match self {
            Self::Main => &[FlightViewTab::Dashboard, FlightViewTab::Plot, FlightViewTab::Tune],
            Self::Analysis => &[FlightViewTab::Vibe, FlightViewTab::Stats, FlightViewTab::Anomalies, FlightViewTab::Error],
            Self::Config => &[FlightViewTab::Setup, FlightViewTab::Filter, FlightViewTab::Suggestions],
            Self::More => &[FlightViewTab::Logs, FlightViewTab::TuningGuide, FlightViewTab::Feedforward],
        }
    }

    fn contains(&self, tab: FlightViewTab) -> bool {
        self.tabs().contains(&tab)
    }
}

const TAB_GROUPS: [TabGroup; 4] = [TabGroup::Main, TabGroup::Analysis, TabGroup::Config, TabGroup::More];

pub struct App {
    open_file_dialog: Option<OpenFileDialog>,
    flight_view_tab: FlightViewTab,
    opened_files: Vec<OpenedFile>,
    selected: Selection,
    left_panel_open: bool,
    settings: AppSettings,
    toaster: Toaster,
    /// Pending path to open from recent files
    pending_open_path: Option<PathBuf>,
}

impl App {
    pub fn new(cc: &eframe::CreationContext, path: Option<PathBuf>) -> Self {
        egui_extras::install_image_loaders(&cc.egui_ctx);

        // Install Phosphor icons font with high priority for icon glyphs
        let mut fonts = egui::FontDefinitions::default();

        // Add Phosphor font data
        fonts.font_data.insert(
            "phosphor".into(),
            egui_phosphor::Variant::Regular.font_data(),
        );

        // Insert Phosphor font at the beginning of the Proportional family
        // so it takes precedence for icon glyphs (PUA characters)
        if let Some(font_keys) = fonts.families.get_mut(&egui::FontFamily::Proportional) {
            font_keys.insert(0, "phosphor".into());
        }

        cc.egui_ctx.set_fonts(fonts);

        // Load settings
        let settings = AppSettings::load();

        // Apply dark mode preference
        if let Some(dark_mode) = settings.ui.dark_mode {
            cc.egui_ctx.set_visuals(if dark_mode {
                egui::Visuals::dark()
            } else {
                egui::Visuals::light()
            });
        }

        // Log app start
        analytics::log_app_started();

        let open_file_dialog = path.as_ref().map(|p| OpenFileDialog::new(Some(p.clone())));
        let left_panel_open = settings.ui.left_panel_open.unwrap_or(true);
        
        Self {
            open_file_dialog,
            flight_view_tab: FlightViewTab::Dashboard,
            opened_files: Vec::new(),
            selected: Selection::default(),
            left_panel_open,
            settings,
            toaster: Toaster::new(),
            pending_open_path: None,
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

        let file_name = file_data.file_name.clone();
        let flight_count = flights.len();

        // Track file opened in analytics
        analytics::log_file_opened(&file_name, flight_count);

        // Add to recent files
        if let Some(ref path) = file_data.file_path {
            self.settings.add_recent_file(&path.to_string_lossy());
        }

        let opened_file = OpenedFile::new(file_name.clone(), flights);
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

        // Show toast notification
        self.toaster.success(format!("Loaded {} with {} flight(s)", file_name, flight_count));
    }

    fn close_file(&mut self, file_index: usize) {
        if file_index < self.opened_files.len() {
            let file_name = self.opened_files[file_index].file_name.clone();
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

            // Show toast
            self.toaster.info(format!("Closed {}", file_name));
        }
    }

    /// Handle keyboard shortcuts
    fn handle_keyboard_shortcuts(&mut self, ctx: &egui::Context) -> bool {
        let mut handled = false;

        ctx.input(|i| {
            // Cmd/Ctrl + O: Open file
            if i.modifiers.command && i.key_pressed(Key::O) {
                self.open_file_dialog = Some(OpenFileDialog::new(None));
                handled = true;
            }

            // Escape: Close dialog or deselect
            if i.key_pressed(Key::Escape) {
                if self.open_file_dialog.is_some() {
                    self.open_file_dialog = None;
                    handled = true;
                }
            }

            // Number keys 1-9: Switch tabs (when not in a text field)
            if !i.modifiers.command && !i.modifiers.alt {
                let all_tabs = [
                    FlightViewTab::Dashboard,
                    FlightViewTab::Plot,
                    FlightViewTab::Tune,
                    FlightViewTab::Vibe,
                    FlightViewTab::Stats,
                    FlightViewTab::Anomalies,
                    FlightViewTab::Error,
                    FlightViewTab::Setup,
                    FlightViewTab::Suggestions,
                ];

                for (idx, key) in [Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6, Key::Num7, Key::Num8, Key::Num9].iter().enumerate() {
                    if i.key_pressed(*key) && idx < all_tabs.len() && !self.opened_files.is_empty() {
                        let new_tab = all_tabs[idx];
                        if self.flight_view_tab != new_tab {
                            self.flight_view_tab = new_tab;
                            analytics::log_tab_selected(&new_tab.analytics_name());
                            handled = true;
                        }
                        break;
                    }
                }
            }

            // Cmd/Ctrl + W: Close current file
            if i.modifiers.command && i.key_pressed(Key::W) && !self.opened_files.is_empty() {
                self.close_file(self.selected.file_index);
                handled = true;
            }

            // Cmd/Ctrl + [ or ]: Navigate between tabs
            if i.modifiers.command {
                let all_tabs = [
                    FlightViewTab::Dashboard,
                    FlightViewTab::Plot,
                    FlightViewTab::Tune,
                    FlightViewTab::Vibe,
                    FlightViewTab::Stats,
                    FlightViewTab::Anomalies,
                    FlightViewTab::Error,
                    FlightViewTab::Setup,
                    FlightViewTab::Suggestions,
                ];
                
                if let Some(current_idx) = all_tabs.iter().position(|t| *t == self.flight_view_tab) {
                    if i.key_pressed(Key::OpenBracket) && current_idx > 0 {
                        self.flight_view_tab = all_tabs[current_idx - 1];
                        handled = true;
                    }
                    if i.key_pressed(Key::CloseBracket) && current_idx < all_tabs.len() - 1 {
                        self.flight_view_tab = all_tabs[current_idx + 1];
                        handled = true;
                    }
                }
            }
        });

        handled
    }

    /// Save dark mode preference
    fn save_dark_mode_preference(&mut self, ctx: &egui::Context) {
        let is_dark = ctx.style().visuals.dark_mode;
        if self.settings.ui.dark_mode != Some(is_dark) {
            self.settings.ui.dark_mode = Some(is_dark);
            self.settings.save();
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

        // Handle pending path from recent files
        if let Some(path) = self.pending_open_path.take() {
            self.open_file_dialog = Some(OpenFileDialog::new(Some(path)));
        }

        // Handle keyboard shortcuts
        self.handle_keyboard_shortcuts(ctx);

        // Save dark mode preference when changed
        self.save_dark_mode_preference(ctx);

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
        let medium = width < 900.0;

        egui::TopBottomPanel::top("menubar")
            .min_height(30.0)
            .max_height(30.0)
            .show(ctx, |ui| {
                ui.set_enabled(enabled);
                ui.horizontal_centered(|ui| {
                    // Sidebar toggle with tooltip
                    let sidebar_btn = ui.button(if self.left_panel_open {
                        icons::CARET_LEFT
                    } else {
                        icons::LIST
                    });
                    if sidebar_btn.on_hover_text("Toggle sidebar (files & flights)").clicked() {
                        self.left_panel_open = !self.left_panel_open;
                        self.settings.ui.left_panel_open = Some(self.left_panel_open);
                        self.settings.save();
                        analytics::log_panel_toggled("left", self.left_panel_open);
                    }

                    // Open file button with tooltip
                    let open_label = if narrow {
                        icons::FOLDER_OPEN.to_string()
                    } else {
                        format!("{} Open", icons::FOLDER_OPEN)
                    };
                    let open_btn = ui.button(&open_label);
                    let shortcut = if cfg!(target_os = "macos") { "⌘O" } else { "Ctrl+O" };
                    if open_btn.on_hover_text(format!("Open blackbox log file ({})", shortcut)).clicked() {
                        self.open_file_dialog = Some(OpenFileDialog::new(None));
                        ctx.request_repaint();
                    }

                    ui.separator();

                    // Tabs - use grouped dropdowns on narrow screens, flat tabs on wide screens
                    if medium || narrow {
                        // Grouped dropdown menus for narrow/medium screens
                        for group in TAB_GROUPS {
                            let is_current_group = group.contains(self.flight_view_tab);
                            let label = if is_current_group {
                                format!("{} ▼", self.flight_view_tab.short_label())
                            } else {
                                format!("{} ▼", group.label())
                            };

                            let response = ui.selectable_label(is_current_group, &label);
                            
                            // Show dropdown menu on click
                            let popup_id = ui.make_persistent_id(format!("tab_group_{:?}", group));
                            if response.clicked() {
                                ui.memory_mut(|mem| mem.toggle_popup(popup_id));
                            }

                            egui::popup::popup_below_widget(ui, popup_id, &response, |ui| {
                                ui.set_min_width(150.0);
                                for (idx, tab) in group.tabs().iter().enumerate() {
                                    let is_selected = self.flight_view_tab == *tab;
                                    let shortcut_num = match group {
                                        TabGroup::Main => idx + 1,
                                        TabGroup::Analysis => idx + 4,
                                        TabGroup::Config => idx + 8,
                                        _ => 0,
                                    };
                                    let shortcut_text = if shortcut_num > 0 && shortcut_num <= 9 {
                                        format!("  {}", shortcut_num)
                                    } else {
                                        String::new()
                                    };

                                    let label_text = format!("{}{}", tab.label(), shortcut_text);
                                    if ui.selectable_label(is_selected, label_text).clicked() {
                                        let previous_tab = self.flight_view_tab;
                                        self.flight_view_tab = *tab;
                                        if previous_tab != self.flight_view_tab {
                                            analytics::log_tab_selected(&tab.analytics_name());
                                        }
                                        ui.memory_mut(|mem| mem.close_popup());
                                    }
                                }
                            });
                        }
                    } else {
                        // Flat tabs for wide screens - primary tabs only
                        const PRIMARY_TABS: [FlightViewTab; 9] = [
                            FlightViewTab::Dashboard,
                            FlightViewTab::Plot,
                            FlightViewTab::Tune,
                            FlightViewTab::Vibe,
                            FlightViewTab::Stats,
                            FlightViewTab::Anomalies,
                            FlightViewTab::Setup,
                            FlightViewTab::Suggestions,
                            FlightViewTab::Filter,
                        ];

                        for (idx, tab) in PRIMARY_TABS.iter().enumerate() {
                            let label = tab.short_label();
                            let previous_tab = self.flight_view_tab;
                            let response = ui.selectable_value(&mut self.flight_view_tab, *tab, label);
                            if response.on_hover_text(format!("{} ({})", tab.label(), idx + 1)).clicked() {
                                if previous_tab != self.flight_view_tab {
                                    analytics::log_tab_selected(&tab.analytics_name());
                                }
                            }
                        }

                        // "More" dropdown for less-used tabs
                        let more_tabs = [FlightViewTab::Error, FlightViewTab::Logs, FlightViewTab::TuningGuide, FlightViewTab::Feedforward];
                        let is_more_selected = more_tabs.contains(&self.flight_view_tab);
                        let more_label = if is_more_selected {
                            format!("{} {}", icons::DOTS_THREE, self.flight_view_tab.short_label())
                        } else {
                            format!("{} More", icons::DOTS_THREE)
                        };

                        let more_response = ui.selectable_label(is_more_selected, &more_label);
                        let more_popup_id = ui.make_persistent_id("more_tabs_popup");
                        if more_response.clicked() {
                            ui.memory_mut(|mem| mem.toggle_popup(more_popup_id));
                        }

                        egui::popup::popup_below_widget(ui, more_popup_id, &more_response, |ui| {
                            ui.set_min_width(150.0);
                            for tab in more_tabs {
                                if ui.selectable_label(self.flight_view_tab == tab, tab.label()).clicked() {
                                    let previous_tab = self.flight_view_tab;
                                    self.flight_view_tab = tab;
                                    if previous_tab != self.flight_view_tab {
                                        analytics::log_tab_selected(&tab.analytics_name());
                                    }
                                    ui.memory_mut(|mem| mem.close_popup());
                                }
                            }
                        });
                    }

                    ui.separator();

                    ui.with_layout(Layout::right_to_left(egui::Align::Center), |ui| {
                        // GitHub link with tooltip
                        ui.hyperlink_to(icons::GITHUB_LOGO, env!("CARGO_PKG_REPOSITORY"))
                            .on_hover_text("View source on GitHub");

                        ui.separator();

                        // Dark mode switch with tooltip
                        let dark_mode_btn = egui::widgets::global_dark_light_mode_switch(ui);
                        
                        ui.separator();

                        // Help/keyboard shortcuts hint
                        let help_btn = ui.small_button(icons::KEYBOARD)
                            .on_hover_text("Keyboard shortcuts (hover for list)");
                        
                        if help_btn.clicked() {
                            // Could show a help popup here in the future
                        }

                        if help_btn.hovered() {
                            egui::show_tooltip(ui.ctx(), egui::Id::new("shortcuts_tooltip"), |ui| {
                                ui.label(RichText::new("Keyboard Shortcuts").strong());
                                ui.separator();
                                egui::Grid::new("shortcuts_grid").show(ui, |ui| {
                                    let cmd = if cfg!(target_os = "macos") { "⌘" } else { "Ctrl+" };
                                    ui.label(format!("{}O", cmd)); ui.label("Open file"); ui.end_row();
                                    ui.label(format!("{}W", cmd)); ui.label("Close file"); ui.end_row();
                                    ui.label("1-9"); ui.label("Switch tabs"); ui.end_row();
                                    ui.label(format!("{}[ / {}]", cmd, cmd)); ui.label("Previous/Next tab"); ui.end_row();
                                    ui.label("Esc"); ui.label("Close dialog"); ui.end_row();
                                });
                            });
                        }
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

                    if self.opened_files.is_empty() {
                        // Empty state hint in sidebar
                        ui.vertical_centered(|ui| {
                            ui.add_space(20.0);
                            ui.label(RichText::new(icons::FOLDER_OPEN).size(24.0).weak());
                            ui.label(RichText::new("No files open").weak());
                            ui.add_space(8.0);
                            if ui.button("Open File").clicked() {
                                // Use same action as menu
                            }
                        });
                    }

                    for (file_idx, opened_file) in self.opened_files.iter().enumerate() {
                        // File header with context menu
                        let file_response = egui::Frame::none()
                            .fill(ui.visuals().faint_bg_color)
                            .rounding(4.0)
                            .inner_margin(4.0)
                            .show(ui, |ui| {
                                ui.horizontal(|ui| {
                                    // Expand/collapse button
                                    let expand_icon = if opened_file.expanded {
                                        icons::CARET_DOWN
                                    } else {
                                        icons::CARET_RIGHT
                                    };
                                    if ui.small_button(expand_icon).on_hover_text("Expand/collapse flights").clicked() {
                                        toggle_expand = Some(file_idx);
                                    }

                                    // File icon and name
                                    ui.label(icons::FILE);
                                    ui.label(&opened_file.file_name);

                                    // Close button on the right
                                    ui.with_layout(
                                        Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            if ui.small_button(icons::X).on_hover_text("Close file").clicked() {
                                                file_to_close = Some(file_idx);
                                            }
                                        },
                                    );
                                });
                            })
                            .response;

                        // Context menu for file
                        file_response.context_menu(|ui| {
                            ui.label(RichText::new(&opened_file.file_name).strong());
                            ui.separator();

                            if ui.button(format!("{} Close file", icons::X)).clicked() {
                                file_to_close = Some(file_idx);
                                ui.close_menu();
                            }

                            if ui.button(format!("{} Expand all flights", icons::CARET_DOWN)).clicked() {
                                toggle_expand = Some(file_idx);
                                ui.close_menu();
                            }

                            ui.separator();

                            ui.label(RichText::new(format!("{} flights", opened_file.flights.len())).weak());
                        });

                        // Flight list (if expanded)
                        if opened_file.expanded {
                            ui.indent(format!("file_{}_flights", file_idx), |ui| {
                                for (flight_idx, (parse_result, flight_view)) in
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

                                    let flight_response = egui::Frame::none()
                                        .fill(bg_color.unwrap_or(egui::Color32::TRANSPARENT))
                                        .rounding(2.0)
                                        .inner_margin(2.0)
                                        .show(ui, |ui| {
                                            ui.horizontal(|ui| {
                                                // Flight indicator
                                                if parse_result.is_ok() {
                                                    ui.label(icons::AIRPLANE_TILT);
                                                } else {
                                                    ui.label(icons::WARNING);
                                                }

                                                ui.label("Flight ");
                                                ui.monospace(format!("#{}", flight_idx + 1));

                                                if parse_result.is_ok() {
                                                    ui.with_layout(
                                                        Layout::right_to_left(egui::Align::Center),
                                                        |ui| {
                                                            if ui
                                                                .small_button(icons::ARROW_RIGHT)
                                                                .on_hover_text("View this flight")
                                                                .clicked()
                                                            {
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
                                        })
                                        .response;

                                    // Context menu for flight
                                    if parse_result.is_ok() {
                                        flight_response.context_menu(|ui| {
                                            ui.label(RichText::new(format!("Flight #{}", flight_idx + 1)).strong());
                                            ui.separator();

                                            if ui.button(format!("{} Select this flight", icons::ARROW_RIGHT)).clicked() {
                                                new_selection = Some(Selection {
                                                    file_index: file_idx,
                                                    flight_index: flight_idx,
                                                });
                                                ui.close_menu();
                                            }

                                            if let Ok(flight_data) = parse_result {
                                                ui.separator();
                                                ui.label(RichText::new(format!(
                                                    "Duration: {:.1}s",
                                                    flight_data.times.last().unwrap_or(&0.0) - flight_data.times.first().unwrap_or(&0.0)
                                                )).weak());
                                            }
                                        });

                                        // Double-click to select
                                        if flight_response.double_clicked() {
                                            new_selection = Some(Selection {
                                                file_index: file_idx,
                                                flight_index: flight_idx,
                                            });
                                        }
                                    }
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

                // Show welcome panel if no files are open
                if self.opened_files.is_empty() {
                    match WelcomePanel::show(ui, &self.settings.ui.recent_files) {
                        WelcomeAction::OpenFile => {
                            self.open_file_dialog = Some(OpenFileDialog::new(None));
                        }
                        WelcomeAction::OpenRecent(path) => {
                            self.pending_open_path = Some(PathBuf::from(path));
                        }
                        WelcomeAction::None => {}
                    }
                } else if let Some(view) = self.get_selected_view_mut() {
                    // Check for navigation actions (e.g., from Anomalies tab "Jump" buttons or Dashboard cards)
                    if let Some(nav_action) = view.show(ui, current_tab) {
                        // Set the time range to focus on the target (if specified)
                        if let (Some(start), Some(end)) = (nav_action.target_time_start, nav_action.target_time_end) {
                            view.set_time_range(start, end);
                            log::info!("Navigating to time range: {:.1}s - {:.1}s", start, end);
                        }
                        nav_result = Some(nav_action);
                    }
                }
            });

            // Handle navigation after the borrow ends
            if let Some(nav) = nav_result {
                // Navigate to specific tab if requested
                if let Some(target_tab) = nav.target_tab {
                    self.flight_view_tab = target_tab;
                    self.toaster.info(format!("Navigated to {} tab", target_tab.label()));
                } else if let (Some(start), Some(end)) = (nav.target_time_start, nav.target_time_end) {
                    // Time-based navigation without tab change goes to Plot
                    self.flight_view_tab = FlightViewTab::Plot;
                    self.toaster.info(format!(
                        "Jumped to {:.1}s - {:.1}s in Plot view",
                        start,
                        end
                    ));
                }
            }
        }

        // Show toast notifications (always on top)
        self.toaster.show(ctx);
    }
}

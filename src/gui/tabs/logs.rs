use egui::{Color32, RichText, Ui};

use crate::analytics;
use crate::app_logger::{logger, LogCategory, LogEntry, LogLevel};

/// Filter state for the logs tab
#[derive(Default)]
struct LogFilter {
    category: Option<LogCategory>,
    search_text: String,
    auto_scroll: bool,
}

pub struct LogsTab {
    filter: LogFilter,
}

impl LogsTab {
    pub fn new() -> Self {
        Self {
            filter: LogFilter {
                auto_scroll: true,
                ..Default::default()
            },
        }
    }

    fn level_color(level: LogLevel) -> Color32 {
        match level {
            LogLevel::Debug => Color32::GRAY,
            LogLevel::Info => Color32::LIGHT_BLUE,
            LogLevel::Warning => Color32::YELLOW,
            LogLevel::Error => Color32::from_rgb(255, 100, 100),
        }
    }

    fn category_color(category: LogCategory) -> Color32 {
        match category {
            LogCategory::General => Color32::from_rgb(150, 150, 150),
            LogCategory::AI => Color32::from_rgb(100, 200, 255),
            LogCategory::Analysis => Color32::from_rgb(100, 255, 150),
            LogCategory::File => Color32::from_rgb(255, 200, 100),
        }
    }

    fn filter_entries(&self, entries: &[LogEntry]) -> Vec<LogEntry> {
        entries
            .iter()
            .filter(|e| {
                // Category filter
                if let Some(cat) = self.filter.category {
                    if e.category != cat {
                        return false;
                    }
                }
                // Search filter
                if !self.filter.search_text.is_empty() {
                    let search_lower = self.filter.search_text.to_lowercase();
                    if !e.message.to_lowercase().contains(&search_lower) {
                        return false;
                    }
                }
                true
            })
            .cloned()
            .collect()
    }

    pub fn show(&mut self, ui: &mut Ui) {
        ui.heading("≡ Application Logs");
        ui.add_space(8.0);

        // Controls bar
        egui::Frame::none()
            .fill(ui.style().visuals.extreme_bg_color)
            .rounding(6.0)
            .inner_margin(8.0)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    // Category filter
                    ui.label("Category:");
                    egui::ComboBox::from_id_source("log_category_filter")
                        .selected_text(self.filter.category.map(|c| c.label()).unwrap_or("All"))
                        .show_ui(ui, |ui| {
                            if ui
                                .selectable_label(self.filter.category.is_none(), "All")
                                .clicked()
                            {
                                self.filter.category = None;
                            }
                            for cat in LogCategory::all() {
                                if ui
                                    .selectable_label(
                                        self.filter.category == Some(*cat),
                                        cat.label(),
                                    )
                                    .clicked()
                                {
                                    self.filter.category = Some(*cat);
                                }
                            }
                        });

                    ui.add_space(16.0);

                    // Search
                    ui.label("⌕");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.filter.search_text)
                            .hint_text("Search logs...")
                            .desired_width(200.0),
                    );

                    ui.add_space(16.0);

                    // Auto-scroll toggle
                    ui.checkbox(&mut self.filter.auto_scroll, "Auto-scroll");

                    ui.add_space(16.0);

                    // Clear button
                    if ui.button("✗ Clear").clicked() {
                        logger().clear();
                        analytics::log_logs_cleared();
                    }
                });
            });

        ui.add_space(8.0);

        // Get and filter entries
        let all_entries = logger().get_entries();
        let filtered_entries = self.filter_entries(&all_entries);

        // Stats
        ui.horizontal(|ui| {
            ui.label(
                RichText::new(format!(
                    "Showing {} of {} entries",
                    filtered_entries.len(),
                    all_entries.len()
                ))
                .weak(),
            );
        });

        ui.add_space(4.0);

        // Log entries
        let scroll = egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .stick_to_bottom(self.filter.auto_scroll);

        scroll.show(ui, |ui| {
            if filtered_entries.is_empty() {
                ui.label(RichText::new("No log entries to display.").weak().italics());
                return;
            }

            for entry in &filtered_entries {
                egui::Frame::none()
                    .fill(ui.style().visuals.faint_bg_color)
                    .rounding(4.0)
                    .inner_margin(6.0)
                    .show(ui, |ui| {
                        ui.horizontal_wrapped(|ui| {
                            // Timestamp
                            ui.label(
                                RichText::new(&entry.timestamp)
                                    .monospace()
                                    .color(Color32::GRAY)
                                    .small(),
                            );

                            // Level badge
                            let level_color = Self::level_color(entry.level);
                            ui.label(
                                RichText::new(format!("[{}]", entry.level.label()))
                                    .color(level_color)
                                    .strong()
                                    .small(),
                            );

                            // Category badge
                            let cat_color = Self::category_color(entry.category);
                            ui.label(
                                RichText::new(format!("({})", entry.category.label()))
                                    .color(cat_color)
                                    .small(),
                            );
                        });

                        ui.add_space(2.0);

                        // Message - check if it's a multi-line message (like AI prompts)
                        if entry.message.contains('\n') {
                            // Collapsing header for long messages
                            egui::CollapsingHeader::new(
                                RichText::new(entry.message.lines().next().unwrap_or("(empty)"))
                                    .monospace(),
                            )
                            .default_open(false)
                            .show(ui, |ui| {
                                egui::Frame::none()
                                    .fill(ui.style().visuals.extreme_bg_color)
                                    .rounding(4.0)
                                    .inner_margin(8.0)
                                    .show(ui, |ui| {
                                        ui.label(
                                            RichText::new(&entry.message).monospace().size(11.0),
                                        );
                                    });
                            });
                        } else {
                            ui.label(RichText::new(&entry.message).size(12.0));
                        }
                    });

                ui.add_space(2.0);
            }
        });
    }
}

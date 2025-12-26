//! Welcome/empty state panel shown when no files are loaded

use egui::{Color32, RichText, Vec2};
use egui_phosphor::regular as icons;

/// Welcome panel shown when no flight data is loaded
pub struct WelcomePanel;

impl WelcomePanel {
    /// Show the welcome panel with onboarding content
    /// Returns true if user clicked "Open File"
    pub fn show(ui: &mut egui::Ui, recent_files: &[String]) -> WelcomeAction {
        let mut action = WelcomeAction::None;

        let available_size = ui.available_size();
        let center_x = available_size.x / 2.0;

        ui.vertical_centered(|ui| {
            ui.add_space(available_size.y * 0.15);

            // App logo/title
            ui.label(
                RichText::new(icons::AIRPLANE_TILT)
                    .size(64.0)
                    .color(Color32::from_rgb(0xfa, 0xbd, 0x2f)),
            );

            ui.add_space(16.0);

            ui.label(
                RichText::new("PID Lab")
                    .size(32.0)
                    .strong(),
            );

            ui.label(
                RichText::new("Betaflight Blackbox Log Analyzer")
                    .size(16.0)
                    .weak(),
            );

            ui.add_space(32.0);

            // Drop zone
            let drop_zone_size = Vec2::new(400.0f32.min(available_size.x - 40.0), 120.0);
            
            let (rect, response) = ui.allocate_exact_size(drop_zone_size, egui::Sense::click());
            
            let is_hovered = response.hovered();
            let stroke_color = if is_hovered {
                Color32::from_rgb(0xfa, 0xbd, 0x2f)
            } else {
                Color32::from_gray(100)
            };

            ui.painter().rect_stroke(
                rect,
                12.0,
                egui::Stroke::new(2.0, stroke_color),
            );

            // Draw dashed pattern effect
            let center = rect.center();
            ui.painter().text(
                center - Vec2::new(0.0, 20.0),
                egui::Align2::CENTER_CENTER,
                icons::UPLOAD_SIMPLE,
                egui::FontId::proportional(32.0),
                stroke_color,
            );
            ui.painter().text(
                center + Vec2::new(0.0, 15.0),
                egui::Align2::CENTER_CENTER,
                "Drop .BBL or .BFL file here",
                egui::FontId::proportional(14.0),
                stroke_color,
            );
            ui.painter().text(
                center + Vec2::new(0.0, 35.0),
                egui::Align2::CENTER_CENTER,
                "or click to browse",
                egui::FontId::proportional(12.0),
                Color32::from_gray(120),
            );

            if response.clicked() {
                action = WelcomeAction::OpenFile;
            }

            ui.add_space(24.0);

            // Keyboard shortcut hint
            ui.label(
                RichText::new(format!("{} Press {} to open a file", icons::KEYBOARD, if cfg!(target_os = "macos") { "⌘O" } else { "Ctrl+O" }))
                    .size(12.0)
                    .weak(),
            );

            ui.add_space(32.0);

            // Recent files section
            if !recent_files.is_empty() {
                ui.label(
                    RichText::new(format!("{} Recent Files", icons::CLOCK_COUNTER_CLOCKWISE))
                        .size(14.0)
                        .strong(),
                );

                ui.add_space(8.0);

                egui::Frame::none()
                    .fill(ui.style().visuals.extreme_bg_color)
                    .rounding(8.0)
                    .inner_margin(12.0)
                    .show(ui, |ui| {
                        for (i, path) in recent_files.iter().take(5).enumerate() {
                            let file_name = std::path::Path::new(path)
                                .file_name()
                                .and_then(|n| n.to_str())
                                .unwrap_or(path);

                            let response = ui.horizontal(|ui| {
                                ui.label(icons::FILE);
                                ui.label(file_name);
                            }).response;

                            if response.interact(egui::Sense::click()).clicked() {
                                action = WelcomeAction::OpenRecent(path.clone());
                            }

                            if response.hovered() {
                                ui.painter().rect_stroke(
                                    response.rect,
                                    4.0,
                                    egui::Stroke::new(1.0, ui.visuals().selection.bg_fill),
                                );
                            }
                        }
                    });
            }

            ui.add_space(32.0);

            // Quick tips
            ui.label(
                RichText::new(format!("{} Quick Tips", icons::LIGHTBULB))
                    .size(14.0)
                    .strong(),
            );

            ui.add_space(8.0);

            let tips = [
                ("Dashboard", "Get a quick health overview of your flight"),
                ("Tune", "Analyze step response for PID tuning"),
                ("Vibe", "View frequency spectrum to diagnose noise"),
                ("Suggestions", "Get AI-powered tuning recommendations"),
            ];

            egui::Frame::none()
                .fill(ui.style().visuals.extreme_bg_color)
                .rounding(8.0)
                .inner_margin(12.0)
                .show(ui, |ui| {
                    for (tab, description) in &tips {
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(format!("• {}", tab)).strong());
                            ui.label(RichText::new(format!("- {}", description)).weak());
                        });
                    }
                });

            ui.add_space(32.0);

            // Version and links
            ui.label(
                RichText::new(format!("v{}", env!("CARGO_PKG_VERSION")))
                    .size(11.0)
                    .weak(),
            );
        });

        action
    }
}

/// Action requested by the welcome panel
#[derive(Clone)]
pub enum WelcomeAction {
    None,
    OpenFile,
    OpenRecent(String),
}

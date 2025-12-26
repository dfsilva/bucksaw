use egui::{Color32, FontId, RichText};

/// A global loading modal that displays a centered overlay with a spinner
/// Use this for long-running operations that block the UI
pub struct LoadingModal;

impl LoadingModal {
    /// Show the loading modal if is_loading is true
    /// Call this at the end of your update() function so it renders on top
    pub fn show(ctx: &egui::Context, is_loading: bool, message: &str) {
        if !is_loading {
            return;
        }

        // Create a semi-transparent overlay
        egui::Area::new(egui::Id::new("loading_modal_overlay"))
            .fixed_pos(egui::pos2(0.0, 0.0))
            .order(egui::Order::Foreground)
            .show(ctx, |ui| {
                let screen_rect = ctx.screen_rect();

                // Dark overlay background
                ui.painter().rect_filled(
                    screen_rect,
                    0.0,
                    Color32::from_rgba_unmultiplied(0, 0, 0, 180),
                );

                // Center the loading content
                let center = screen_rect.center();

                // Create a centered frame for the loading content
                egui::Area::new(egui::Id::new("loading_modal_content"))
                    .fixed_pos(center - egui::vec2(100.0, 40.0))
                    .order(egui::Order::Foreground)
                    .show(ctx, |ui| {
                        egui::Frame::popup(ui.style())
                            .fill(Color32::from_rgb(40, 40, 40))
                            .rounding(8.0)
                            .inner_margin(20.0)
                            .show(ui, |ui| {
                                ui.vertical_centered(|ui| {
                                    ui.spinner();
                                    ui.add_space(10.0);
                                    ui.label(
                                        RichText::new(message)
                                            .color(Color32::WHITE)
                                            .font(FontId::proportional(14.0)),
                                    );
                                });
                            });
                    });
            });
    }

    /// Helper function to show a simple "Loading..." modal
    pub fn show_simple(ctx: &egui::Context, is_loading: bool) {
        Self::show(ctx, is_loading, "Loading...");
    }
}

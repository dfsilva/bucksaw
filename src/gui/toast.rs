//! Toast notification system for showing brief feedback messages

use egui::{Color32, RichText};
use std::collections::VecDeque;

/// Types of toast notifications
#[derive(Clone, Copy, PartialEq)]
pub enum ToastKind {
    Info,
    Success,
    Warning,
    Error,
}

impl ToastKind {
    fn color(&self) -> Color32 {
        match self {
            ToastKind::Info => Color32::from_rgb(0x83, 0xa5, 0x98),    // Blue-gray
            ToastKind::Success => Color32::from_rgb(0x8e, 0xc0, 0x7c), // Green
            ToastKind::Warning => Color32::from_rgb(0xfa, 0xbd, 0x2f), // Yellow
            ToastKind::Error => Color32::from_rgb(0xfb, 0x49, 0x34),   // Red
        }
    }

    fn icon(&self) -> &'static str {
        use egui_phosphor::regular as icons;
        match self {
            ToastKind::Info => icons::INFO,
            ToastKind::Success => icons::CHECK_CIRCLE,
            ToastKind::Warning => icons::WARNING,
            ToastKind::Error => icons::X_CIRCLE,
        }
    }
}

/// A single toast notification
#[derive(Clone)]
pub struct Toast {
    pub message: String,
    pub kind: ToastKind,
    /// Time when the toast was created (egui time in seconds)
    pub created_at: f64,
    /// Duration to show the toast (in seconds)
    pub duration: f64,
}

impl Toast {
    pub fn new(message: impl Into<String>, kind: ToastKind) -> Self {
        Self {
            message: message.into(),
            kind,
            created_at: 0.0, // Will be set when added
            duration: 3.0,   // Default 3 seconds
        }
    }

    pub fn with_duration(mut self, duration: f64) -> Self {
        self.duration = duration;
        self
    }
}

/// Toast notification manager
pub struct Toaster {
    toasts: VecDeque<Toast>,
    /// Pending toasts that need to have their created_at set
    pending: Vec<Toast>,
}

impl Default for Toaster {
    fn default() -> Self {
        Self::new()
    }
}

impl Toaster {
    pub fn new() -> Self {
        Self {
            toasts: VecDeque::new(),
            pending: Vec::new(),
        }
    }

    /// Add a new toast notification (will be assigned timestamp on next show())
    pub fn add(&mut self, toast: Toast) {
        self.pending.push(toast);
    }

    /// Add an info toast
    pub fn info(&mut self, message: impl Into<String>) {
        self.add(Toast::new(message, ToastKind::Info));
    }

    /// Add a success toast
    pub fn success(&mut self, message: impl Into<String>) {
        self.add(Toast::new(message, ToastKind::Success));
    }

    /// Add a warning toast
    pub fn warning(&mut self, message: impl Into<String>) {
        self.add(Toast::new(message, ToastKind::Warning));
    }

    /// Add an error toast
    pub fn error(&mut self, message: impl Into<String>) {
        self.add(Toast::new(message, ToastKind::Error));
    }

    /// Show all active toasts. Call this at the end of your update() function.
    pub fn show(&mut self, ctx: &egui::Context) {
        // Get current time from egui (works on all platforms including WASM)
        let current_time = ctx.input(|i| i.time);

        // Process pending toasts
        for mut toast in self.pending.drain(..) {
            toast.created_at = current_time;
            self.toasts.push_back(toast);
            // Limit to 5 toasts max
            while self.toasts.len() > 5 {
                self.toasts.pop_front();
            }
        }

        // Remove expired toasts
        self.toasts
            .retain(|t| current_time - t.created_at < t.duration);

        if self.toasts.is_empty() {
            return;
        }

        // Request repaint for smooth fade animations
        ctx.request_repaint();

        let screen_rect = ctx.screen_rect();
        let toast_width = 350.0f32.min(screen_rect.width() - 40.0);
        let start_x = screen_rect.right() - toast_width - 20.0;
        let mut current_y = screen_rect.top() + 50.0; // Below menu bar

        for toast in &self.toasts {
            let elapsed = current_time - toast.created_at;
            let remaining = toast.duration - elapsed;

            // Fade in/out animation
            let alpha = if elapsed < 0.2 {
                (elapsed / 0.2) as f32
            } else if remaining < 0.3 {
                (remaining / 0.3) as f32
            } else {
                1.0
            };

            let alpha_u8 = (alpha * 255.0) as u8;

            egui::Area::new(egui::Id::new(format!("toast_{}", toast.created_at as u64)))
                .fixed_pos(egui::pos2(start_x, current_y))
                .order(egui::Order::Foreground)
                .interactable(false)
                .show(ctx, |ui| {
                    egui::Frame::none()
                        .fill(Color32::from_rgba_unmultiplied(40, 40, 40, alpha_u8))
                        .rounding(8.0)
                        .inner_margin(12.0)
                        .stroke(egui::Stroke::new(
                            1.0,
                            toast.kind.color().gamma_multiply(alpha),
                        ))
                        .show(ui, |ui| {
                            ui.set_min_width(toast_width - 24.0);
                            ui.horizontal(|ui| {
                                ui.label(
                                    RichText::new(toast.kind.icon())
                                        .color(toast.kind.color().gamma_multiply(alpha))
                                        .size(18.0),
                                );
                                ui.label(
                                    RichText::new(&toast.message)
                                        .color(Color32::from_rgba_unmultiplied(
                                            255, 255, 255, alpha_u8,
                                        ))
                                        .size(14.0),
                                );
                            });
                        });
                });

            current_y += 60.0; // Space for next toast
        }
    }
}

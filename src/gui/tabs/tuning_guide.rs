use egui::{Color32, RichText};
use egui_phosphor::regular as icons;

pub struct TuningGuideTab {
    current_step: usize,
}

struct TuningStep {
    title: &'static str,
    description: &'static str,
    action_items: &'static [&'static str],
    recommended_tools: &'static [&'static str],
}

const STEPS: &[TuningStep] = &[
    TuningStep {
        title: "1. Baselines & Filters",
        description: "Before tuning PIDs, ensure your filters are clean to avoid heating motors.",
        action_items: &[
            "Check 'Noise Analysis' in Dashboard for gyro noise levels.",
            "Verify motor temperatures (hover for 30s).",
            "Set filters: enable RPM Filter, set Dynamic Notch.",
        ],
        recommended_tools: &["Dashboard", "Filter", "Vibe (Spectrogram)"],
    },
    TuningStep {
        title: "2. Feedforward (Stick Tracking)",
        description: "Tune FF first to get good stick tracking without relying on P-term.",
        action_items: &[
            "Do crisp flips/rolls and check 'Feedforward' tab.",
            "Adjust FF Gain until gyro tracks setpoint transition well.",
            "If overshoot occurs at start of move, reduce FF or increase smoothing.",
        ],
        recommended_tools: &["Feedforward", "Tune (Step Response)"],
    },
    TuningStep {
        title: "3. P-Gain (Responsiveness)",
        description: "P-term provides the sharpness and holding power.",
        action_items: &[
            "Increase P until you see high-frequency oscillations (P-wobble) or overshoot.",
            "Back off P by 10-15% from the oscillation point.",
            "Check 'Step Response' in Suggestions tab for overshoot.",
        ],
        recommended_tools: &["Tune", "Suggestions"],
    },
    TuningStep {
        title: "4. D-Gain (Damping)",
        description: "D-term dampens the P-term and propwash.",
        action_items: &[
            "Increase D to reduce propwash oscillations (see Suggestions).",
            "Watch out for hot motors - D-term amplifies noise!",
            "Use 'Damping' slider in Setup tab to adjust D-gains globally.",
        ],
        recommended_tools: &["Suggestions (Propwash)", "Vibe (Noise)"],
    },
    TuningStep {
        title: "5. I-Term (Holding)",
        description: "I-term keeps the drone holding its angle against wind/drift.",
        action_items: &[
            "Check 'Anomalies' for I-term windup.",
            "If drone drifts in forward flight, increase I-gain.",
            "Tune 'Anti-Gravity' if nose dips during throttle punches.",
        ],
        recommended_tools: &["Anomalies", "Suggestions"],
    },
];

impl TuningGuideTab {
    pub fn new() -> Self {
        Self { current_step: 0 }
    }

    pub fn show(&mut self, ui: &mut egui::Ui) {
        ui.heading(format!("{} Betaflight Tuning Guide", icons::BOOK_OPEN));
        ui.label("Follow this interactive workflow to get a locked-in tune.");
        ui.add_space(10.0);

        // Progress Bar
        let progress = (self.current_step as f32 + 1.0) / STEPS.len() as f32;
        ui.add(egui::ProgressBar::new(progress).animate(true));
        ui.add_space(10.0);

        // Navigation
        ui.horizontal(|ui| {
            if ui.button(format!("{} Previous", icons::ARROW_LEFT)).clicked() && self.current_step > 0 {
                self.current_step -= 1;
            }
            ui.label(
                RichText::new(format!("Step {} of {}", self.current_step + 1, STEPS.len()))
                    .strong(),
            );
            if ui.button(format!("Next {}", icons::ARROW_RIGHT)).clicked() && self.current_step < STEPS.len() - 1 {
                self.current_step += 1;
            }
        });

        ui.separator();
        ui.add_space(10.0);

        // Current Step Content
        let step = &STEPS[self.current_step];

        egui::Frame::none()
            .fill(ui.style().visuals.extreme_bg_color)
            .rounding(8.0)
            .inner_margin(16.0)
            .show(ui, |ui| {
                ui.heading(RichText::new(step.title).color(Color32::LIGHT_BLUE));
                ui.add_space(8.0);
                ui.label(RichText::new(step.description).size(16.0));
                ui.add_space(16.0);

                ui.label(RichText::new(format!("{} Action Items:", icons::CHECK_SQUARE)).strong());
                for item in step.action_items {
                    ui.horizontal(|ui| {
                        ui.label("•");
                        ui.label(*item);
                    });
                }

                ui.add_space(16.0);
                ui.label(RichText::new("⚒ Recommended Tools:").strong());
                ui.horizontal(|ui| {
                    for tool in step.recommended_tools {
                        ui.label(RichText::new(*tool).code().color(Color32::YELLOW));
                    }
                });
            });
    }
}

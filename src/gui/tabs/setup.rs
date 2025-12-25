use std::sync::Arc;

use egui::{Color32, RichText, Vec2};
use egui_plot::{Legend, Line, Plot, PlotPoints};

use crate::flight_data::FlightData;
use crate::gui::colors::Colors;

/// Betaflight-style dark theme colors
mod betaflight_colors {
    use egui::Color32;

    pub const BG_DARK: Color32 = Color32::from_rgb(0x2e, 0x2e, 0x2e);
    pub const BG_SECTION: Color32 = Color32::from_rgb(0x3a, 0x3a, 0x3a);
    pub const ACCENT_ORANGE: Color32 = Color32::from_rgb(0xff, 0xbb, 0x00);
    pub const TOGGLE_ON: Color32 = Color32::from_rgb(0xe8, 0x9f, 0x17);
    pub const TOGGLE_OFF: Color32 = Color32::from_rgb(0x55, 0x55, 0x55);
    pub const TEXT_LABEL: Color32 = Color32::from_rgb(0xaa, 0xaa, 0xaa);
    pub const ROLL_RED: Color32 = Color32::from_rgb(0xcf, 0x3d, 0x34);
    pub const PITCH_GREEN: Color32 = Color32::from_rgb(0x55, 0x8b, 0x2f);
    pub const YAW_YELLOW: Color32 = Color32::from_rgb(0xe8, 0x9f, 0x17);
    pub const SEPARATOR: Color32 = Color32::from_rgb(0x4a, 0x4a, 0x4a);
}

/// Sub-tabs for the Setup tab
#[derive(Default, Clone, Copy, PartialEq)]
pub enum SetupSubTab {
    #[default]
    PIDProfileSettings,
    RateprofileSettings,
    FilterSettings,
}

impl SetupSubTab {
    fn label(&self) -> &'static str {
        match self {
            SetupSubTab::PIDProfileSettings => "PID Profile Settings",
            SetupSubTab::RateprofileSettings => "Rateprofile Settings",
            SetupSubTab::FilterSettings => "Filter Settings",
        }
    }
}

/// Setup Information Tab - displays flight controller configuration in Betaflight style
pub struct SetupTab {
    fd: Arc<FlightData>,
    sub_tab: SetupSubTab,
}

impl SetupTab {
    pub fn new(fd: Arc<FlightData>) -> Self {
        Self {
            fd,
            sub_tab: SetupSubTab::default(),
        }
    }

    pub fn show(&mut self, ui: &mut egui::Ui) {
        let _colors = Colors::get(ui);

        // Top header bar with profile selectors
        self.show_header_bar(ui);

        ui.add_space(4.0);

        // Sub-tab selector
        self.show_sub_tab_selector(ui);

        ui.add_space(8.0);

        // Content based on selected sub-tab
        egui::ScrollArea::both().show(ui, |ui| match self.sub_tab {
            SetupSubTab::PIDProfileSettings => self.show_pid_profile_settings(ui),
            SetupSubTab::RateprofileSettings => self.show_rateprofile_settings(ui),
            SetupSubTab::FilterSettings => self.show_filter_settings(ui),
        });
    }

    fn show_header_bar(&self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.heading("PID Tuning");

            ui.add_space(20.0);

            // Profile selector
            Self::styled_label(ui, "Profile");
            ui.add(egui::Button::new("Profile 1 ▼").min_size(Vec2::new(80.0, 20.0)));

            ui.add_space(10.0);

            // Rateprofile selector
            Self::styled_label(ui, "Rateprofile");
            ui.add(egui::Button::new("Rateprofile 1 ▼").min_size(Vec2::new(100.0, 20.0)));

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("Show all PIDs").clicked() {}
                if ui.button("Reset this Profile").clicked() {}
                if ui.button("Copy rateprofile").clicked() {}
                if ui.button("Copy profile").clicked() {}
            });
        });
    }

    fn show_sub_tab_selector(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            for tab in [
                SetupSubTab::PIDProfileSettings,
                SetupSubTab::RateprofileSettings,
                SetupSubTab::FilterSettings,
            ] {
                let is_selected = self.sub_tab == tab;
                let text = RichText::new(tab.label());
                let text = if is_selected {
                    text.color(betaflight_colors::ACCENT_ORANGE)
                } else {
                    text.color(betaflight_colors::TEXT_LABEL)
                };

                if ui.selectable_label(is_selected, text).clicked() {
                    self.sub_tab = tab;
                }

                ui.separator();
            }
        });

        // Orange underline for selected tab
        let painter = ui.painter();
        let rect = ui.min_rect();
        painter.line_segment(
            [rect.left_bottom(), rect.right_bottom()],
            egui::Stroke::new(2.0, betaflight_colors::SEPARATOR),
        );
    }

    // ============================================
    // PID Profile Settings Sub-Tab
    // ============================================
    fn show_pid_profile_settings(&self, ui: &mut egui::Ui) {
        // Get available width and split between columns
        let available_width = ui.available_width();
        let left_width = (available_width * 0.55).min(550.0).max(400.0);
        let right_width = (available_width * 0.42).min(400.0).max(280.0);

        ui.horizontal(|ui| {
            // Left column - PID table and sliders
            ui.vertical(|ui| {
                ui.set_min_width(left_width);
                ui.set_max_width(left_width);
                self.show_pid_table(ui);
            });

            ui.separator();

            // Right column - PID Controller Settings
            ui.vertical(|ui| {
                ui.set_min_width(right_width);
                self.show_pid_controller_settings(ui);
            });
        });
    }

    fn show_pid_table(&self, ui: &mut egui::Ui) {
        let headers = &self.fd.unknown_headers;

        Self::styled_label(ui, "PID profile name");
        ui.add_space(4.0);

        // Column headers
        ui.horizontal(|ui| {
            ui.label(RichText::new("Basic/Acro").strong());
            ui.add_space(40.0);
            for col in [
                "Proportional",
                "Integral",
                "D Max",
                "Derivative",
                "Feedforward",
            ] {
                ui.label(RichText::new(col).strong().size(11.0));
                ui.add_space(20.0);
            }
        });

        // Parse shared multi-axis values
        // d_min contains d_min for roll,pitch,yaw (e.g., "50,65,0")
        let d_min_values: Vec<&str> = headers
            .get("d_min")
            .map(|s| s.split(',').collect())
            .unwrap_or_default();

        // ff_weight contains feedforward for roll,pitch,yaw (e.g., "100,105,80")
        let ff_values: Vec<&str> = headers
            .get("ff_weight")
            .map(|s| s.split(',').collect())
            .unwrap_or_default();

        // Axis rows with axis index for d_min and ff_weight
        let axes = [
            ("ROLL", betaflight_colors::ROLL_RED, "rollPID", 0),
            ("PITCH", betaflight_colors::PITCH_GREEN, "pitchPID", 1),
            ("YAW", betaflight_colors::YAW_YELLOW, "yawPID", 2),
        ];

        for (axis, color, pid_key, axis_idx) in axes {
            ui.horizontal(|ui| {
                // Colored axis indicator
                let (rect, _) = ui.allocate_exact_size(Vec2::new(60.0, 24.0), egui::Sense::hover());
                ui.painter().rect_filled(rect, 2.0, color);
                ui.painter().text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    axis,
                    egui::FontId::proportional(12.0),
                    Color32::WHITE,
                );

                // PID values from axis-specific header (format: "P,I,D")
                let pid_value = headers.get(pid_key).map(|s| s.as_str()).unwrap_or("");
                let pid_parts: Vec<&str> = pid_value.split(',').collect();

                // P (Proportional) - index 0
                let p_val = pid_parts.get(0).unwrap_or(&"--");
                Self::value_field(ui, p_val);

                // I (Integral) - index 1
                let i_val = pid_parts.get(1).unwrap_or(&"--");
                Self::value_field(ui, i_val);

                // D Max - index 2 from PID (the maximum D value)
                let d_max_val = pid_parts.get(2).unwrap_or(&"--");
                Self::value_field(ui, d_max_val);

                // Derivative (D Min) - from d_min header for this axis
                let d_min_val = d_min_values.get(axis_idx).unwrap_or(&"--");
                Self::value_field(ui, d_min_val);

                // Feedforward - from ff_weight header for this axis
                let ff_val = ff_values.get(axis_idx).unwrap_or(&"--");
                Self::value_field(ui, ff_val);
            });
        }
    }

    fn show_pid_controller_settings(&self, ui: &mut egui::Ui) {
        Self::section_header(ui, "PID Controller Settings");

        let headers = &self.fd.unknown_headers;

        // Feedforward section
        egui::Grid::new("pid_controller_grid")
            .num_columns(2)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                ui.label("Feed-");
                ui.horizontal(|ui| {
                    Self::value_field(
                        ui,
                        headers
                            .get("feedforward_jitter_factor")
                            .map(|s| s.as_str())
                            .unwrap_or("10"),
                    );
                    ui.label("Jitter Reduction");
                });
                ui.end_row();

                ui.label("forward");
                ui.horizontal(|ui| {
                    Self::value_field(
                        ui,
                        headers
                            .get("feedforward_smooth_factor")
                            .map(|s| s.as_str())
                            .unwrap_or("35"),
                    );
                    ui.label("Smoothness");
                });
                ui.end_row();

                ui.label("");
                ui.horizontal(|ui| {
                    let averaging = headers
                        .get("feedforward_averaging")
                        .map(|s| match s.as_str() {
                            "0" => "OFF",
                            "1" => "2 Point",
                            "2" => "3 Point",
                            "3" => "4 Point",
                            _ => s.as_str(),
                        })
                        .unwrap_or("2 Point");
                    ui.label(averaging);
                    ui.label("Averaging");
                });
                ui.end_row();

                ui.label("");
                ui.horizontal(|ui| {
                    Self::value_field(
                        ui,
                        headers
                            .get("feedforward_boost")
                            .map(|s| s.as_str())
                            .unwrap_or("15"),
                    );
                    ui.label("Boost");
                });
                ui.end_row();

                ui.label("");
                ui.horizontal(|ui| {
                    Self::value_field(
                        ui,
                        headers
                            .get("feedforward_max_rate_limit")
                            .map(|s| s.as_str())
                            .unwrap_or("90"),
                    );
                    ui.label("Max Rate Limit");
                });
                ui.end_row();

                ui.label("");
                ui.horizontal(|ui| {
                    Self::value_field(
                        ui,
                        headers
                            .get("feedforward_transition")
                            .map(|s| s.as_str())
                            .unwrap_or("0"),
                    );
                    ui.label("Transition");
                });
                ui.end_row();
            });

        ui.add_space(8.0);

        // I Term Relax
        let iterm_relax_enabled = headers.get("iterm_relax").map(|s| s != "0").unwrap_or(true);
        Self::toggle_row(ui, "I Term Relax", iterm_relax_enabled);
        egui::Grid::new("iterm_relax_grid")
            .num_columns(2)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                let axes = headers
                    .get("iterm_relax")
                    .map(|s| match s.as_str() {
                        "1" => "RP",
                        "2" => "RPY",
                        _ => s.as_str(),
                    })
                    .unwrap_or("RP");
                ui.label(axes);
                ui.label("Axes");
                ui.end_row();

                let relax_type = headers
                    .get("iterm_relax_type")
                    .map(|s| match s.as_str() {
                        "0" => "Gyro",
                        "1" => "Setpoint",
                        _ => s.as_str(),
                    })
                    .unwrap_or("Setpoint");
                ui.label(relax_type);
                ui.label("Type");
                ui.end_row();

                ui.horizontal(|ui| {
                    Self::value_field(
                        ui,
                        headers
                            .get("iterm_relax_cutoff")
                            .map(|s| s.as_str())
                            .unwrap_or("10"),
                    );
                });
                ui.label("Cutoff");
                ui.end_row();
            });

        ui.add_space(8.0);

        // Anti Gravity
        let anti_gravity_enabled = headers
            .get("anti_gravity_gain")
            .map(|s| s != "0")
            .unwrap_or(true);
        Self::toggle_row(ui, "Anti Gravity", anti_gravity_enabled);
        ui.horizontal(|ui| {
            ui.label("Permanently enable");
        });
        ui.horizontal(|ui| {
            // Anti-gravity gain is stored as integer but displayed with decimal (80 -> 8.0)
            let gain_str = headers
                .get("anti_gravity_gain")
                .and_then(|s| s.parse::<f32>().ok())
                .map(|v| format!("{:.1}", v / 10.0))
                .unwrap_or_else(|| "8.0".to_string());
            Self::value_field(ui, &gain_str);
            ui.label("Gain");
        });

        ui.add_space(8.0);

        // I Term Rotation
        let iterm_rotation = headers
            .get("iterm_rotation")
            .map(|s| s != "0")
            .unwrap_or(false);
        Self::toggle_row(ui, "I Term Rotation", iterm_rotation);

        ui.add_space(8.0);

        // Dynamic Damping (D Max)
        Self::subsection_header(ui, "Dynamic Damping");
        egui::Grid::new("dynamic_damping_grid")
            .num_columns(2)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    Self::value_field(
                        ui,
                        headers
                            .get("d_max_gain")
                            .map(|s| s.as_str())
                            .unwrap_or("37"),
                    );
                });
                ui.label("Gain");
                ui.end_row();

                ui.horizontal(|ui| {
                    Self::value_field(
                        ui,
                        headers
                            .get("d_max_advance")
                            .map(|s| s.as_str())
                            .unwrap_or("20"),
                    );
                });
                ui.label("Advance");
                ui.end_row();
            });

        ui.add_space(8.0);
        Self::section_header(ui, "Throttle and Motor Settings");
        egui::Grid::new("throttle_motor_grid")
            .num_columns(2)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    Self::value_field(
                        ui,
                        headers
                            .get("throttle_boost")
                            .map(|s| s.as_str())
                            .unwrap_or("5"),
                    );
                });
                ui.label("Throttle Boost");
                ui.end_row();

                ui.horizontal(|ui| {
                    Self::value_field(
                        ui,
                        headers
                            .get("motor_output_limit")
                            .map(|s| s.as_str())
                            .unwrap_or("100"),
                    );
                });
                ui.label("Motor Output Limit");
                ui.end_row();

                ui.horizontal(|ui| {
                    Self::value_field(
                        ui,
                        headers
                            .get("dyn_idle_min_rpm")
                            .map(|s| s.as_str())
                            .unwrap_or("0"),
                    );
                });
                ui.label("Dynamic Idle Value [* 100 RPM]");
                ui.end_row();
            });

        // Vbat Sag Compensation
        let vbat_sag_enabled = headers
            .get("vbat_sag_compensation")
            .map(|s| s != "0")
            .unwrap_or(true);
        Self::toggle_row(ui, "Vbat Sag Compensation", vbat_sag_enabled);
        ui.horizontal(|ui| {
            Self::value_field(
                ui,
                headers
                    .get("vbat_sag_compensation")
                    .map(|s| s.as_str())
                    .unwrap_or("100"),
            );
            ui.label("%");
        });

        // Thrust Linearization
        let thrust_linear_enabled = headers
            .get("thrust_linear")
            .map(|s| s != "0")
            .unwrap_or(false);
        Self::toggle_row(ui, "Thrust Linearization", thrust_linear_enabled);
        ui.horizontal(|ui| {
            Self::value_field(
                ui,
                headers
                    .get("thrust_linear")
                    .map(|s| s.as_str())
                    .unwrap_or("10"),
            );
            ui.label("%");
        });

        ui.add_space(8.0);
        Self::section_header(ui, "TPA Mode");
        egui::Grid::new("tpa_grid")
            .num_columns(3)
            .spacing([20.0, 4.0])
            .show(ui, |ui| {
                ui.label("TPA Mode");
                ui.label(RichText::new("TPA Rate (%)").strong());
                ui.label(RichText::new("TPA Breakpoint (µs)").strong());
                ui.end_row();

                let tpa_mode = headers
                    .get("tpa_mode")
                    .map(|s| match s.as_str() {
                        "0" => "OFF",
                        "1" => "D",
                        "2" => "PD",
                        _ => s.as_str(),
                    })
                    .unwrap_or("D");
                ui.label(tpa_mode);
                Self::value_field(
                    ui,
                    headers.get("tpa_rate").map(|s| s.as_str()).unwrap_or("50"),
                );
                Self::value_field(
                    ui,
                    headers
                        .get("tpa_breakpoint")
                        .map(|s| s.as_str())
                        .unwrap_or("1200"),
                );
                ui.end_row();
            });

        ui.add_space(8.0);
        Self::section_header(ui, "Miscellaneous Settings");
        egui::Grid::new("misc_grid")
            .num_columns(2)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                ui.label("Disable");
                ui.label("Cell Count - for auto Profile switching");
                ui.end_row();

                ui.horizontal(|ui| {
                    Self::value_field(
                        ui,
                        headers
                            .get("acro_trainer_angle_limit")
                            .map(|s| s.as_str())
                            .unwrap_or("20"),
                    );
                });
                ui.label("Acro Trainer Angle Limit");
                ui.end_row();
            });

        let integrated_yaw = headers
            .get("use_integrated_yaw")
            .map(|s| s != "0")
            .unwrap_or(false);
        Self::toggle_row(ui, "Integrated Yaw", integrated_yaw);

        ui.horizontal(|ui| {
            Self::value_field(
                ui,
                headers
                    .get("abs_control_gain")
                    .map(|s| s.as_str())
                    .unwrap_or("0"),
            );
            ui.label("Absolute Control");
        });
    }

    // ============================================
    // Rateprofile Settings Sub-Tab
    // ============================================
    fn show_rateprofile_settings(&self, ui: &mut egui::Ui) {
        let headers = &self.fd.unknown_headers;

        ui.horizontal(|ui| {
            // Left column
            ui.vertical(|ui| {
                ui.set_min_width(400.0);

                Self::styled_label(ui, "Rate profile name");
                ui.add_space(8.0);

                // Rates Type
                Self::section_header(ui, "Rates Type");
                let rates_type = headers
                    .get("rates_type")
                    .unwrap_or(&"Actual".to_string())
                    .clone();
                ui.label(RichText::new(&rates_type).strong());

                ui.add_space(8.0);
                self.show_rates_table(ui);

                ui.add_space(12.0);
                Self::section_header(ui, "Rates Preview");

                // Calculate rates curves for each axis
                let get_rate_val = |key: &str, default: f32| -> f32 {
                    headers
                        .get(key)
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(default)
                };

                // Get rate parameters for each axis
                let roll_center = get_rate_val("roll_rc_rate", 30.0);
                let roll_max = get_rate_val("roll_srate", 800.0);
                let roll_expo = get_rate_val("roll_expo", 0.35);

                let pitch_center = get_rate_val("pitch_rc_rate", 30.0);
                let pitch_max = get_rate_val("pitch_srate", 800.0);
                let pitch_expo = get_rate_val("pitch_expo", 0.35);

                let yaw_center = get_rate_val("yaw_rc_rate", 30.0);
                let yaw_max = get_rate_val("yaw_srate", 650.0);
                let yaw_expo = get_rate_val("yaw_expo", 0.35);

                // Betaflight Actual rates formula
                // rate = center_sensitivity * stick + (max_rate - center_sensitivity) * stick^3 * (1 - expo + expo * stick^2)
                let calc_rate = |stick: f64, center: f64, max_rate: f64, expo: f64| -> f64 {
                    let stick_abs = stick.abs();
                    let expo_factor = 1.0 - expo + expo * stick_abs * stick_abs;
                    center * stick_abs + (max_rate - center) * stick_abs.powi(3) * expo_factor
                };

                // Generate points for each axis (stick input 0-100%)
                let steps = 50;
                let roll_points: PlotPoints = (0..=steps)
                    .map(|i| {
                        let stick = i as f64 / steps as f64;
                        let rate =
                            calc_rate(stick, roll_center as f64, roll_max as f64, roll_expo as f64);
                        [stick * 100.0, rate]
                    })
                    .collect();

                let pitch_points: PlotPoints = (0..=steps)
                    .map(|i| {
                        let stick = i as f64 / steps as f64;
                        let rate = calc_rate(
                            stick,
                            pitch_center as f64,
                            pitch_max as f64,
                            pitch_expo as f64,
                        );
                        [stick * 100.0, rate]
                    })
                    .collect();

                let yaw_points: PlotPoints = (0..=steps)
                    .map(|i| {
                        let stick = i as f64 / steps as f64;
                        let rate =
                            calc_rate(stick, yaw_center as f64, yaw_max as f64, yaw_expo as f64);
                        [stick * 100.0, rate]
                    })
                    .collect();

                Plot::new("rates_preview_plot")
                    .legend(Legend::default())
                    .height(200.0)
                    .width(350.0)
                    .x_axis_label("Stick Input (%)")
                    .y_axis_label("Rate (deg/s)")
                    .show(ui, |plot_ui| {
                        plot_ui.line(
                            Line::new(roll_points)
                                .name("Roll")
                                .color(betaflight_colors::ROLL_RED)
                                .width(2.0),
                        );
                        plot_ui.line(
                            Line::new(pitch_points)
                                .name("Pitch")
                                .color(betaflight_colors::PITCH_GREEN)
                                .width(2.0),
                        );
                        plot_ui.line(
                            Line::new(yaw_points)
                                .name("Yaw")
                                .color(betaflight_colors::YAW_YELLOW)
                                .width(2.0),
                        );
                    });
            });

            ui.separator();

            // Right column - Throttle settings
            ui.vertical(|ui| {
                ui.set_min_width(300.0);

                Self::section_header(ui, "Throttle Limit");
                egui::Grid::new("throttle_limit_grid")
                    .num_columns(2)
                    .spacing([20.0, 4.0])
                    .show(ui, |ui| {
                        ui.label("Throttle Limit");
                        ui.label(
                            headers
                                .get("throttle_limit_type")
                                .unwrap_or(&"OFF".to_string()),
                        );
                        ui.end_row();

                        ui.label("Throttle Limit %");
                        Self::value_field(
                            ui,
                            headers
                                .get("throttle_limit_percent")
                                .map(|s| s.as_str())
                                .unwrap_or("100"),
                        );
                        ui.end_row();
                    });

                ui.add_space(8.0);
                egui::Grid::new("throttle_midexpo_grid")
                    .num_columns(2)
                    .spacing([20.0, 4.0])
                    .show(ui, |ui| {
                        ui.label("Throttle MID");
                        Self::value_field(
                            ui,
                            headers.get("thr_mid").map(|s| s.as_str()).unwrap_or("0.40"),
                        );
                        ui.end_row();

                        ui.label("Throttle EXPO");
                        Self::value_field(
                            ui,
                            headers
                                .get("thr_expo")
                                .map(|s| s.as_str())
                                .unwrap_or("0.70"),
                        );
                        ui.end_row();
                    });

                ui.add_space(8.0);
                Self::section_header(ui, "Throttle Curve Preview");
                ui.label("0% = 0%");

                ui.add_space(20.0);
                Self::section_header(ui, "Rates Preview");
                // Placeholder for 3D quad preview
                let (rect, _) =
                    ui.allocate_exact_size(Vec2::new(200.0, 150.0), egui::Sense::hover());
                ui.painter()
                    .rect_filled(rect, 4.0, betaflight_colors::BG_SECTION);
                ui.painter().text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    "3D Preview",
                    egui::FontId::proportional(14.0),
                    betaflight_colors::TEXT_LABEL,
                );
            });
        });
    }

    fn show_rates_table(&self, ui: &mut egui::Ui) {
        let headers = &self.fd.unknown_headers;

        // Column headers
        ui.horizontal(|ui| {
            ui.label(RichText::new("Basic/Acro Rates").strong());
            ui.add_space(20.0);
            for col in ["Center Sensitivity", "Max Rate", "Expo", "Max Vel (deg/s)"] {
                ui.label(RichText::new(col).strong().size(11.0));
                ui.add_space(15.0);
            }
        });

        let axes = [
            (
                "ROLL",
                betaflight_colors::ROLL_RED,
                "roll_rc_rate",
                "roll_srate",
                "roll_expo",
            ),
            (
                "PITCH",
                betaflight_colors::PITCH_GREEN,
                "pitch_rc_rate",
                "pitch_srate",
                "pitch_expo",
            ),
            (
                "YAW",
                betaflight_colors::YAW_YELLOW,
                "yaw_rc_rate",
                "yaw_srate",
                "yaw_expo",
            ),
        ];

        for (axis, color, rc_rate_key, srate_key, expo_key) in axes {
            ui.horizontal(|ui| {
                // Colored axis indicator
                let (rect, _) = ui.allocate_exact_size(Vec2::new(50.0, 22.0), egui::Sense::hover());
                ui.painter().rect_filled(rect, 2.0, color);
                ui.painter().text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    axis,
                    egui::FontId::proportional(11.0),
                    Color32::WHITE,
                );

                // Values
                Self::value_field(
                    ui,
                    headers.get(rc_rate_key).map(|s| s.as_str()).unwrap_or("30"),
                );
                Self::value_field(
                    ui,
                    headers.get(srate_key).map(|s| s.as_str()).unwrap_or("800"),
                );
                Self::value_field(
                    ui,
                    headers.get(expo_key).map(|s| s.as_str()).unwrap_or("0.35"),
                );

                // Max velocity (calculated or from header)
                let max_vel = match axis {
                    "ROLL" | "PITCH" => "800",
                    "YAW" => "650",
                    _ => "800",
                };
                Self::value_field(ui, max_vel);
            });
        }

        // Angle Mode row
        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.label(RichText::new("Angle Mode").color(betaflight_colors::TEXT_LABEL));
            ui.add_space(50.0);
            Self::value_field(ui, "2.3...60");
            Self::value_field(ui, "2.3...60");
        });
    }

    // ============================================
    // Filter Settings Sub-Tab
    // ============================================
    fn show_filter_settings(&self, ui: &mut egui::Ui) {
        let headers = &self.fd.unknown_headers;

        ui.horizontal(|ui| {
            // Left column - Profile independent
            ui.vertical(|ui| {
                ui.set_min_width(450.0);

                Self::section_header(ui, "Profile Independent Filter Settings");

                self.show_gyro_lowpass_filters(ui, headers);
                ui.add_space(8.0);
                self.show_gyro_notch_filters(ui, headers);
                ui.add_space(8.0);
                self.show_gyro_rpm_filter(ui, headers);
                ui.add_space(8.0);
                self.show_dynamic_notch_filter(ui, headers);
            });

            ui.separator();

            // Right column - Profile dependent
            ui.vertical(|ui| {
                ui.set_min_width(400.0);

                Self::section_header(ui, "Profile Dependent Filter Settings");

                self.show_dterm_lowpass_filters(ui, headers);
                ui.add_space(8.0);
                self.show_dterm_notch_filter(ui, headers);
                ui.add_space(8.0);
                self.show_yaw_lowpass_filter(ui, headers);
            });
        });
    }

    fn show_gyro_lowpass_filters(
        &self,
        ui: &mut egui::Ui,
        headers: &std::collections::HashMap<String, String>,
    ) {
        Self::subsection_header(ui, "Gyro Lowpass Filters");

        // Gyro Lowpass 1
        ui.horizontal(|ui| {
            Self::toggle_indicator(ui, true);
            ui.label("Gyro Lowpass 1");
        });

        egui::Grid::new("gyro_lpf1_grid")
            .num_columns(4)
            .spacing([15.0, 4.0])
            .show(ui, |ui| {
                ui.label("Mode");
                ui.label(
                    headers
                        .get("gyro_lpf1_type")
                        .unwrap_or(&"STATIC".to_string()),
                );
                ui.end_row();

                ui.label("Static Cutoff Frequency [Hz]");
                Self::value_field(
                    ui,
                    headers
                        .get("gyro_lpf1_static_hz")
                        .map(|s| s.as_str())
                        .unwrap_or("200"),
                );
                ui.end_row();

                ui.label("Filter Type");
                ui.label(headers.get("gyro_lpf1_type").unwrap_or(&"PT1".to_string()));
                ui.end_row();
            });

        ui.add_space(4.0);

        // Gyro Lowpass 2
        ui.horizontal(|ui| {
            Self::toggle_indicator(ui, true);
            ui.label("Gyro Lowpass 2");
        });

        egui::Grid::new("gyro_lpf2_grid")
            .num_columns(4)
            .spacing([15.0, 4.0])
            .show(ui, |ui| {
                ui.label("Static Cutoff Frequency [Hz]");
                Self::value_field(
                    ui,
                    headers
                        .get("gyro_lpf2_static_hz")
                        .map(|s| s.as_str())
                        .unwrap_or("1000"),
                );
                ui.end_row();

                ui.label("Filter Type");
                ui.label(headers.get("gyro_lpf2_type").unwrap_or(&"PT1".to_string()));
                ui.end_row();
            });
    }

    fn show_gyro_notch_filters(
        &self,
        ui: &mut egui::Ui,
        headers: &std::collections::HashMap<String, String>,
    ) {
        Self::subsection_header(ui, "Gyro Notch Filters");

        let notch1_enabled = headers
            .get("gyro_notch1_hz")
            .map(|s| s != "0")
            .unwrap_or(false);
        ui.horizontal(|ui| {
            Self::toggle_indicator(ui, notch1_enabled);
            ui.label("Gyro Notch Filter 1");
        });

        let notch2_enabled = headers
            .get("gyro_notch2_hz")
            .map(|s| s != "0")
            .unwrap_or(false);
        ui.horizontal(|ui| {
            Self::toggle_indicator(ui, notch2_enabled);
            ui.label("Gyro Notch Filter 2");
        });
    }

    fn show_gyro_rpm_filter(
        &self,
        ui: &mut egui::Ui,
        headers: &std::collections::HashMap<String, String>,
    ) {
        Self::subsection_header(ui, "Gyro RPM Filter");

        let rpm_enabled = headers
            .get("rpm_filter_harmonics")
            .map(|s| s != "0")
            .unwrap_or(true);
        ui.horizontal(|ui| {
            Self::toggle_indicator(ui, rpm_enabled);
            ui.label("Gyro RPM Filter");
        });

        egui::Grid::new("rpm_filter_grid")
            .num_columns(2)
            .spacing([15.0, 4.0])
            .show(ui, |ui| {
                ui.label("Gyro RPM Filter Harmonics Number");
                Self::value_field(
                    ui,
                    headers
                        .get("rpm_filter_harmonics")
                        .map(|s| s.as_str())
                        .unwrap_or("3"),
                );
                ui.end_row();

                ui.label("Gyro RPM Filter Min Frequency [Hz]");
                Self::value_field(
                    ui,
                    headers
                        .get("rpm_filter_min_hz")
                        .map(|s| s.as_str())
                        .unwrap_or("90"),
                );
                ui.end_row();
            });
    }

    fn show_dynamic_notch_filter(
        &self,
        ui: &mut egui::Ui,
        headers: &std::collections::HashMap<String, String>,
    ) {
        Self::subsection_header(ui, "Dynamic Notch Filter");

        let dyn_enabled = headers
            .get("dyn_notch_count")
            .map(|s| s != "0")
            .unwrap_or(true);
        ui.horizontal(|ui| {
            Self::toggle_indicator(ui, dyn_enabled);
            ui.label("Dynamic Notch Filter");
        });

        egui::Grid::new("dyn_notch_grid")
            .num_columns(2)
            .spacing([15.0, 4.0])
            .show(ui, |ui| {
                ui.label("Notch Count");
                Self::value_field(
                    ui,
                    headers
                        .get("dyn_notch_count")
                        .map(|s| s.as_str())
                        .unwrap_or("3"),
                );
                ui.end_row();

                ui.label("Q factor");
                Self::value_field(
                    ui,
                    headers
                        .get("dyn_notch_q")
                        .map(|s| s.as_str())
                        .unwrap_or("500"),
                );
                ui.end_row();

                ui.label("Min Frequency [Hz]");
                Self::value_field(
                    ui,
                    headers
                        .get("dyn_notch_min_hz")
                        .map(|s| s.as_str())
                        .unwrap_or("80"),
                );
                ui.end_row();

                ui.label("Max Frequency [Hz]");
                Self::value_field(
                    ui,
                    headers
                        .get("dyn_notch_max_hz")
                        .map(|s| s.as_str())
                        .unwrap_or("500"),
                );
                ui.end_row();
            });
    }

    fn show_dterm_lowpass_filters(
        &self,
        ui: &mut egui::Ui,
        headers: &std::collections::HashMap<String, String>,
    ) {
        Self::subsection_header(ui, "D Term Lowpass Filters");

        // D Term Lowpass 1
        ui.horizontal(|ui| {
            Self::toggle_indicator(ui, true);
            ui.label("D Term Lowpass 1");
        });

        egui::Grid::new("dterm_lpf1_grid")
            .num_columns(2)
            .spacing([15.0, 4.0])
            .show(ui, |ui| {
                ui.label("Mode");
                ui.label(
                    headers
                        .get("dterm_lpf1_type")
                        .unwrap_or(&"STATIC".to_string()),
                );
                ui.end_row();

                ui.label("Static Cutoff Frequency [Hz]");
                Self::value_field(
                    ui,
                    headers
                        .get("dterm_lpf1_static_hz")
                        .map(|s| s.as_str())
                        .unwrap_or(""),
                );
                ui.end_row();

                ui.label("Filter Type");
                ui.label(headers.get("dterm_lpf1_type").unwrap_or(&"PT1".to_string()));
                ui.end_row();
            });

        ui.add_space(4.0);

        // D Term Lowpass 2
        ui.horizontal(|ui| {
            Self::toggle_indicator(ui, true);
            ui.label("D Term Lowpass 2");
        });

        egui::Grid::new("dterm_lpf2_grid")
            .num_columns(2)
            .spacing([15.0, 4.0])
            .show(ui, |ui| {
                ui.label("Static Cutoff Frequency [Hz]");
                Self::value_field(
                    ui,
                    headers
                        .get("dterm_lpf2_static_hz")
                        .map(|s| s.as_str())
                        .unwrap_or(""),
                );
                ui.end_row();

                ui.label("Filter Type");
                ui.label(headers.get("dterm_lpf2_type").unwrap_or(&"PT1".to_string()));
                ui.end_row();
            });
    }

    fn show_dterm_notch_filter(
        &self,
        ui: &mut egui::Ui,
        headers: &std::collections::HashMap<String, String>,
    ) {
        Self::subsection_header(ui, "D Term Notch Filter");

        let notch_enabled = headers
            .get("dterm_notch_hz")
            .map(|s| s != "0")
            .unwrap_or(false);
        ui.horizontal(|ui| {
            Self::toggle_indicator(ui, notch_enabled);
            ui.label("D Term Notch Filter");
        });
    }

    fn show_yaw_lowpass_filter(
        &self,
        ui: &mut egui::Ui,
        headers: &std::collections::HashMap<String, String>,
    ) {
        Self::subsection_header(ui, "Yaw Lowpass Filter");

        let yaw_enabled = headers.get("yaw_lpf_hz").map(|s| s != "0").unwrap_or(true);
        ui.horizontal(|ui| {
            Self::toggle_indicator(ui, yaw_enabled);
            ui.label("Yaw Lowpass Filter");
        });

        egui::Grid::new("yaw_lpf_grid")
            .num_columns(2)
            .spacing([15.0, 4.0])
            .show(ui, |ui| {
                ui.label("Static Cutoff Frequency [Hz]");
                Self::value_field(
                    ui,
                    headers
                        .get("yaw_lpf_hz")
                        .map(|s| s.as_str())
                        .unwrap_or("50"),
                );
                ui.end_row();
            });
    }

    // ============================================
    // Helper Methods
    // ============================================
    fn styled_label(ui: &mut egui::Ui, text: &str) {
        ui.label(RichText::new(text).color(betaflight_colors::TEXT_LABEL));
    }

    fn section_header(ui: &mut egui::Ui, title: &str) {
        ui.add_space(4.0);
        ui.label(
            RichText::new(title)
                .strong()
                .color(betaflight_colors::ACCENT_ORANGE),
        );
        ui.separator();
    }

    fn subsection_header(ui: &mut egui::Ui, title: &str) {
        ui.label(
            RichText::new(title)
                .strong()
                .color(betaflight_colors::TEXT_LABEL),
        );
    }

    fn toggle_indicator(ui: &mut egui::Ui, enabled: bool) {
        let (rect, _) = ui.allocate_exact_size(Vec2::new(40.0, 18.0), egui::Sense::hover());
        let color = if enabled {
            betaflight_colors::TOGGLE_ON
        } else {
            betaflight_colors::TOGGLE_OFF
        };

        // Draw toggle background
        ui.painter().rect_filled(rect, 9.0, color);

        // Draw toggle circle
        let circle_x = if enabled {
            rect.right() - 9.0
        } else {
            rect.left() + 9.0
        };
        ui.painter()
            .circle_filled(egui::pos2(circle_x, rect.center().y), 7.0, Color32::WHITE);
    }

    fn toggle_row(ui: &mut egui::Ui, label: &str, enabled: bool) {
        ui.horizontal(|ui| {
            Self::toggle_indicator(ui, enabled);
            ui.label(label);
        });
    }

    fn value_field(ui: &mut egui::Ui, value: &str) {
        let mut val = value.to_string();
        ui.add(
            egui::TextEdit::singleline(&mut val)
                .desired_width(60.0)
                .interactive(false),
        );
    }

    fn setting_row(ui: &mut egui::Ui, label: &str, value: Option<&String>) {
        ui.label(RichText::new(label).color(betaflight_colors::TEXT_LABEL));
        let val = value.map(|s| s.as_str()).unwrap_or("--");
        Self::value_field(ui, val);
        ui.end_row();
    }

    fn setting_row_single(ui: &mut egui::Ui, label: &str, value: Option<&String>) {
        ui.horizontal(|ui| {
            ui.label(RichText::new(label).color(betaflight_colors::TEXT_LABEL));
            let val = value.map(|s| s.as_str()).unwrap_or("--");
            Self::value_field(ui, val);
        });
    }
}

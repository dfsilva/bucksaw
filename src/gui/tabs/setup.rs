use std::sync::Arc;

use egui::{Color32, RichText};

use crate::flight_data::FlightData;
use crate::gui::colors::Colors;

/// Setup Information Tab - displays flight controller configuration
pub struct SetupTab {
    fd: Arc<FlightData>,
}

impl SetupTab {
    pub fn new(fd: Arc<FlightData>) -> Self {
        Self { fd }
    }

    pub fn show(&mut self, ui: &mut egui::Ui) {
        let colors = Colors::get(ui);

        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("Flight Controller Setup Information");
            ui.add_space(8.0);

            // Basic Info Section
            Self::section_header(ui, "📋 Basic Information", colors.triple_primary[0]);
            egui::Grid::new("basic_info_grid")
                .num_columns(2)
                .spacing([20.0, 4.0])
                .striped(true)
                .show(ui, |ui| {
                    Self::info_row(ui, "Firmware", &format!("{:?}", self.fd.firmware));
                    if let Some(date) = &self.fd.firmware_date {
                        Self::info_row(ui, "Firmware Date", date);
                    }
                    if let Some(board) = &self.fd.board_info {
                        Self::info_row(ui, "Board", board);
                    }
                    if let Some(name) = &self.fd.craft_name {
                        Self::info_row(ui, "Craft Name", name);
                    }
                    Self::info_row(ui, "Debug Mode", &format!("{:?}", self.fd.debug_mode));
                    Self::info_row(ui, "ESC Protocol", &format!("{:?}", self.fd.esc_protocol));
                    Self::info_row(
                        ui,
                        "Sample Rate",
                        &format!("{:.0} Hz", self.fd.sample_rate()),
                    );
                    Self::info_row(
                        ui,
                        "Log Duration",
                        &format!(
                            "{:.1} s",
                            self.fd.times.last().unwrap_or(&0.0)
                                - self.fd.times.first().unwrap_or(&0.0)
                        ),
                    );
                    Self::info_row(ui, "Total Samples", &format!("{}", self.fd.times.len()));
                });

            ui.add_space(12.0);

            // Features Section
            if !self.fd.features.is_empty() {
                Self::section_header(ui, "⚙️ Enabled Features", colors.triple_primary[1]);
                ui.horizontal_wrapped(|ui| {
                    for feature in &self.fd.features {
                        ui.label(
                            RichText::new(format!("• {}", feature)).color(colors.triple_primary[1]),
                        );
                    }
                });
                ui.add_space(12.0);
            }

            // PID Settings
            Self::section_header(ui, "🎛️ PID Settings", colors.p);
            Self::show_pid_settings(ui, &self.fd.unknown_headers);
            ui.add_space(12.0);

            // Rate Settings
            Self::section_header(ui, "📐 Rate Settings", colors.setpoint);
            Self::show_rate_settings(ui, &self.fd.unknown_headers);
            ui.add_space(12.0);

            // Filter Settings
            Self::section_header(ui, "🔊 Filter Settings", colors.d);
            Self::show_filter_settings(ui, &self.fd.unknown_headers);
            ui.add_space(12.0);

            // Motor/ESC Settings
            Self::section_header(ui, "🔧 Motor/ESC Settings", colors.gyro_filtered);
            Self::show_motor_settings(ui, &self.fd.unknown_headers);
            ui.add_space(12.0);

            // All Other Settings (Collapsible)
            ui.collapsing(
                RichText::new("📄 All Raw Settings").strong().size(14.0),
                |ui| {
                    Self::show_all_settings(ui, &self.fd.unknown_headers);
                },
            );
        });
    }

    fn section_header(ui: &mut egui::Ui, title: &str, color: Color32) {
        ui.add_space(4.0);
        ui.label(RichText::new(title).strong().size(14.0).color(color));
        ui.separator();
    }

    fn info_row(ui: &mut egui::Ui, label: &str, value: &str) {
        ui.label(RichText::new(label).strong());
        ui.label(value);
        ui.end_row();
    }

    fn show_pid_settings(ui: &mut egui::Ui, headers: &std::collections::HashMap<String, String>) {
        // PID-related keys
        let pid_keys = [
            "rollPID",
            "pitchPID",
            "yawPID",
            "pid_roll",
            "pid_pitch",
            "pid_yaw",
            "d_min_roll",
            "d_min_pitch",
            "d_min_yaw",
            "d_min",
            "d_max_advance",
            "d_max_gain",
            "ff_weight",
            "feedforward_weight",
            "feedforward_transition",
            "feedforward_averaging",
            "feedforward_smooth_factor",
            "feedforward_jitter_factor",
            "feedforward_boost",
            "feedforward_max_rate_limit",
            "iterm_relax",
            "iterm_relax_type",
            "iterm_relax_cutoff",
            "iterm_limit",
            "iterm_rotation",
            "pidsum_limit",
            "pidsum_limit_yaw",
            "anti_gravity_gain",
            "anti_gravity_cutoff_hz",
            "tpa_rate",
            "tpa_breakpoint",
            "tpa_low_rate",
            "tpa_low_breakpoint",
            "thrust_linear",
            "motor_output_limit",
        ];

        Self::show_settings_grid(ui, headers, &pid_keys, "pid_settings");
    }

    fn show_rate_settings(ui: &mut egui::Ui, headers: &std::collections::HashMap<String, String>) {
        let rate_keys = [
            "rates_type",
            "roll_rc_rate",
            "pitch_rc_rate",
            "yaw_rc_rate",
            "roll_expo",
            "pitch_expo",
            "yaw_expo",
            "roll_srate",
            "pitch_srate",
            "yaw_srate",
            "rc_rate",
            "rc_expo",
            "rc_rate_yaw",
            "rc_expo_yaw",
            "roll_rate",
            "pitch_rate",
            "yaw_rate",
            "thr_mid",
            "thr_expo",
            "rc_smoothing",
            "rc_smoothing_type",
            "rc_smoothing_auto_factor",
            "rc_smoothing_feedforward_hz",
            "rc_smoothing_setpoint_hz",
            "rc_smoothing_throttle_hz",
        ];

        Self::show_settings_grid(ui, headers, &rate_keys, "rate_settings");
    }

    fn show_filter_settings(
        ui: &mut egui::Ui,
        headers: &std::collections::HashMap<String, String>,
    ) {
        let filter_keys = [
            "gyro_lpf1_static_hz",
            "gyro_lpf2_static_hz",
            "gyro_lpf1_dyn_min_hz",
            "gyro_lpf1_dyn_max_hz",
            "gyro_lpf1_type",
            "gyro_lpf2_type",
            "dterm_lpf1_static_hz",
            "dterm_lpf2_static_hz",
            "dterm_lpf1_dyn_min_hz",
            "dterm_lpf1_dyn_max_hz",
            "dterm_lpf1_type",
            "dterm_lpf2_type",
            "dterm_notch_hz",
            "dterm_notch_cutoff",
            "gyro_notch1_hz",
            "gyro_notch1_cutoff",
            "gyro_notch2_hz",
            "gyro_notch2_cutoff",
            "dyn_notch_count",
            "dyn_notch_q",
            "dyn_notch_min_hz",
            "dyn_notch_max_hz",
            "rpm_filter_harmonics",
            "rpm_filter_q",
            "rpm_filter_min_hz",
            "gyro_hardware_lpf",
            "gyro_lpf_hz",
            "dterm_lpf_hz",
            "dterm_lpf_type",
            "yaw_lpf_hz",
        ];

        Self::show_settings_grid(ui, headers, &filter_keys, "filter_settings");
    }

    fn show_motor_settings(ui: &mut egui::Ui, headers: &std::collections::HashMap<String, String>) {
        let motor_keys = [
            "motor_pwm_protocol",
            "motor_pwm_rate",
            "dshot_bidir",
            "dshot_idle_value",
            "motor_poles",
            "min_throttle",
            "max_throttle",
            "min_command",
            "motor_idle_speed",
            "digitalIdlePercent",
            "gyro_sync",
            "pid_process_denom",
            "use_unsynced_pwm",
        ];

        Self::show_settings_grid(ui, headers, &motor_keys, "motor_settings");
    }

    fn show_settings_grid(
        ui: &mut egui::Ui,
        headers: &std::collections::HashMap<String, String>,
        keys: &[&str],
        id: &str,
    ) {
        let mut found_any = false;
        egui::Grid::new(id)
            .num_columns(2)
            .spacing([20.0, 2.0])
            .striped(true)
            .show(ui, |ui| {
                for key in keys {
                    if let Some(value) = headers.get(*key) {
                        Self::info_row(ui, key, value);
                        found_any = true;
                    }
                }
            });

        if !found_any {
            ui.label(RichText::new("No settings found in this category").weak());
        }
    }

    fn show_all_settings(ui: &mut egui::Ui, headers: &std::collections::HashMap<String, String>) {
        // Sort keys alphabetically
        let mut keys: Vec<_> = headers.keys().collect();
        keys.sort();

        ui.add_space(4.0);
        ui.label(format!("Total settings: {}", keys.len()));
        ui.add_space(4.0);

        // Use monospace for raw settings
        egui::Grid::new("all_settings_grid")
            .num_columns(2)
            .spacing([20.0, 2.0])
            .striped(true)
            .show(ui, |ui| {
                for key in keys {
                    if let Some(value) = headers.get(key) {
                        ui.label(RichText::new(key).monospace().strong());
                        ui.label(RichText::new(value).monospace());
                        ui.end_row();
                    }
                }
            });
    }
}

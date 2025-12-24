use std::sync::Arc;

use egui::{Color32, RichText, Ui};

use crate::flight_data::FlightData;
use crate::step_response::{calculate_step_response, SmoothingLevel};

/// Severity level for suggestions
#[derive(Clone, Copy, PartialEq)]
pub enum Severity {
    Info,
    Warning,
    Critical,
}

impl Severity {
    fn color(&self) -> Color32 {
        match self {
            Severity::Info => Color32::from_rgb(0x83, 0xa5, 0x98),      // Green
            Severity::Warning => Color32::from_rgb(0xfa, 0xbd, 0x2f),   // Yellow
            Severity::Critical => Color32::from_rgb(0xfb, 0x49, 0x34), // Red
        }
    }

    fn icon(&self) -> &'static str {
        match self {
            Severity::Info => "ℹ️",
            Severity::Warning => "⚠️",
            Severity::Critical => "🔴",
        }
    }
}

/// A single tuning suggestion
#[derive(Clone)]
pub struct TuningSuggestion {
    pub category: String,
    pub title: String,
    pub description: String,
    pub recommendation: String,
    pub cli_command: Option<String>,
    pub severity: Severity,
}

/// Analysis results for generating suggestions
#[allow(dead_code)]
struct AnalysisResults {
    // Step response metrics per axis
    step_overshoot: [f64; 3],      // Max value above 1.0
    step_undershoot: [f64; 3],     // Min value below 1.0 in first 100ms
    step_settling_time: [f64; 3], // Time to reach 0.95-1.05 range (ms)
    step_oscillations: [bool; 3],  // Has oscillations after settling

    // Noise metrics
    gyro_noise_rms: [f32; 3],      // RMS of high-freq gyro noise
    dterm_noise_rms: [f32; 2],     // RMS of D-term (Roll, Pitch)
    
    // Motor metrics
    motor_saturation_pct: f32,     // % of time any motor at max
    motor_desync_risk: bool,       // Large motor output variance
    
    // Error metrics
    tracking_error_rms: [f32; 3],  // RMS of setpoint - gyro
}

impl AnalysisResults {
    fn compute(fd: &FlightData) -> Self {
        let sample_rate = fd.sample_rate();
        
        // Compute step response metrics
        let mut step_overshoot = [0.0; 3];
        let mut step_undershoot = [0.0; 3];
        let mut step_settling_time = [0.0; 3];
        let mut step_oscillations = [false; 3];

        if let (Some(setpoint), Some(gyro)) = (fd.setpoint(), fd.gyro_filtered()) {
            for axis in 0..3 {
                let response = calculate_step_response(
                    &fd.times,
                    setpoint[axis],
                    gyro[axis],
                    sample_rate,
                    SmoothingLevel::Off,
                );
                
                // Analyze the step response
                if !response.is_empty() {
                    // Max overshoot (value above 1.0)
                    step_overshoot[axis] = response
                        .iter()
                        .map(|(_, y)| (y - 1.0).max(0.0))
                        .fold(0.0, f64::max);
                    
                    // Undershoot in first 100ms (settling too slow)
                    let first_100ms = (sample_rate * 0.1) as usize;
                    step_undershoot[axis] = response
                        .iter()
                        .take(first_100ms)
                        .map(|(_, y)| (1.0 - y).max(0.0))
                        .fold(0.0, f64::max);
                    
                    // Settling time (time to stay within 0.95-1.05)
                    let mut settled = false;
                    for (i, (t, y)) in response.iter().enumerate() {
                        if *y >= 0.95 && *y <= 1.05 {
                            if !settled {
                                step_settling_time[axis] = *t;
                                settled = true;
                            }
                        } else if i > 50 && settled {
                            // Oscillation detected after settling
                            step_oscillations[axis] = true;
                        }
                    }
                }
            }
        }

        // Compute noise metrics (high-pass filter approximation)
        let mut gyro_noise_rms = [0.0f32; 3];
        let mut dterm_noise_rms = [0.0f32; 2];
        
        if let Some(gyro) = fd.gyro_filtered() {
            for axis in 0..3 {
                if gyro[axis].len() > 10 {
                    // Simple high-pass: difference from moving average
                    let noise: Vec<f32> = gyro[axis]
                        .windows(5)
                        .map(|w| {
                            let avg: f32 = w.iter().sum::<f32>() / 5.0;
                            (w[2] - avg).abs()
                        })
                        .collect();
                    
                    let sum_sq: f32 = noise.iter().map(|x| x * x).sum();
                    gyro_noise_rms[axis] = (sum_sq / noise.len() as f32).sqrt();
                }
            }
        }

        let d_terms = fd.d();
        for axis in 0..2 {
            if let Some(d) = d_terms[axis] {
                if d.len() > 10 {
                    let noise: Vec<f32> = d
                        .windows(5)
                        .map(|w| {
                            let avg: f32 = w.iter().sum::<f32>() / 5.0;
                            (w[2] - avg).abs()
                        })
                        .collect();
                    
                    let sum_sq: f32 = noise.iter().map(|x| x * x).sum();
                    dterm_noise_rms[axis] = (sum_sq / noise.len() as f32).sqrt();
                }
            }
        }

        // Motor saturation analysis
        let mut motor_saturation_pct = 0.0f32;
        let mut motor_desync_risk = false;
        
        if let Some(motors) = fd.motor() {
            if motors.len() >= 4 {
                let total_samples = motors[0].len();
                let mut saturated_count = 0;
                
                for i in 0..total_samples {
                    let motor_vals: Vec<f32> = motors.iter().filter_map(|m| m.get(i).copied()).collect();
                    
                    // Check if any motor at max (typically ~2000 for DShot)
                    if motor_vals.iter().any(|&v| v > 1950.0) {
                        saturated_count += 1;
                    }
                    
                    // Check for large variance (desync risk)
                    if motor_vals.len() == 4 {
                        let max = motor_vals.iter().cloned().fold(0.0f32, f32::max);
                        let min = motor_vals.iter().cloned().fold(f32::MAX, f32::min);
                        if max - min > 500.0 {
                            motor_desync_risk = true;
                        }
                    }
                }
                
                motor_saturation_pct = saturated_count as f32 / total_samples as f32 * 100.0;
            }
        }

        // Tracking error
        let mut tracking_error_rms = [0.0f32; 3];
        if let (Some(setpoint), Some(gyro)) = (fd.setpoint(), fd.gyro_filtered()) {
            for axis in 0..3 {
                let len = setpoint[axis].len().min(gyro[axis].len());
                if len > 0 {
                    let sum_sq: f32 = (0..len)
                        .map(|i| {
                            let err = setpoint[axis][i] - gyro[axis][i];
                            err * err
                        })
                        .sum();
                    tracking_error_rms[axis] = (sum_sq / len as f32).sqrt();
                }
            }
        }

        Self {
            step_overshoot,
            step_undershoot,
            step_settling_time,
            step_oscillations,
            gyro_noise_rms,
            dterm_noise_rms,
            motor_saturation_pct,
            motor_desync_risk,
            tracking_error_rms,
        }
    }
}

pub struct SuggestionsTab {
    #[allow(dead_code)]
    fd: Arc<FlightData>,
    suggestions: Vec<TuningSuggestion>,
}

impl SuggestionsTab {
    pub fn new(fd: Arc<FlightData>) -> Self {
        let analysis = AnalysisResults::compute(&fd);
        let suggestions = Self::generate_suggestions(&analysis, &fd);
        
        Self { fd, suggestions }
    }

    fn generate_suggestions(analysis: &AnalysisResults, fd: &FlightData) -> Vec<TuningSuggestion> {
        let mut suggestions = Vec::new();
        let axis_names = ["Roll", "Pitch", "Yaw"];
        let cli_axis_names = ["roll", "pitch", "yaw"];
        
        // Helper to get header values
        let headers = &fd.unknown_headers;
        let get_val = |keys: &[&str]| -> Option<f32> {
            for key in keys {
                if let Some(val) = headers.get(*key).and_then(|v| v.parse::<f32>().ok()) {
                    return Some(val);
                }
            }
            None
        };

        // Parse current PIDs
        let mut p_gains = [0.0; 3];
        let mut i_gains = [0.0; 3];
        let mut d_gains = [0.0; 3];
        let mut f_gains = [0.0; 3];
        
        // Try to find PID values (BF 4.x style arrays or individual fields)
        // Note: This is a best-effort parsing. Real implementation might need more robust header parsing.
        for i in 0..3 {
            // P gains
            p_gains[i] = get_val(&[&format!("{}_p", cli_axis_names[i]), &format!("pid_{}_p", i)])
                .unwrap_or(0.0);
            // I gains
            i_gains[i] = get_val(&[&format!("{}_i", cli_axis_names[i]), &format!("pid_{}_i", i)])
                .unwrap_or(0.0);
            // D gains
            d_gains[i] = get_val(&[&format!("{}_d", cli_axis_names[i]), &format!("pid_{}_d", i)])
                .unwrap_or(0.0);
            // F gains
            f_gains[i] = get_val(&[&format!("{}_f", cli_axis_names[i]), &format!("pid_{}_f", i)])
                .unwrap_or(0.0);
        }

        // === P-GAIN SUGGESTIONS ===
        for (axis, name) in axis_names.iter().enumerate() {
            let overshoot = analysis.step_overshoot[axis];
            let undershoot = analysis.step_undershoot[axis];
            let current_p = p_gains[axis];
            
            if overshoot > 0.15 {
                let suggested_p = if current_p > 0.0 {
                    let reduction = if overshoot > 0.25 { 0.85 } else { 0.90 }; // 10-15% reduction
                    Some((current_p * reduction).round() as i32)
                } else {
                    None
                };

                let cmd = suggested_p.map(|p| format!("set {}_p = {}", cli_axis_names[axis], p));

                suggestions.push(TuningSuggestion {
                    category: "P Gain".to_string(),
                    title: format!("{} P-gain may be too high", name),
                    description: format!(
                        "Step response shows {:.0}% overshoot on {} axis.",
                        overshoot * 100.0, name
                    ),
                    recommendation: format!(
                        "Consider reducing {} P-gain by 10-15% to reduce overshoot.",
                        name
                    ),
                    cli_command: cmd,
                    severity: if overshoot > 0.25 { Severity::Warning } else { Severity::Info },
                });
            }

            if undershoot > 0.3 && overshoot < 0.05 {
                let suggested_p = if current_p > 0.0 {
                    let increase = 1.10; // 10% increase
                    Some((current_p * increase).round() as i32)
                } else {
                    None
                };

                let cmd = suggested_p.map(|p| format!("set {}_p = {}", cli_axis_names[axis], p));

                suggestions.push(TuningSuggestion {
                    category: "P Gain".to_string(),
                    title: format!("{} P-gain may be too low", name),
                    description: format!(
                        "Step response is sluggish on {} axis (slow to reach target).",
                        name
                    ),
                    recommendation: format!(
                        "Consider increasing {} P-gain by ~10% for snappier response.",
                        name
                    ),
                    cli_command: cmd,
                    severity: Severity::Info,
                });
            }
        }

        // === D-GAIN SUGGESTIONS ===  
        for (axis, name) in axis_names.iter().take(2).enumerate() {
            let current_d = d_gains[axis];

            if analysis.step_oscillations[axis] {
                let suggested_d = if current_d > 0.0 {
                    let increase = 1.15; // 15% increase
                    Some((current_d * increase).round() as i32)
                } else {
                    None
                };
                
                let cmd = suggested_d.map(|d| format!("set {}_d = {}", cli_axis_names[axis], d));

                suggestions.push(TuningSuggestion {
                    category: "D Gain".to_string(),
                    title: format!("{} shows oscillation after settling", name),
                    description: format!(
                        "Step response has oscillations on {} axis after initial settling.",
                        name
                    ),
                    recommendation: format!(
                        "Increase {} D-gain by ~15% to dampen oscillations.",
                        name
                    ),
                    cli_command: cmd,
                    severity: Severity::Warning,
                });
            }

            if analysis.dterm_noise_rms[axis] > 50.0 {
                let suggested_d = if current_d > 0.0 {
                    let reduction = 0.90; // 10% reduction
                    Some((current_d * reduction).round() as i32)
                } else {
                    None
                };

                let cmd = suggested_d.map(|d| format!("set {}_d = {}", cli_axis_names[axis], d));
                // Also suggest filter changes, but separate suggestion or combined?
                // For now, D-gain reduction is the primary quick fix.
                
                suggestions.push(TuningSuggestion {
                    category: "D Gain / Filters".to_string(),
                    title: format!("{} D-term is noisy", name),
                    description: format!(
                        "D-term noise RMS: {:.1} on {} axis. Hot motors possible.",
                        analysis.dterm_noise_rms[axis], name
                    ),
                    recommendation: "Lower D-term lowpass filter cutoff, or reduce D-gain. Consider enabling D-term notch filter.".to_string(),
                    cli_command: cmd, // Suggest lowering D gain
                    severity: if analysis.dterm_noise_rms[axis] > 100.0 { 
                        Severity::Critical 
                    } else { 
                        Severity::Warning 
                    },
                });
            }
        }

        // === FILTER SUGGESTIONS ===
        let max_gyro_noise = analysis.gyro_noise_rms.iter().cloned().fold(0.0f32, f32::max);
        if max_gyro_noise > 30.0 {
            // Suggest enabling dynamic lowpass if not already (hard to check simply, assuming safe suggestions)
            // Or suggest lowering static LPF
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "High gyro noise detected".to_string(),
                description: format!(
                    "Gyro noise RMS: R={:.1}, P={:.1}, Y={:.1}",
                    analysis.gyro_noise_rms[0],
                    analysis.gyro_noise_rms[1],
                    analysis.gyro_noise_rms[2]
                ),
                recommendation: "Consider lowering 'gyro_lowpass_hz' or enabling dynamic filtering.".to_string(),
                cli_command: None, // Too risky to suggest blind filter changes without more context
                severity: if max_gyro_noise > 50.0 { Severity::Warning } else { Severity::Info },
            });
        }

        // === MOTOR SUGGESTIONS ===
        if analysis.motor_saturation_pct > 5.0 {
            suggestions.push(TuningSuggestion {
                category: "Motors".to_string(),
                title: "Motor saturation detected".to_string(),
                description: format!(
                    "Motors at max output {:.1}% of flight time.",
                    analysis.motor_saturation_pct
                ),
                recommendation: "Reduce overall PID gains to prevent saturation (Master Multiplier < 1.0).".to_string(),
                cli_command: None,
                severity: if analysis.motor_saturation_pct > 15.0 { 
                    Severity::Critical 
                } else { 
                    Severity::Warning 
                },
            });
        }

        if analysis.motor_desync_risk {
            suggestions.push(TuningSuggestion {
                category: "Motors".to_string(),
                title: "Large motor output variance detected".to_string(),
                description: "Motors have large differences in output, possible desync risk or unbalanced quad.".to_string(),
                recommendation: "Check motor/ESC health, prop balance, and COG position.".to_string(),
                cli_command: None,
                severity: Severity::Warning,
            });
        }

        // === TRACKING ERROR SUGGESTIONS ===
        let max_error = analysis.tracking_error_rms.iter().cloned().fold(0.0f32, f32::max);
        if max_error > 50.0 {
            // Find axis with worst error
            let mut worst_axis = 0;
            let mut worst_val = 0.0;
            for i in 0..3 {
                if analysis.tracking_error_rms[i] > worst_val {
                    worst_val = analysis.tracking_error_rms[i];
                    worst_axis = i;
                }
            }
            
            let current_i = i_gains[worst_axis];
            let cmd = if current_i > 0.0 {
                Some(format!("set {}_i = {}", cli_axis_names[worst_axis], (current_i * 1.15).round() as i32))
            } else {
                None
            };
            
            suggestions.push(TuningSuggestion {
                category: "Tracking".to_string(),
                title: "High tracking error".to_string(),
                description: format!(
                    "Average tracking error: R={:.1}, P={:.1}, Y={:.1} deg/s",
                    analysis.tracking_error_rms[0],
                    analysis.tracking_error_rms[1],
                    analysis.tracking_error_rms[2]
                ),
                recommendation: format!("Increase I-gain on {} axis to improve tracking.", axis_names[worst_axis]),
                cli_command: cmd,
                severity: if max_error > 100.0 { Severity::Warning } else { Severity::Info },
            });
        }

        // === GENERAL TIPS (if no major issues) ===
        if suggestions.is_empty() {
            suggestions.push(TuningSuggestion {
                category: "Overall".to_string(),
                title: "No major issues detected".to_string(),
                description: "Flight data looks reasonably well-tuned.".to_string(),
                recommendation: "Fine-tune by adjusting feel: more P = snappier, more D = more locked-in, more I = better holding.".to_string(),
                cli_command: None,
                severity: Severity::Info,
            });
        }

        // Add current settings info
        if let Some(settings) = Self::get_current_settings(fd) {
            suggestions.insert(0, TuningSuggestion {
                category: "Current Settings".to_string(),
                title: "Detected PID values".to_string(),
                description: settings,
                recommendation: "These are the PID values extracted from the log header.".to_string(),
                cli_command: None,
                severity: Severity::Info,
            });
        }

        suggestions
    }

    fn get_current_settings(fd: &FlightData) -> Option<String> {
        let headers = &fd.unknown_headers;
        
        let roll_p = headers.get("rollPID").or_else(|| headers.get("pid[0][0]"));
        let pitch_p = headers.get("pitchPID").or_else(|| headers.get("pid[1][0]"));
        let yaw_p = headers.get("yawPID").or_else(|| headers.get("pid[2][0]"));
        
        if roll_p.is_some() || pitch_p.is_some() {
            Some(format!(
                "Roll: {}, Pitch: {}, Yaw: {}",
                roll_p.unwrap_or(&"N/A".to_string()),
                pitch_p.unwrap_or(&"N/A".to_string()),
                yaw_p.unwrap_or(&"N/A".to_string())
            ))
        } else {
            None
        }
    }

    pub fn show(&mut self, ui: &mut Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("🔧 PID & Filter Tuning Suggestions");
            ui.add_space(8.0);
            ui.label("Based on analysis of your flight log, here are tuning recommendations:");
            ui.add_space(16.0);

            // Group by category
            let mut current_category = String::new();
            
            for suggestion in &self.suggestions {
                // Category header
                if suggestion.category != current_category {
                    if !current_category.is_empty() {
                        ui.add_space(12.0);
                    }
                    ui.label(RichText::new(&suggestion.category).strong().size(16.0));
                    current_category = suggestion.category.clone();
                    ui.add_space(4.0);
                }

                // Suggestion card
                egui::Frame::none()
                    .fill(ui.style().visuals.extreme_bg_color)
                    .rounding(8.0)
                    .inner_margin(12.0)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(suggestion.severity.icon()).size(18.0));
                            ui.label(
                                RichText::new(&suggestion.title)
                                    .strong()
                                    .color(suggestion.severity.color()),
                            );
                        });

                        ui.add_space(4.0);
                        ui.label(&suggestion.description);
                        
                        ui.add_space(4.0);
                        ui.horizontal(|ui| {
                            ui.label(RichText::new("💡 ").size(14.0));
                            ui.label(RichText::new(&suggestion.recommendation).italics());
                        });

                        if let Some(cmd) = &suggestion.cli_command {
                            ui.add_space(8.0);
                            ui.scope(|ui| {
                                ui.style_mut().visuals.extreme_bg_color = egui::Color32::from_black_alpha(100);
                                egui::Frame::group(ui.style())
                                    .rounding(4.0)
                                    .inner_margin(8.0)
                                    .stroke(egui::Stroke::new(1.0, egui::Color32::from_gray(60)))
                                    .show(ui, |ui| {
                                        ui.horizontal(|ui| {
                                            ui.monospace(RichText::new(cmd).color(egui::Color32::LIGHT_GREEN));
                                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                                if ui.button("📋 Copy").on_hover_text("Copy to clipboard").clicked() {
                                                    ui.output_mut(|o| o.copied_text = cmd.clone());
                                                }
                                            });
                                        });
                                    });
                            });
                        }
                    });

                ui.add_space(8.0);
            }

            ui.add_space(16.0);
            ui.separator();
            ui.add_space(8.0);
            
            // Disclaimer
            ui.label(
                RichText::new("⚠️ Disclaimer: These are automated suggestions based on log analysis. \
                               Always test changes carefully and make small adjustments.")
                    .small()
                    .weak(),
            );
        });
    }
}

use std::sync::Arc;
use std::sync::mpsc::Receiver;

use egui::{Color32, RichText, Ui};

use crate::ai_integration::{AIAnalysisResult, AIModel, FlightMetrics, ModelFetchResult, OpenRouterClient, DEFAULT_MODELS};
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
    
    // Throttle-segmented noise (low/mid/high)
    gyro_noise_low_throttle: f32,   // <30% throttle
    gyro_noise_mid_throttle: f32,   // 30-70% throttle
    gyro_noise_high_throttle: f32,  // >70% throttle
    
    // Motor metrics
    motor_saturation_pct: f32,     // % of time any motor at max
    motor_desync_risk: bool,       // Large motor output variance
    
    // Error metrics
    tracking_error_rms: [f32; 3],  // RMS of setpoint - gyro
    
    // Flight characteristics
    avg_throttle_pct: f32,         // Average throttle percentage
    throttle_variance: f32,        // How much throttle varies
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

        // Throttle-segmented noise analysis and flight characteristics
        let mut gyro_noise_low_throttle = 0.0f32;
        let mut gyro_noise_mid_throttle = 0.0f32;
        let mut gyro_noise_high_throttle = 0.0f32;
        let mut avg_throttle_pct = 0.0f32;
        let mut throttle_variance = 0.0f32;
        
        if let (Some(motors), Some(gyro)) = (fd.motor(), fd.gyro_filtered()) {
            if motors.len() >= 4 && !motors[0].is_empty() {
                let total_samples = motors[0].len().min(gyro[0].len());
                
                // Calculate throttle values (average of 4 motors, normalized to %)
                let throttle_values: Vec<f32> = (0..total_samples)
                    .map(|i| {
                        let sum: f32 = motors.iter().filter_map(|m| m.get(i)).sum();
                        sum / 4.0 / 20.0  // Normalize to rough percentage (2000 = 100%)
                    })
                    .collect();
                
                if !throttle_values.is_empty() {
                    avg_throttle_pct = throttle_values.iter().sum::<f32>() / throttle_values.len() as f32;
                    let variance: f32 = throttle_values.iter().map(|t| (t - avg_throttle_pct).powi(2)).sum::<f32>() / throttle_values.len() as f32;
                    throttle_variance = variance.sqrt();
                }
                
                // Segment gyro noise by throttle band
                let mut low_samples = Vec::new();
                let mut mid_samples = Vec::new();
                let mut high_samples = Vec::new();
                
                for i in 0..total_samples.saturating_sub(5) {
                    let throttle = throttle_values.get(i).copied().unwrap_or(0.0);
                    // Gyro noise: difference from local mean
                    let gyro_slice = &gyro[0][i..i+5.min(gyro[0].len()-i)];
                    if gyro_slice.len() >= 3 {
                        let avg: f32 = gyro_slice.iter().sum::<f32>() / gyro_slice.len() as f32;
                        let noise = (gyro_slice[gyro_slice.len()/2] - avg).abs();
                        
                        if throttle < 30.0 {
                            low_samples.push(noise);
                        } else if throttle < 70.0 {
                            mid_samples.push(noise);
                        } else {
                            high_samples.push(noise);
                        }
                    }
                }
                
                // RMS for each band
                if !low_samples.is_empty() {
                    let sum_sq: f32 = low_samples.iter().map(|x| x * x).sum();
                    gyro_noise_low_throttle = (sum_sq / low_samples.len() as f32).sqrt();
                }
                if !mid_samples.is_empty() {
                    let sum_sq: f32 = mid_samples.iter().map(|x| x * x).sum();
                    gyro_noise_mid_throttle = (sum_sq / mid_samples.len() as f32).sqrt();
                }
                if !high_samples.is_empty() {
                    let sum_sq: f32 = high_samples.iter().map(|x| x * x).sum();
                    gyro_noise_high_throttle = (sum_sq / high_samples.len() as f32).sqrt();
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
            gyro_noise_low_throttle,
            gyro_noise_mid_throttle,
            gyro_noise_high_throttle,
            motor_saturation_pct,
            motor_desync_risk,
            tracking_error_rms,
            avg_throttle_pct,
            throttle_variance,
        }
    }
}

pub struct SuggestionsTab {
    fd: Arc<FlightData>,
    suggestions: Vec<TuningSuggestion>,
    analysis: AnalysisResults,
    
    // AI integration state
    api_key: String,
    selected_model_id: Option<String>,
    model_filter: String,
    ai_settings_open: bool,
    ai_loading: bool,
    ai_response: Option<String>,
    ai_error: Option<String>,
    ai_receiver: Option<Receiver<AIAnalysisResult>>,
    
    // Model list state
    models: Vec<AIModel>,
    models_loading: bool,
    models_receiver: Option<Receiver<ModelFetchResult>>,
}

impl SuggestionsTab {
    pub fn new(fd: Arc<FlightData>) -> Self {
        let analysis = AnalysisResults::compute(&fd);
        let suggestions = Self::generate_suggestions(&analysis, &fd);
        
        // Load saved settings
        let saved_settings = crate::settings::AppSettings::load();
        
        // Initialize with default models
        let default_models: Vec<AIModel> = DEFAULT_MODELS
            .iter()
            .map(|(id, name)| AIModel {
                id: id.to_string(),
                name: name.to_string(),
            })
            .collect();
        
        // Start fetching models in background
        let models_receiver = Some(OpenRouterClient::fetch_models_async());
        
        // Use saved settings if available, otherwise defaults
        let api_key = saved_settings.ai.api_key;
        let selected_model_id = saved_settings.ai.selected_model_id
            .or_else(|| Some("anthropic/claude-3.5-sonnet".to_string()));
        
        Self { 
            fd, 
            suggestions,
            analysis,
            api_key,
            selected_model_id,
            model_filter: String::new(),
            ai_settings_open: false,
            ai_loading: false,
            ai_response: None,
            ai_error: None,
            ai_receiver: None,
            models: default_models,
            models_loading: true,
            models_receiver,
        }
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
        let mut p_gains = [0.0f32; 3];
        let mut i_gains = [0.0f32; 3];
        let mut d_gains = [0.0f32; 3];  // D Max
        let mut d_min = [0.0f32; 3];    // D Min (actual derivative on low stick)
        let mut f_gains = [0.0f32; 3];  // Feedforward
        
        // Try to find PID values (BF 4.x style arrays or individual fields)
        for i in 0..3 {
            // Helper to parse comma-separated "P,I,D" strings like "45,80,35"
            let parse_compound_pid = |key: &str, index: usize| -> Option<f32> {
                headers.get(key)
                    .and_then(|s| s.split(',').nth(index))
                    .and_then(|v| v.trim().parse::<f32>().ok())
            };

            let axis_key = match i { 0 => "rollPID", 1 => "pitchPID", _ => "yawPID" };

            // P gains (try: roll_p, pid_0_p, pid[0][0], or 1st element of rollPID)
            p_gains[i] = get_val(&[&format!("{}_p", cli_axis_names[i]), &format!("pid_{}_p", i), &format!("pid[{}][0]", i)])
                .or_else(|| parse_compound_pid(axis_key, 0))
                .unwrap_or(0.0);
                
            // I gains (try: roll_i, pid_0_i, pid[0][1], or 2nd element of rollPID)
            i_gains[i] = get_val(&[&format!("{}_i", cli_axis_names[i]), &format!("pid_{}_i", i), &format!("pid[{}][1]", i)])
                .or_else(|| parse_compound_pid(axis_key, 1))
                .unwrap_or(0.0);
                
            // D gains / D Max (try: D column in rollPID is usually D Max in BF4+)
            d_gains[i] = get_val(&[&format!("{}_d", cli_axis_names[i]), &format!("pid_{}_d", i), &format!("pid[{}][2]", i)])
                .or_else(|| parse_compound_pid(axis_key, 2))
                .unwrap_or(0.0);
            
            // D Min (separate headers in BF 4.3+)
            d_min[i] = get_val(&[
                &format!("d_min_{}", cli_axis_names[i]),  // d_min_roll, d_min_pitch, d_min_yaw
                &format!("dmin_{}", cli_axis_names[i]),
            ]).unwrap_or(0.0);
                
            // Feedforward (try various header names)
            f_gains[i] = get_val(&[
                &format!("{}_f", cli_axis_names[i]),      // roll_f
                &format!("pid_{}_f", i),                   // pid_0_f
                &format!("pid[{}][3]", i),                 // pid[0][3]
                &format!("f_{}", cli_axis_names[i]),      // f_roll
                &format!("feedforward_{}", cli_axis_names[i]), // feedforward_roll
            ])
            .or_else(|| parse_compound_pid(axis_key, 3))  // 4th element of rollPID if exists
            .unwrap_or(0.0);
        }
        
        // Parse filter settings - Comprehensive filter analysis
        // Gyro Lowpass Filters
        let gyro_lpf1_hz = get_val(&["gyro_lpf1_static_hz", "gyro_lpf_hz", "gyro_lowpass_hz"]).unwrap_or(0.0);
        let gyro_lpf2_hz = get_val(&["gyro_lpf2_static_hz", "gyro_lowpass2_hz"]).unwrap_or(0.0);
        let gyro_lpf1_dyn_min = get_val(&["gyro_lpf1_dyn_min_hz"]).unwrap_or(0.0);
        let gyro_lpf1_dyn_max = get_val(&["gyro_lpf1_dyn_max_hz"]).unwrap_or(0.0);
        
        // D-Term Lowpass Filters
        let dterm_lpf1_hz = get_val(&["dterm_lpf1_static_hz", "dterm_lpf_hz", "dterm_lowpass_hz"]).unwrap_or(0.0);
        let dterm_lpf2_hz = get_val(&["dterm_lpf2_static_hz", "dterm_lowpass2_hz"]).unwrap_or(0.0);
        let dterm_lpf1_dyn_min = get_val(&["dterm_lpf1_dyn_min_hz"]).unwrap_or(0.0);
        let dterm_lpf1_dyn_max = get_val(&["dterm_lpf1_dyn_max_hz"]).unwrap_or(0.0);
        
        // Gyro Notch Filters
        let gyro_notch1_hz = get_val(&["gyro_notch1_hz"]).unwrap_or(0.0);
        let gyro_notch1_cutoff = get_val(&["gyro_notch1_cutoff"]).unwrap_or(0.0);
        let gyro_notch2_hz = get_val(&["gyro_notch2_hz"]).unwrap_or(0.0);
        let gyro_notch2_cutoff = get_val(&["gyro_notch2_cutoff"]).unwrap_or(0.0);
        
        // D-Term Notch Filter
        let dterm_notch_hz = get_val(&["dterm_notch_hz"]).unwrap_or(0.0);
        let dterm_notch_cutoff = get_val(&["dterm_notch_cutoff"]).unwrap_or(0.0);
        
        // Yaw Lowpass Filter
        let yaw_lpf_hz = get_val(&["yaw_lpf_hz", "yaw_lowpass_hz"]).unwrap_or(0.0);
        
        // Dynamic Notch Filter
        let dyn_notch_count = get_val(&["dyn_notch_count"]).unwrap_or(0.0);
        let dyn_notch_q = get_val(&["dyn_notch_q"]).unwrap_or(0.0);
        let dyn_notch_min = get_val(&["dyn_notch_min_hz"]).unwrap_or(0.0);
        let dyn_notch_max = get_val(&["dyn_notch_max_hz"]).unwrap_or(0.0);
        
        // RPM Filter
        let rpm_filter_harmonics = get_val(&["rpm_filter_harmonics"]).unwrap_or(0.0);
        let rpm_filter_q = get_val(&["rpm_filter_q"]).unwrap_or(0.0);
        let rpm_filter_min_hz = get_val(&["rpm_filter_min_hz"]).unwrap_or(0.0);
        
        // Filter multipliers
        let gyro_filter_multiplier = get_val(&["gyro_filter_multiplier", "gyro_lowpass_dyn_gain"]).unwrap_or(1.0);
        let dterm_filter_multiplier = get_val(&["dterm_filter_multiplier", "dterm_lowpass_dyn_gain"]).unwrap_or(1.0);
        
        // Parse feedforward global settings
        let _ff_transition = get_val(&["feedforward_transition"]).unwrap_or(0.0);
        let _ff_boost = get_val(&["feedforward_boost"]).unwrap_or(0.0);
        let _ff_smooth_factor = get_val(&["feedforward_smooth_factor"]).unwrap_or(0.0);
        let ff_jitter_factor = get_val(&["feedforward_jitter_factor"]).unwrap_or(0.0);

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
            // Suggest specific filter changes based on parsed values
            let lpf_cmd = if gyro_lpf1_hz > 200.0 {
                Some(format!("set gyro_lpf1_static_hz = {}", (gyro_lpf1_hz * 0.85) as i32))
            } else {
                None
            };
            
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "High gyro noise detected".to_string(),
                description: format!(
                    "Gyro noise RMS: R={:.1}, P={:.1}, Y={:.1}. {}",
                    analysis.gyro_noise_rms[0],
                    analysis.gyro_noise_rms[1],
                    analysis.gyro_noise_rms[2],
                    if gyro_lpf1_hz > 0.0 { format!("Current gyro LPF1: {}Hz", gyro_lpf1_hz as i32) } else { "".to_string() }
                ),
                recommendation: "Lower gyro_lpf1_static_hz or enable dynamic filtering with rpm_filter if supported.".to_string(),
                cli_command: lpf_cmd,
                severity: if max_gyro_noise > 50.0 { Severity::Warning } else { Severity::Info },
            });
        }
        
        // === D-TERM FILTER SUGGESTIONS ===
        let max_dterm_noise = analysis.dterm_noise_rms.iter().cloned().fold(0.0f32, f32::max);
        if max_dterm_noise > 50.0 && dterm_lpf1_hz > 100.0 {
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "D-term filter may need lowering".to_string(),
                description: format!(
                    "D-term noise is high ({:.1} RMS). Current D-term LPF1: {}Hz",
                    max_dterm_noise, dterm_lpf1_hz as i32
                ),
                recommendation: "Lower D-term lowpass filter to reduce motor heat.".to_string(),
                cli_command: Some(format!("set dterm_lpf1_static_hz = {}", (dterm_lpf1_hz * 0.80) as i32)),
                severity: Severity::Warning,
            });
        }
        
        // === GYRO LPF2 SUGGESTIONS ===
        if gyro_lpf2_hz > 0.0 && gyro_lpf2_hz < gyro_lpf1_hz {
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "Gyro LPF2 is lower than LPF1".to_string(),
                description: format!("Gyro LPF1: {}Hz, LPF2: {}Hz. LPF2 should be higher than LPF1.", gyro_lpf1_hz as i32, gyro_lpf2_hz as i32),
                recommendation: "LPF2 should be ~2x LPF1 frequency or disabled.".to_string(),
                cli_command: Some(format!("set gyro_lpf2_static_hz = {}", (gyro_lpf1_hz * 2.0) as i32)),
                severity: Severity::Info,
            });
        }
        
        // === D-TERM LPF2 SUGGESTIONS ===
        if dterm_lpf2_hz > 0.0 && dterm_lpf2_hz < dterm_lpf1_hz {
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "D-term LPF2 is lower than LPF1".to_string(),
                description: format!("D-term LPF1: {}Hz, LPF2: {}Hz. LPF2 should be higher than LPF1.", dterm_lpf1_hz as i32, dterm_lpf2_hz as i32),
                recommendation: "LPF2 should be ~2x LPF1 frequency or disabled.".to_string(),
                cli_command: Some(format!("set dterm_lpf2_static_hz = {}", (dterm_lpf1_hz * 2.0) as i32)),
                severity: Severity::Info,
            });
        }
        
        // === DYNAMIC GYRO LPF SUGGESTIONS ===
        if gyro_lpf1_dyn_min > 0.0 && gyro_lpf1_dyn_max > 0.0 {
            if gyro_lpf1_dyn_max < gyro_lpf1_dyn_min * 1.5 {
                suggestions.push(TuningSuggestion {
                    category: "Filtering".to_string(),
                    title: "Narrow dynamic gyro LPF range".to_string(),
                    description: format!("Dynamic LPF range {}Hz-{}Hz is narrow.", gyro_lpf1_dyn_min as i32, gyro_lpf1_dyn_max as i32),
                    recommendation: "Widen the range for better dynamic response (e.g., 200-500Hz).".to_string(),
                    cli_command: None,
                    severity: Severity::Info,
                });
            }
        } else if max_gyro_noise > 40.0 && gyro_lpf1_dyn_min == 0.0 {
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "Consider enabling dynamic gyro LPF".to_string(),
                description: "Dynamic lowpass adjusts cutoff based on throttle for better noise/latency tradeoff.".to_string(),
                recommendation: "Enable dynamic filtering with typical range 200-500Hz.".to_string(),
                cli_command: Some("set gyro_lpf1_dyn_min_hz = 200\nset gyro_lpf1_dyn_max_hz = 500".to_string()),
                severity: Severity::Info,
            });
        }
        
        // === DYNAMIC D-TERM LPF SUGGESTIONS ===
        if dterm_lpf1_dyn_min > 0.0 && dterm_lpf1_dyn_max > 0.0 && dterm_lpf1_dyn_max < dterm_lpf1_dyn_min * 1.5 {
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "Narrow dynamic D-term LPF range".to_string(),
                description: format!("D-term dynamic LPF range {}Hz-{}Hz is narrow.", dterm_lpf1_dyn_min as i32, dterm_lpf1_dyn_max as i32),
                recommendation: "Widen the range for better dynamic response (e.g., 70-170Hz).".to_string(),
                cli_command: None,
                severity: Severity::Info,
            });
        }
        
        // === GYRO NOTCH FILTER SUGGESTIONS ===
        if gyro_notch1_hz > 0.0 && (gyro_notch1_cutoff == 0.0 || gyro_notch1_cutoff > gyro_notch1_hz) {
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "Gyro notch 1 cutoff may be misconfigured".to_string(),
                description: format!("Notch center: {}Hz, Cutoff: {}Hz", gyro_notch1_hz as i32, gyro_notch1_cutoff as i32),
                recommendation: "Notch cutoff should be lower than center frequency.".to_string(),
                cli_command: Some(format!("set gyro_notch1_cutoff = {}", (gyro_notch1_hz * 0.7) as i32)),
                severity: Severity::Warning,
            });
        }
        
        if gyro_notch2_hz > 0.0 && (gyro_notch2_cutoff == 0.0 || gyro_notch2_cutoff > gyro_notch2_hz) {
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "Gyro notch 2 cutoff may be misconfigured".to_string(),
                description: format!("Notch center: {}Hz, Cutoff: {}Hz", gyro_notch2_hz as i32, gyro_notch2_cutoff as i32),
                recommendation: "Notch cutoff should be lower than center frequency.".to_string(),
                cli_command: Some(format!("set gyro_notch2_cutoff = {}", (gyro_notch2_hz * 0.7) as i32)),
                severity: Severity::Warning,
            });
        }
        
        // === D-TERM NOTCH SUGGESTIONS ===
        if dterm_notch_hz > 0.0 && (dterm_notch_cutoff == 0.0 || dterm_notch_cutoff > dterm_notch_hz) {
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "D-term notch cutoff may be misconfigured".to_string(),
                description: format!("D-term notch center: {}Hz, Cutoff: {}Hz", dterm_notch_hz as i32, dterm_notch_cutoff as i32),
                recommendation: "D-term notch cutoff should be lower than center frequency.".to_string(),
                cli_command: Some(format!("set dterm_notch_cutoff = {}", (dterm_notch_hz * 0.7) as i32)),
                severity: Severity::Warning,
            });
        }
        
        // === YAW LOWPASS FILTER SUGGESTIONS ===
        if yaw_lpf_hz > 0.0 {
            if yaw_lpf_hz < 30.0 {
                suggestions.push(TuningSuggestion {
                    category: "Filtering".to_string(),
                    title: "Yaw lowpass filter is very low".to_string(),
                    description: format!("Yaw LPF: {}Hz. Very low values add latency to yaw response.", yaw_lpf_hz as i32),
                    recommendation: "Consider raising yaw LPF to 50-100Hz for snappier yaw.".to_string(),
                    cli_command: Some("set yaw_lpf_hz = 50".to_string()),
                    severity: Severity::Info,
                });
            } else if yaw_lpf_hz > 150.0 {
                suggestions.push(TuningSuggestion {
                    category: "Filtering".to_string(),
                    title: "Yaw lowpass filter may be too high".to_string(),
                    description: format!("Yaw LPF: {}Hz. High values may allow noise through.", yaw_lpf_hz as i32),
                    recommendation: "Consider lowering yaw LPF to 50-100Hz if yaw feels jittery.".to_string(),
                    cli_command: Some("set yaw_lpf_hz = 100".to_string()),
                    severity: Severity::Info,
                });
            }
        }
        
        // === DYNAMIC NOTCH FILTER SUGGESTIONS ===
        if dyn_notch_count > 0.0 {
            // Dynamic notch is enabled
            if dyn_notch_min > 0.0 && dyn_notch_max > 0.0 {
                if dyn_notch_min < 70.0 {
                    suggestions.push(TuningSuggestion {
                        category: "Filtering".to_string(),
                        title: "Dynamic notch min frequency is low".to_string(),
                        description: format!("Dyn notch min: {}Hz. Values below 80Hz can cause tracking issues.", dyn_notch_min as i32),
                        recommendation: "Raise dynamic notch minimum to 80Hz or higher.".to_string(),
                        cli_command: Some("set dyn_notch_min_hz = 80".to_string()),
                        severity: Severity::Info,
                    });
                }
                if dyn_notch_max < 400.0 && max_gyro_noise > 30.0 {
                    suggestions.push(TuningSuggestion {
                        category: "Filtering".to_string(),
                        title: "Dynamic notch max may be too low".to_string(),
                        description: format!("Dyn notch max: {}Hz. Higher frequency noise may not be filtered.", dyn_notch_max as i32),
                        recommendation: "Consider raising dyn_notch_max_hz to 500-600Hz.".to_string(),
                        cli_command: Some("set dyn_notch_max_hz = 500".to_string()),
                        severity: Severity::Info,
                    });
                }
            }
            if dyn_notch_q > 0.0 && dyn_notch_q < 200.0 {
                suggestions.push(TuningSuggestion {
                    category: "Filtering".to_string(),
                    title: "Dynamic notch Q is low".to_string(),  
                    description: format!("Dyn notch Q: {:.0}. Low Q = wide notches = more filtering but more delay.", dyn_notch_q),
                    recommendation: "Raise Q to 300-500 for tighter notches with less delay.".to_string(),
                    cli_command: Some("set dyn_notch_q = 350".to_string()),
                    severity: Severity::Info,
                });
            }
        } else if max_gyro_noise > 30.0 {
            // Dynamic notch not enabled but noise is high
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "Consider enabling dynamic notch".to_string(),
                description: "Dynamic notch filter adapts to motor noise in real-time.".to_string(),
                recommendation: "Enable dynamic notch with 3-4 notches for better noise control.".to_string(),
                cli_command: Some("set dyn_notch_count = 3\nset dyn_notch_q = 350\nset dyn_notch_min_hz = 80\nset dyn_notch_max_hz = 500".to_string()),
                severity: Severity::Info,
            });
        }
        
        // === RPM FILTER SUGGESTIONS ===
        if rpm_filter_harmonics > 0.0 {
            // RPM filter is enabled
            if rpm_filter_min_hz < 80.0 && rpm_filter_min_hz > 0.0 {
                suggestions.push(TuningSuggestion {
                    category: "Filtering".to_string(),
                    title: "RPM filter min frequency is low".to_string(),
                    description: format!("RPM filter min: {}Hz. Very low values can cause issues.", rpm_filter_min_hz as i32),
                    recommendation: "Raise RPM filter min to 80-100Hz.".to_string(),
                    cli_command: Some("set rpm_filter_min_hz = 100".to_string()),
                    severity: Severity::Info,
                });
            }
            if rpm_filter_q > 0.0 && rpm_filter_q < 400.0 {
                suggestions.push(TuningSuggestion {
                    category: "Filtering".to_string(),
                    title: "RPM filter Q is low".to_string(),
                    description: format!("RPM filter Q: {:.0}. Lower Q = wider notches.", rpm_filter_q),
                    recommendation: "Consider Q of 500 for tighter motor noise targeting.".to_string(),
                    cli_command: Some("set rpm_filter_q = 500".to_string()),
                    severity: Severity::Info,
                });
            }
        } else if max_gyro_noise > 40.0 {
            // RPM filter not enabled, suggest it for high noise
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "Consider enabling RPM filter".to_string(),
                description: "RPM filter is the most effective filter for motor noise (requires bidirectional DShot).".to_string(),
                recommendation: "Enable RPM filter with 3 harmonics if you have bidirectional DShot.".to_string(),
                cli_command: Some("set dshot_bidir = ON\nset rpm_filter_harmonics = 3".to_string()),
                severity: Severity::Info,
            });
        }
        
        // === FILTER MULTIPLIER SUGGESTIONS ===
        if gyro_filter_multiplier < 0.8 {
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "Gyro filter multiplier is very low".to_string(),
                description: format!("Gyro filter multiplier: {:.2}. Values below 0.8 add significant latency.", gyro_filter_multiplier),
                recommendation: "Consider raising to 0.9-1.0 if your quad is clean.".to_string(),
                cli_command: Some("set gyro_filter_multiplier = 100".to_string()),
                severity: Severity::Info,
            });
        } else if gyro_filter_multiplier > 1.3 {
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "Gyro filter multiplier is high".to_string(),
                description: format!("Gyro filter multiplier: {:.2}. High values reduce filtering effectiveness.", gyro_filter_multiplier),
                recommendation: "Lower to 1.0-1.1 if experiencing noise or hot motors.".to_string(),
                cli_command: Some("set gyro_filter_multiplier = 100".to_string()),
                severity: Severity::Warning,
            });
        }
        
        if dterm_filter_multiplier < 0.8 {
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "D-term filter multiplier is very low".to_string(),
                description: format!("D-term filter multiplier: {:.2}. May cause mushy feel.", dterm_filter_multiplier),
                recommendation: "Consider raising to 0.85-1.0 for crisper response.".to_string(),
                cli_command: Some("set dterm_filter_multiplier = 85".to_string()),
                severity: Severity::Info,
            });
        } else if dterm_filter_multiplier > 1.2 {
            suggestions.push(TuningSuggestion {
                category: "Filtering".to_string(),
                title: "D-term filter multiplier is high".to_string(),
                description: format!("D-term filter multiplier: {:.2}. High values can cause hot motors.", dterm_filter_multiplier),
                recommendation: "Lower to 0.85-1.0 if motors run hot.".to_string(),
                cli_command: Some("set dterm_filter_multiplier = 85".to_string()),
                severity: Severity::Warning,
            });
        }
        
        // === D-MIN / D-MAX BALANCE SUGGESTIONS ===
        for (axis, name) in axis_names.iter().enumerate() {
            let d_max_val = d_gains[axis];
            let d_min_val = d_min[axis];
            
            if d_max_val > 0.0 && d_min_val > 0.0 {
                let ratio = d_min_val / d_max_val;
                
                // D-min should typically be 60-80% of D-max
                if ratio < 0.5 {
                    suggestions.push(TuningSuggestion {
                        category: "D Gain".to_string(),
                        title: format!("{} D-min is low relative to D-max", name),
                        description: format!(
                            "{}: D-min={:.0}, D-max={:.0} (ratio: {:.0}%). May cause inconsistent feel.",
                            name, d_min_val, d_max_val, ratio * 100.0
                        ),
                        recommendation: "Consider raising D-min to ~70% of D-max for smoother response.".to_string(),
                        cli_command: Some(format!("set d_min_{} = {}", cli_axis_names[axis], (d_max_val * 0.70) as i32)),
                        severity: Severity::Info,
                    });
                } else if ratio > 0.95 {
                     suggestions.push(TuningSuggestion {
                        category: "D Gain".to_string(),
                        title: format!("{} D-min nearly equals D-max", name),
                        description: format!(
                            "{}: D-min={:.0}, D-max={:.0}. D-gain is effectively static.",
                            name, d_min_val, d_max_val
                        ),
                        recommendation: "Lower D-min to allow D-gain to scale with stick movement.".to_string(),
                        cli_command: Some(format!("set d_min_{} = {}", cli_axis_names[axis], (d_max_val * 0.70) as i32)),
                        severity: Severity::Info,
                    });
                }
            }
        }
        
        // === FEEDFORWARD SUGGESTIONS ===
        let max_ff = f_gains.iter().cloned().fold(0.0f32, f32::max);
        let _min_ff = f_gains.iter().cloned().fold(f32::MAX, f32::min);
        
        // Check if FF values are present and reasonable
        if max_ff > 0.0 {
            // High FF can cause propwash issues with certain setups
            if max_ff > 200.0 {
                suggestions.push(TuningSuggestion {
                    category: "Feedforward".to_string(),
                    title: "High feedforward values detected".to_string(),
                    description: format!(
                        "FF values: R={:.0}, P={:.0}, Y={:.0}. High FF can cause propwash oscillation.",
                        f_gains[0], f_gains[1], f_gains[2]
                    ),
                    recommendation: "If experiencing propwash, try reducing FF by 15-20%.".to_string(),
                    cli_command: Some(format!("set feedforward_weight = {}", (max_ff * 0.85) as i32)),
                    severity: Severity::Info,
                });
            }
            
            // Check for FF imbalance (roll vs pitch should be similar)
            if f_gains[0] > 0.0 && f_gains[1] > 0.0 {
                let ff_ratio = f_gains[0] / f_gains[1];
                if !(0.7..=1.4).contains(&ff_ratio) {
                    suggestions.push(TuningSuggestion {
                        category: "Feedforward".to_string(),
                        title: "Roll/Pitch feedforward imbalance".to_string(),
                        description: format!(
                            "Roll FF={:.0}, Pitch FF={:.0}. Large difference may feel inconsistent.",
                            f_gains[0], f_gains[1]
                        ),
                        recommendation: "Consider matching Roll and Pitch FF values.".to_string(),
                        cli_command: None,
                        severity: Severity::Info,
                    });
                }
            }
            
            // Low Yaw FF might cause sluggish yaw response
            if f_gains[2] < 50.0 && f_gains[2] > 0.0 && (f_gains[0] > 100.0 || f_gains[1] > 100.0) {
                suggestions.push(TuningSuggestion {
                    category: "Feedforward".to_string(),
                    title: "Yaw feedforward is relatively low".to_string(),
                    description: format!(
                        "Yaw FF={:.0} while Roll/Pitch are {:.0}/{:.0}. Yaw may feel sluggish.",
                        f_gains[2], f_gains[0], f_gains[1]
                    ),
                    recommendation: "Increase Yaw FF for snappier yaw response.".to_string(),
                    cli_command: Some(format!("set yaw_f = {}", ((f_gains[0] + f_gains[1]) / 2.0 * 0.7) as i32)),
                    severity: Severity::Info,
                });
            }
        } else {
            // No FF detected - check if it should be enabled
            suggestions.push(TuningSuggestion {
                category: "Feedforward".to_string(),
                title: "No feedforward values detected".to_string(),
                description: "Feedforward is not configured or log doesn't include FF data.".to_string(),
                recommendation: "Modern Betaflight benefits from feedforward. Consider enabling it for sharper stick response.".to_string(),
                cli_command: Some("set feedforward_weight = 100".to_string()),
                severity: Severity::Info,
            });
        }
        
        // === FF SMOOTHING SUGGESTIONS ===
        if ff_jitter_factor > 0.0 && ff_jitter_factor < 5.0 {
            suggestions.push(TuningSuggestion {
                category: "Feedforward".to_string(),
                title: "Low feedforward jitter factor".to_string(),
                description: format!("FF jitter factor is {:.0}. Very low values may cause jitter on stick movement.", ff_jitter_factor),
                recommendation: "Increase jitter factor to 7-10 for smoother response.".to_string(),
                cli_command: Some("set feedforward_jitter_factor = 7".to_string()),
                severity: Severity::Info,
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

        // === P:D RATIO ANALYSIS ===
        for (axis, name) in axis_names.iter().take(2).enumerate() {
            let p = p_gains[axis];
            let d = d_gains[axis];
            if p > 0.0 && d > 0.0 {
                let pd_ratio = d / p;
                // Good range is typically 0.5-0.8 (D is 50-80% of P)
                if pd_ratio < 0.4 {
                    suggestions.push(TuningSuggestion {
                        category: "P:D Ratio".to_string(),
                        title: format!("{} D is very low relative to P", name),
                        description: format!("{}: P={:.0}, D={:.0} (D/P ratio: {:.2}). May oscillate.", name, p, d, pd_ratio),
                        recommendation: "Consider raising D to ~60-70% of P value for better stability.".to_string(),
                        cli_command: Some(format!("set {}_d = {}", cli_axis_names[axis], (p * 0.65) as i32)),
                        severity: Severity::Warning,
                    });
                } else if pd_ratio > 1.0 {
                    suggestions.push(TuningSuggestion {
                        category: "P:D Ratio".to_string(),
                        title: format!("{} D is higher than P", name),
                        description: format!("{}: P={:.0}, D={:.0} (D/P ratio: {:.2}). Unusual configuration.", name, p, d, pd_ratio),
                        recommendation: "D higher than P is unusual. Consider increasing P or reducing D.".to_string(),
                        cli_command: None,
                        severity: Severity::Warning,
                    });
                }
            }
        }

        // === I-TERM ANALYSIS ===
        // Check for I-term configuration issues
        for (axis, name) in axis_names.iter().enumerate() {
            let i = i_gains[axis];
            let p = p_gains[axis];
            if p > 0.0 && i > 0.0 {
                // I should typically be 1.5-2.5x P for good tracking
                let ip_ratio = i / p;
                if ip_ratio < 1.0 {
                    suggestions.push(TuningSuggestion {
                        category: "I Gain".to_string(),
                        title: format!("{} I-gain is low relative to P", name),
                        description: format!("{}: P={:.0}, I={:.0} (I/P: {:.2}). May drift on windy days.", name, p, i, ip_ratio),
                        recommendation: "Consider I at 1.5-2x P value for better wind resistance.".to_string(),
                        cli_command: Some(format!("set {}_i = {}", cli_axis_names[axis], (p * 1.8) as i32)),
                        severity: Severity::Info,
                    });
                } else if ip_ratio > 3.5 {
                    suggestions.push(TuningSuggestion {
                        category: "I Gain".to_string(),
                        title: format!("{} I-gain is very high", name),
                        description: format!("{}: P={:.0}, I={:.0} (I/P: {:.2}). May cause I-term bounce.", name, p, i, ip_ratio),
                        recommendation: "High I can cause slow oscillations. Consider reducing if quad feels floaty.".to_string(),
                        cli_command: Some(format!("set {}_i = {}", cli_axis_names[axis], (p * 2.0) as i32)),
                        severity: Severity::Info,
                    });
                }
            }
        }
        
        // I-term relax suggestion based on error data
        if analysis.tracking_error_rms.iter().any(|&e| e > 80.0) {
            let iterm_relax = get_val(&["iterm_relax", "iterm_relax_type"]).unwrap_or(0.0);
            if iterm_relax == 0.0 {
                suggestions.push(TuningSuggestion {
                    category: "I Gain".to_string(),
                    title: "Consider enabling I-term relax".to_string(),
                    description: "I-term relax reduces I-term buildup during fast maneuvers.".to_string(),
                    recommendation: "Enable I-term relax for better freestyle performance.".to_string(),
                    cli_command: Some("set iterm_relax = RP\nset iterm_relax_type = SETPOINT".to_string()),
                    severity: Severity::Info,
                });
            }
        }
        
        // Antigravity suggestion
        let antigravity_gain = get_val(&["anti_gravity_gain", "antigravity_gain"]).unwrap_or(0.0);
        if antigravity_gain < 3.0 && antigravity_gain > 0.0 {
            suggestions.push(TuningSuggestion {
                category: "I Gain".to_string(),
                title: "Antigravity gain is low".to_string(),
                description: format!("Antigravity: {:.1}. Low values may cause altitude bobbing on punchouts.", antigravity_gain),
                recommendation: "Increase antigravity to 3.5-5.0 for better throttle response.".to_string(),
                cli_command: Some("set anti_gravity_gain = 35".to_string()),
                severity: Severity::Info,
            });
        }

        // === TPA (Throttle PID Attenuation) SUGGESTIONS ===
        let tpa_rate = get_val(&["tpa_rate"]).unwrap_or(0.0);
        let tpa_breakpoint = get_val(&["tpa_breakpoint"]).unwrap_or(1250.0);
        
        // If high noise and TPA not configured
        if max_gyro_noise > 40.0 && tpa_rate < 0.2 {
            suggestions.push(TuningSuggestion {
                category: "TPA".to_string(),
                title: "Consider adding TPA".to_string(),
                description: "TPA reduces PID authority at high throttle to prevent oscillations.".to_string(),
                recommendation: "Add TPA to reduce noise sensitivity at high throttle.".to_string(),
                cli_command: Some("set tpa_rate = 65\nset tpa_breakpoint = 1350".to_string()),
                severity: Severity::Info,
            });
        }
        
        // TPA breakpoint too low
        if tpa_breakpoint < 1200.0 && tpa_rate > 0.3 {
            suggestions.push(TuningSuggestion {
                category: "TPA".to_string(),
                title: "TPA breakpoint may be too low".to_string(),
                description: format!("TPA breakpoint: {}. Low values affect mid-throttle handling.", tpa_breakpoint as i32),
                recommendation: "Raise TPA breakpoint to 1350-1450 for better mid-throttle feel.".to_string(),
                cli_command: Some("set tpa_breakpoint = 1400".to_string()),
                severity: Severity::Info,
            });
        }

        // === THROTTLE-BAND SPECIFIC ANALYSIS ===
        // Analyze noise at different throttle positions for targeted suggestions
        if analysis.gyro_noise_high_throttle > 0.0 || analysis.gyro_noise_low_throttle > 0.0 {
            let low_noise = analysis.gyro_noise_low_throttle;
            let mid_noise = analysis.gyro_noise_mid_throttle;
            let high_noise = analysis.gyro_noise_high_throttle;
            
            // High throttle noise significantly higher than low
            if high_noise > low_noise * 2.0 && high_noise > 20.0 {
                suggestions.push(TuningSuggestion {
                    category: "Throttle Analysis".to_string(),
                    title: "Noise increases with throttle".to_string(),
                    description: format!("Noise: Low={:.1}, Mid={:.1}, High={:.1}. High-throttle noise is significantly elevated.", low_noise, mid_noise, high_noise),
                    recommendation: "Consider: TPA, RPM filter, or motor/prop balance issues.".to_string(),
                    cli_command: Some("set tpa_rate = 65\nset tpa_breakpoint = 1350".to_string()),
                    severity: Severity::Warning,
                });
            }
            
            // Low throttle noise high (hover instability)
            if low_noise > 25.0 && low_noise > high_noise * 0.8 {
                suggestions.push(TuningSuggestion {
                    category: "Throttle Analysis".to_string(),
                    title: "Elevated low-throttle noise (hover)".to_string(),
                    description: format!("Hover noise: {:.1}. May cause instability at low throttle.", low_noise),
                    recommendation: "Possible causes: D-term too high, prop damage, or frame flex.".to_string(),
                    cli_command: None,
                    severity: Severity::Info,
                });
            }
            
            // Mid throttle is the sweet spot - use it as reference
            if mid_noise > 30.0 && mid_noise > low_noise && mid_noise > high_noise {
                suggestions.push(TuningSuggestion {
                    category: "Throttle Analysis".to_string(),
                    title: "Mid-throttle noise is highest".to_string(),
                    description: format!("Mid-throttle noise ({:.1}) exceeds low and high. Possible resonance.", mid_noise),
                    recommendation: "Check for frame resonance at cruising speed. Dynamic notch may help.".to_string(),
                    cli_command: Some("set dyn_notch_count = 3\nset dyn_notch_q = 400".to_string()),
                    severity: Severity::Warning,
                });
            }
        }
        
        // Throttle usage pattern suggestions
        if analysis.avg_throttle_pct > 0.0 {
            if analysis.avg_throttle_pct > 70.0 {
                suggestions.push(TuningSuggestion {
                    category: "Throttle Analysis".to_string(),
                    title: "High average throttle".to_string(),
                    description: format!("Avg throttle: {:.0}%. Flying at high power most of time.", analysis.avg_throttle_pct),
                    recommendation: "Consider more aggressive TPA settings to protect against high-throttle oscillation.".to_string(),
                    cli_command: None,
                    severity: Severity::Info,
                });
            }
            
            if analysis.throttle_variance > 20.0 {
                suggestions.push(TuningSuggestion {
                    category: "Throttle Analysis".to_string(),
                    title: "High throttle variance".to_string(),
                    description: format!("Throttle variance: {:.0}%. Dynamic flying with big throttle changes.", analysis.throttle_variance),
                    recommendation: "Ensure antigravity is enabled for punch-out performance.".to_string(),
                    cli_command: Some("set anti_gravity_gain = 40".to_string()),
                    severity: Severity::Info,
                });
            }
        }

        // === RATES ANALYSIS ===
        // Parse rates from headers
        let roll_rc_rate = get_val(&["roll_rc_rate", "rates_0_0", "rc_rates[0]"]).unwrap_or(0.0);
        let _roll_srate = get_val(&["roll_srate", "rates_0_1", "rc_expo[0]"]).unwrap_or(0.0);
        let roll_expo = get_val(&["roll_expo", "rates_0_2"]).unwrap_or(0.0);
        
        // Check if rates are extremely high (freestyle) or low (cinematic)
        if roll_rc_rate > 2.0 {
            suggestions.push(TuningSuggestion {
                category: "Rates".to_string(),
                title: "Very high rates detected".to_string(),
                description: format!("RC rate: {:.2}. Very high rates require precise PIDs.", roll_rc_rate),
                recommendation: "High rates amplify tuning issues. Consider slightly lower rates if unstable.".to_string(),
                cli_command: None,
                severity: Severity::Info,
            });
        }
        
        // Low expo warning
        if (0.0..0.1).contains(&roll_expo) && roll_rc_rate > 1.0 {
            suggestions.push(TuningSuggestion {
                category: "Rates".to_string(),
                title: "Low or no expo".to_string(),
                description: format!("Expo: {:.2}. Low expo makes small corrections harder.", roll_expo),
                recommendation: "Add expo (0.1-0.3) for better fine control around center stick.".to_string(),
                cli_command: None,
                severity: Severity::Info,
            });
        }

        // === PROPWASH INDICATION ===
        // Propwash tends to show as oscillation + high D noise together
        if analysis.step_oscillations.iter().any(|&o| o) && max_dterm_noise > 60.0 {
            suggestions.push(TuningSuggestion {
                category: "Propwash".to_string(),
                title: "Possible propwash oscillation".to_string(),
                description: "Oscillations detected with high D-term noise. Classic propwash signature.".to_string(),
                recommendation: "Try: lower D, raise D-min, enable anti_gravity, use feedforward smoothing.".to_string(),
                cli_command: Some("set feedforward_smooth_factor = 25\nset feedforward_jitter_factor = 10".to_string()),
                severity: Severity::Warning,
            });
        }

        // === FLIGHT STYLE DETECTION ===
        // Analyze setpoint variance to detect flying style
        let mut setpoint_variance = 0.0f32;
        let mut avg_throttle = 0.0f32;
        if let Some(setpoint) = fd.setpoint() {
            // High variance = aggressive flying, low = smooth cinematic
            if setpoint[0].len() > 100 {
                let mean: f32 = setpoint[0].iter().sum::<f32>() / setpoint[0].len() as f32;
                let variance: f32 = setpoint[0].iter().map(|x| (x - mean).powi(2)).sum::<f32>() / setpoint[0].len() as f32;
                setpoint_variance = variance.sqrt();
            }
        }
        
        // Get average throttle from motor data (if available)
        if let Some(motors) = fd.motor() {
            if motors.len() >= 4 && !motors[0].is_empty() {
                let total: f32 = motors.iter().filter_map(|m| m.first()).sum();
                avg_throttle = total / 4.0;
            }
        }
        
        // Flight style tips based on detected patterns
        if setpoint_variance > 200.0 {
            suggestions.push(TuningSuggestion {
                category: "Flight Style".to_string(),
                title: "Aggressive flying style detected".to_string(),
                description: format!("High stick input variance ({:.0}). Freestyle/racing tune recommended.", setpoint_variance),
                recommendation: "For aggressive flying: slightly higher P, moderate D, lower I may feel better.".to_string(),
                cli_command: None,
                severity: Severity::Info,
            });
        } else if setpoint_variance < 50.0 && setpoint_variance > 0.0 {
            suggestions.push(TuningSuggestion {
                category: "Flight Style".to_string(),
                title: "Smooth flying style detected".to_string(),
                description: format!("Low stick variance ({:.0}). Cinematic tune may be preferred.", setpoint_variance),
                recommendation: "For cinematic: lower P for smoother feel, higher I for precise holds.".to_string(),
                cli_command: None,
                severity: Severity::Info,
            });
        }

        // === LOG QUALITY CHECK ===
        {
            let total_samples = fd.times.len();
            if total_samples < 1000 {
                suggestions.push(TuningSuggestion {
                    category: "Log Quality".to_string(),
                    title: "Short flight log".to_string(),
                    description: format!("Only {} samples. Analysis may be less accurate.", total_samples),
                    recommendation: "Longer flights (30+ seconds) provide more accurate analysis.".to_string(),
                    cli_command: None,
                    severity: Severity::Info,
                });
            }
            
            // Check sample rate consistency
            if fd.times.len() > 100 {
                let dt_samples: Vec<f64> = fd.times.windows(2).take(100).map(|w| w[1] - w[0]).collect();
                let avg_dt = dt_samples.iter().sum::<f64>() / dt_samples.len() as f64;
                let dt_variance: f64 = dt_samples.iter().map(|dt| (dt - avg_dt).powi(2)).sum::<f64>() / dt_samples.len() as f64;
                let dt_std = dt_variance.sqrt();
                
                // High variance indicates logging issues
                if dt_std > avg_dt * 0.2 {
                    suggestions.push(TuningSuggestion {
                        category: "Log Quality".to_string(),
                        title: "Inconsistent sample timing".to_string(),
                        description: format!("Sample timing variance: {:.2}ms. May indicate SD card issues.", dt_std * 1000.0),
                        recommendation: "Use a fast SD card (U3/V30) for consistent blackbox logging.".to_string(),
                        cli_command: None,
                        severity: Severity::Warning,
                    });
                }
            }
        }

        // === MOTOR TEMP ESTIMATION ===
        // Combine D-term noise, throttle, and duration for heat warning
        let flight_duration = fd.times.last().copied().unwrap_or(0.0) - fd.times.first().copied().unwrap_or(0.0);
        let heat_risk = max_dterm_noise * (avg_throttle / 1500.0) * (flight_duration as f32 / 60.0);
        if heat_risk > 50.0 {
            suggestions.push(TuningSuggestion {
                category: "Motor Health".to_string(),
                title: "High motor heat risk".to_string(),
                description: format!("Estimated heat factor: {:.0}. Based on D-noise + throttle + duration.", heat_risk),
                recommendation: "Let motors cool between flights. Consider lower D or RPM filter.".to_string(),
                cli_command: None,
                severity: Severity::Warning,
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
        // Check for async AI response
        if let Some(ref receiver) = self.ai_receiver {
            if let Ok(result) = receiver.try_recv() {
                self.ai_loading = false;
                match result {
                    AIAnalysisResult::Success(response) => {
                        self.ai_response = Some(response);
                        self.ai_error = None;
                    }
                    AIAnalysisResult::Error(err) => {
                        self.ai_error = Some(err);
                        self.ai_response = None;
                    }
                }
                self.ai_receiver = None;
            }
        }
        
        // Check for async model fetch result
        if let Some(ref receiver) = self.models_receiver {
            if let Ok(result) = receiver.try_recv() {
                self.models_loading = false;
                match result {
                    ModelFetchResult::Success(fetched_models) => {
                        self.models = fetched_models;
                    }
                    ModelFetchResult::Error(_) => {
                        // Keep default models on error
                    }
                }
                self.models_receiver = None;
            }
        }

        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("🔧 PID & Filter Tuning Suggestions");
            ui.add_space(8.0);
            
            // === AI ANALYSIS SECTION ===
            ui.add_space(8.0);
            egui::CollapsingHeader::new(RichText::new("🤖 AI Analysis (OpenRouter)").strong())
                .default_open(self.ai_settings_open)
                .show(ui, |ui| {
                    self.ai_settings_open = true;
                    
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.label("API Key:");
                        ui.add(egui::TextEdit::singleline(&mut self.api_key)
                            .password(true)
                            .hint_text("sk-or-...")
                            .desired_width(300.0));
                    });
                    
                    // Model filter and selection
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        ui.label("🔍 Filter:");
                        ui.add(egui::TextEdit::singleline(&mut self.model_filter)
                            .hint_text("Type to filter models...")
                            .desired_width(200.0));
                        
                        if self.models_loading {
                            ui.spinner();
                            ui.label("Loading models...");
                        } else {
                            ui.label(format!("{} models available", self.models.len()));
                        }
                    });
                    
                    // Filter models by name (case-insensitive contains)
                    let filter_lower = self.model_filter.to_lowercase();
                    let filtered_models: Vec<&AIModel> = self.models
                        .iter()
                        .filter(|m| {
                            if filter_lower.is_empty() {
                                true
                            } else {
                                m.name.to_lowercase().contains(&filter_lower) ||
                                m.id.to_lowercase().contains(&filter_lower)
                            }
                        })
                        .collect();
                    
                    // Get selected model name for display
                    let selected_name = self.selected_model_id
                        .as_ref()
                        .and_then(|id| self.models.iter().find(|m| &m.id == id))
                        .map(|m| m.name.as_str())
                        .unwrap_or("Select a model...");
                    
                    ui.horizontal(|ui| {
                        ui.label("Model:");
                        egui::ComboBox::from_label("")
                            .selected_text(selected_name)
                            .width(400.0)
                            .show_ui(ui, |ui| {
                                egui::ScrollArea::vertical()
                                    .max_height(300.0)
                                    .show(ui, |ui| {
                                        for model in &filtered_models {
                                            let is_selected = self.selected_model_id.as_ref() == Some(&model.id);
                                            if ui.selectable_label(is_selected, &model.name).clicked() {
                                                self.selected_model_id = Some(model.id.clone());
                                            }
                                        }
                                        
                                        if filtered_models.is_empty() {
                                            ui.label("No models match filter");
                                        }
                                    });
                            });
                        
                        // Save Settings button
                        if ui.button("💾 Save Settings")
                            .on_hover_text("Save API key and model selection for next session")
                            .clicked()
                        {
                            let settings = crate::settings::AppSettings {
                                ai: crate::settings::AISettings {
                                    api_key: self.api_key.clone(),
                                    selected_model_id: self.selected_model_id.clone(),
                                },
                            };
                            settings.save();
                            log::info!("AI settings saved");
                        }
                    });
                    
                    ui.add_space(8.0);
                    
                    ui.horizontal(|ui| {
                        let can_analyze = !self.api_key.is_empty() && 
                                          !self.ai_loading && 
                                          self.selected_model_id.is_some();
                        
                        if ui.add_enabled(can_analyze, egui::Button::new("🚀 Analyze with AI")).clicked() {
                            // Build metrics from analysis
                            let metrics = self.build_flight_metrics();
                            let model_id = self.selected_model_id.clone().unwrap_or_default();
                            let api_key = self.api_key.clone();
                            
                            // Start async analysis
                            self.ai_loading = true;
                            self.ai_error = None;
                            self.ai_receiver = Some(OpenRouterClient::analyze_async(api_key, model_id, metrics));
                        }
                        
                        if ui.button("🔄 Refresh Models").clicked() && !self.models_loading {
                            self.models_loading = true;
                            self.models_receiver = Some(OpenRouterClient::fetch_models_async());
                        }
                        
                        if self.ai_loading {
                            ui.spinner();
                            ui.label("Analyzing...");
                        }
                    });
                    
                    // Show error if any
                    if let Some(ref err) = self.ai_error {
                        ui.add_space(8.0);
                        ui.colored_label(Color32::RED, format!("❌ {}", err));
                    }
                    
                    // Show AI response
                    if let Some(ref response) = self.ai_response {
                        ui.add_space(12.0);
                        ui.separator();
                        ui.add_space(8.0);
                        
                        ui.label(RichText::new("AI Analysis Results").strong().size(15.0));
                        ui.add_space(8.0);
                        
                        egui::Frame::none()
                            .fill(ui.style().visuals.extreme_bg_color)
                            .rounding(8.0)
                            .inner_margin(12.0)
                            .show(ui, |ui| {
                                egui::ScrollArea::vertical()
                                    .max_height(400.0)
                                    .show(ui, |ui| {
                                        // Render markdown-like response
                                        for line in response.lines() {
                                            if line.starts_with("```") {
                                                // Skip code block markers
                                            } else if line.starts_with('#') {
                                                let text = line.trim_start_matches('#').trim();
                                                ui.label(RichText::new(text).strong().size(14.0));
                                            } else if line.starts_with('`') && line.ends_with('`') {
                                                let cmd = line.trim_matches('`');
                                                ui.horizontal(|ui| {
                                                    ui.monospace(RichText::new(cmd).color(Color32::LIGHT_GREEN));
                                                    if ui.small_button("📋").on_hover_text("Copy").clicked() {
                                                        ui.output_mut(|o| o.copied_text = cmd.to_string());
                                                    }
                                                });
                                            } else if line.starts_with("- ") || line.starts_with("* ") {
                                                ui.horizontal(|ui| {
                                                    ui.label("•");
                                                    ui.label(&line[2..]);
                                                });
                                            } else if !line.is_empty() {
                                                ui.label(line);
                                            } else {
                                                ui.add_space(4.0);
                                            }
                                        }
                                    });
                            });
                        
                        ui.add_space(8.0);
                        ui.horizontal(|ui| {
                            if ui.button("📋 Copy Response").clicked() {
                                ui.output_mut(|o| o.copied_text = response.clone());
                            }
                            if ui.button("🗑 Clear").clicked() {
                                self.ai_response = None;
                            }
                        });
                    }
                });
            
            ui.add_space(16.0);
            ui.separator();
            ui.add_space(8.0);
            
            // === RULE-BASED SUGGESTIONS ===
            ui.label(RichText::new("📋 Automated Analysis").strong().size(16.0));
            ui.label("Based on analysis of your flight log, here are tuning recommendations:");
            ui.add_space(8.0);
            
            // Export All CLI Commands section
            ui.horizontal(|ui| {
                // Copy All button - groups by severity for safe application order
                if ui.button("📋 Copy All CLI Commands").on_hover_text("Copy all CLI commands to clipboard (Critical → Warning → Info)").clicked() {
                    let all_commands = self.generate_cli_export();
                    if !all_commands.is_empty() {
                        ui.output_mut(|o| o.copied_text = all_commands);
                    }
                }
                
                // Copy Critical Only button
                let critical_count = self.suggestions.iter()
                    .filter(|s| s.severity == Severity::Critical && s.cli_command.is_some())
                    .count();
                if critical_count > 0 {
                    if ui.button(format!("🔴 Copy Critical Only ({})", critical_count))
                        .on_hover_text("Copy only critical priority commands")
                        .clicked() 
                    {
                        let commands = self.generate_cli_export_filtered(Some(Severity::Critical));
                        ui.output_mut(|o| o.copied_text = commands);
                    }
                }
                
                // Count how many have CLI commands
                let cmd_count = self.suggestions.iter().filter(|s| s.cli_command.is_some()).count();
                let warning_count = self.suggestions.iter()
                    .filter(|s| s.severity == Severity::Warning && s.cli_command.is_some())
                    .count();
                ui.label(RichText::new(format!("{} total ({} critical, {} warning)", 
                    cmd_count, critical_count, warning_count)).weak());
            });
            
            // Collapsible CLI Snippet Preview
            if self.suggestions.iter().any(|s| s.cli_command.is_some()) {
                ui.add_space(8.0);
                egui::CollapsingHeader::new(RichText::new("📄 CLI Commands Preview").size(13.0))
                    .default_open(false)
                    .show(ui, |ui| {
                        let preview = self.generate_cli_export();
                        egui::Frame::none()
                            .fill(egui::Color32::from_black_alpha(60))
                            .rounding(4.0)
                            .inner_margin(8.0)
                            .show(ui, |ui| {
                                egui::ScrollArea::vertical()
                                    .max_height(200.0)
                                    .show(ui, |ui| {
                                        ui.monospace(RichText::new(&preview).size(11.0).color(egui::Color32::LIGHT_GREEN));
                                    });
                            });
                    });
            }
            
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
    
    /// Build FlightMetrics for AI analysis
    fn build_flight_metrics(&self) -> FlightMetrics {
        let headers = &self.fd.unknown_headers;
        
        // Parse PIDs from headers
        let parse_pid = |key: &str| -> [f32; 4] {
            headers.get(key)
                .map(|s| {
                    let parts: Vec<f32> = s.split(',')
                        .filter_map(|v| v.trim().parse().ok())
                        .collect();
                    [
                        parts.first().copied().unwrap_or(0.0),
                        parts.get(1).copied().unwrap_or(0.0),
                        parts.get(2).copied().unwrap_or(0.0),
                        parts.get(3).copied().unwrap_or(0.0),
                    ]
                })
                .unwrap_or([0.0; 4])
        };
        
        // Parse D-min values
        let get_dmin = |axis: &str| -> f32 {
            headers.get(&format!("d_min_{}", axis))
                .or_else(|| headers.get(&format!("dmin_{}", axis)))
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0)
        };
        
        let duration = self.fd.times.last().copied().unwrap_or(0.0);
        
        FlightMetrics {
            firmware: headers.get("Firmware revision").cloned().unwrap_or_else(|| "Unknown".to_string()),
            craft_name: headers.get("Craft name").cloned().unwrap_or_else(|| "Unknown".to_string()),
            duration_sec: duration,
            roll_pid: parse_pid("rollPID"),
            pitch_pid: parse_pid("pitchPID"),
            yaw_pid: parse_pid("yawPID"),
            d_min: [get_dmin("roll"), get_dmin("pitch"), get_dmin("yaw")],
            step_overshoot: [
                self.analysis.step_overshoot[0] as f32,
                self.analysis.step_overshoot[1] as f32,
                self.analysis.step_overshoot[2] as f32,
            ],
            step_undershoot: [
                self.analysis.step_undershoot[0] as f32,
                self.analysis.step_undershoot[1] as f32,
                self.analysis.step_undershoot[2] as f32,
            ],
            gyro_noise_rms: self.analysis.gyro_noise_rms,
            dterm_noise_rms: [self.analysis.dterm_noise_rms[0], self.analysis.dterm_noise_rms[1], 0.0],
            tracking_error_rms: self.analysis.tracking_error_rms,
            motor_saturation_pct: self.analysis.motor_saturation_pct,
            gyro_lpf_hz: headers.get("gyro_lpf1_static_hz")
                .or_else(|| headers.get("gyro_lowpass_hz"))
                .and_then(|s| s.parse().ok()),
            gyro_lpf2_hz: headers.get("gyro_lpf2_static_hz")
                .and_then(|s| s.parse().ok()),
            dterm_lpf_hz: headers.get("dterm_lpf1_static_hz")
                .or_else(|| headers.get("dterm_lowpass_hz"))
                .and_then(|s| s.parse().ok()),
            dterm_lpf2_hz: headers.get("dterm_lpf2_static_hz")
                .and_then(|s| s.parse().ok()),
            yaw_lpf_hz: headers.get("yaw_lpf_hz")
                .or_else(|| headers.get("yaw_lowpass_hz"))
                .and_then(|s| s.parse().ok()),
            dyn_notch_count: headers.get("dyn_notch_count")
                .and_then(|s| s.parse().ok()),
            dyn_notch_min: headers.get("dyn_notch_min_hz")
                .and_then(|s| s.parse().ok()),
            dyn_notch_max: headers.get("dyn_notch_max_hz")
                .and_then(|s| s.parse().ok()),
            rpm_filter_harmonics: headers.get("rpm_filter_harmonics")
                .and_then(|s| s.parse().ok()),
            
            // ===== Enhanced metrics =====
            // Rate settings
            rc_rate: {
                let roll = headers.get("roll_rc_rate").or_else(|| headers.get("rc_rate")).and_then(|s| s.parse().ok());
                let pitch = headers.get("pitch_rc_rate").or_else(|| headers.get("rc_rate")).and_then(|s| s.parse().ok());
                let yaw = headers.get("yaw_rc_rate").and_then(|s| s.parse().ok());
                match (roll, pitch, yaw) {
                    (Some(r), Some(p), Some(y)) => Some([r, p, y]),
                    _ => None,
                }
            },
            rc_expo: {
                let roll = headers.get("roll_expo").or_else(|| headers.get("rc_expo")).and_then(|s| s.parse().ok());
                let pitch = headers.get("pitch_expo").or_else(|| headers.get("rc_expo")).and_then(|s| s.parse().ok());
                let yaw = headers.get("yaw_expo").and_then(|s| s.parse().ok());
                match (roll, pitch, yaw) {
                    (Some(r), Some(p), Some(y)) => Some([r, p, y]),
                    _ => None,
                }
            },
            super_rate: {
                let roll = headers.get("roll_srate").or_else(|| headers.get("rc_super_rate")).and_then(|s| s.parse().ok());
                let pitch = headers.get("pitch_srate").or_else(|| headers.get("rc_super_rate")).and_then(|s| s.parse().ok());
                let yaw = headers.get("yaw_srate").and_then(|s| s.parse().ok());
                match (roll, pitch, yaw) {
                    (Some(r), Some(p), Some(y)) => Some([r, p, y]),
                    _ => None,
                }
            },
            
            // Motor statistics - use defaults (can be enhanced later)
            motor_min_pct: 0.0,
            motor_max_pct: 100.0,
            motor_avg_pct: 50.0,
            motor_differential: 0.0,
            
            // Anomaly counts - use defaults (can be enhanced later)
            anomaly_count: 0,
            motor_saturation_events: 0,
            high_vibration_events: 0,
            
            // Step response timing (not computed yet - future enhancement)
            step_rise_time_ms: None,
            step_settling_time_ms: None,
        }
    }
    
    /// Generate CLI export string with all commands grouped by severity
    fn generate_cli_export(&self) -> String {
        self.generate_cli_export_filtered(None)
    }
    
    /// Generate CLI export string, optionally filtered by severity
    fn generate_cli_export_filtered(&self, severity_filter: Option<Severity>) -> String {
        let mut output = String::from("# Bucksaw Tuning Suggestions\n");
        output.push_str("# Generated from flight log analysis\n");
        output.push_str("# Apply in Betaflight CLI (Configurator → CLI tab)\n\n");
        
        // Group by severity for safe application order: Critical → Warning → Info
        let severity_order = [Severity::Critical, Severity::Warning, Severity::Info];
        let severity_labels = ["CRITICAL", "WARNING", "INFO"];
        
        let mut has_commands = false;
        
        for (severity, label) in severity_order.iter().zip(severity_labels.iter()) {
            // Skip if filtering and this isn't the selected severity
            if let Some(filter) = severity_filter {
                if *severity != filter {
                    continue;
                }
            }
            
            let severity_suggestions: Vec<_> = self.suggestions.iter()
                .filter(|s| s.severity == *severity && s.cli_command.is_some())
                .collect();
            
            if severity_suggestions.is_empty() {
                continue;
            }
            
            output.push_str(&format!("\n# ========== {} ==========\n", label));
            
            let mut current_category = String::new();
            for suggestion in severity_suggestions {
                if let Some(cmd) = &suggestion.cli_command {
                    if suggestion.category != current_category {
                        output.push_str(&format!("\n# --- {} ---\n", suggestion.category));
                        current_category = suggestion.category.clone();
                    }
                    
                    output.push_str(&format!("# {}\n", suggestion.title));
                    for line in cmd.lines() {
                        output.push_str(line);
                        output.push('\n');
                    }
                    has_commands = true;
                }
            }
        }
        
        if has_commands {
            output.push_str("\n# Don't forget to save!\nsave\n");
        }
        
        output
    }
}

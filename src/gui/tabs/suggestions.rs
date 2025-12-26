use std::sync::mpsc::Receiver;
use std::sync::Arc;

use egui::{Color32, RichText, Ui};

use crate::analytics;
use crate::ai_integration::{
    AIAnalysisResult, AIModel, AnalysisFocus, FlightMetrics, ModelFetchResult, OpenRouterClient,
    DEFAULT_MODELS,
};
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
            Severity::Info => Color32::from_rgb(0x83, 0xa5, 0x98), // Green
            Severity::Warning => Color32::from_rgb(0xfa, 0xbd, 0x2f), // Yellow
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
    step_overshoot: [f64; 3],  // Max value above 1.0 (relative to step size)
    step_undershoot: [f64; 3], // Min value below 1.0 in first 100ms
    step_settling_time: [f64; 3], // Time to reach 0.98-1.02 range (ms)
    step_rise_time: [f64; 3],  // Time to go from 10% to 90% of setpoint (ms)
    step_bounce_back: [f64; 3], // Magnitude of negative peak after positive step
    step_oscillations: [bool; 3], // Has oscillations after settling

    // Noise metrics
    gyro_noise_rms: [f32; 3],  // RMS of high-freq gyro noise
    dterm_noise_rms: [f32; 2], // RMS of D-term (Roll, Pitch)

    // Throttle-segmented noise (low/mid/high) - average across axes
    gyro_noise_low_throttle: f32,  // <30% throttle
    gyro_noise_mid_throttle: f32,  // 30-70% throttle
    gyro_noise_high_throttle: f32, // >70% throttle

    // Per-axis throttle noise (Roll, Pitch, Yaw) for low, mid, high
    // [axis][throttle_band] where 0=low, 1=mid, 2=high
    gyro_noise_by_throttle: [[f32; 3]; 3],

    // Motor metrics
    motor_saturation_pct: f32,        // % of time any motor at max
    motor_saturation_sustained: bool, // True if saturation coincides with oscillations
    motor_desync_risk: bool,          // Large motor output variance
    motor_heat_risk: f32,             // Estimated heat risk factor

    // Error metrics
    tracking_error_rms: [f32; 3], // RMS of setpoint - gyro

    // Flight characteristics
    avg_throttle_pct: f32,  // Average throttle percentage
    throttle_variance: f32, // How much throttle varies
    flight_duration: f64,   // Duration in seconds
}

impl AnalysisResults {
    fn compute(fd: &FlightData) -> Self {
        let sample_rate = fd.sample_rate();
        let flight_duration =
            fd.times.last().copied().unwrap_or(0.0) - fd.times.first().copied().unwrap_or(0.0);

        // Compute step response metrics
        let mut step_overshoot = [0.0; 3];
        let mut step_undershoot = [0.0; 3];
        let mut step_settling_time = [0.0; 3];
        let mut step_oscillations = [false; 3];
        let mut step_rise_time = [0.0; 3];
        let mut step_bounce_back = [0.0; 3];

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
                    let mut max_overshoot = 0.0;
                    let mut max_bounce_back = 0.0;

                    // Rise time calculation (10% to 90%)
                    let start_idx = response.iter().position(|(_, y)| *y >= 0.1);
                    let end_idx = response.iter().position(|(_, y)| *y >= 0.9);

                    if let (Some(start), Some(end)) = (start_idx, end_idx) {
                        if end > start {
                            step_rise_time[axis] = response[end].0 - response[start].0;
                        }
                    } else if let Some(start) = start_idx {
                        // If it never reaches 90%, just take time to max
                        step_rise_time[axis] = response.last().unwrap().0 - response[start].0;
                    }

                    // Max overshoot (value above 1.0)
                    for (t, y) in &response {
                        let overshoot = (*y - 1.0).max(0.0);
                        if overshoot > max_overshoot {
                            max_overshoot = overshoot;
                        }

                        // Bounce back detection: look for dip after initial rise
                        // This is simplified; a real check would detect peak first
                    }
                    step_overshoot[axis] = max_overshoot;

                    // Improved bounce-back and settling
                    let mut peak_found = false;
                    let mut peak_val = 0.0;
                    let mut settled = false;

                    for (i, (t, y)) in response.iter().enumerate() {
                        if !peak_found {
                            if *y > 1.0 && (i + 1 < response.len() && response[i + 1].1 < *y) {
                                peak_found = true;
                                peak_val = *y;
                            }
                        } else {
                            // After peak, look for minimum
                            if *y < 1.0 {
                                let bounce = (1.0 - *y).max(0.0);
                                if bounce > max_bounce_back {
                                    max_bounce_back = bounce;
                                }
                            }
                        }

                        // Settling time (time to stay within 0.98-1.02)
                        if (*y - 1.0).abs() <= 0.02 {
                            if !settled {
                                step_settling_time[axis] = *t;
                                settled = true;
                            }
                        } else if *t > 0.05 {
                            // Ignore initial rise
                            settled = false; // Reset if it goes out of bounds
                        }

                        // Oscillation check after "settling" timeframe (e.g. > 100ms)
                        if *t > 0.100 && settled && (*y - 1.0).abs() > 0.05 {
                            step_oscillations[axis] = true;
                        }
                    }
                    step_bounce_back[axis] = max_bounce_back;

                    // Undershoot in first 100ms (settling too slow)
                    let first_100ms = (sample_rate * 0.1) as usize;
                    step_undershoot[axis] = response
                        .iter()
                        .take(first_100ms)
                        .map(|(_, y)| (1.0 - y).max(0.0))
                        .fold(0.0, f64::max);
                }
            }
        }

        // Commpute throttle values first for segmentation
        let mut throttle_values = Vec::new();
        let mut avg_throttle_pct = 0.0f32;
        let mut throttle_variance = 0.0f32;

        if let Some(motors) = fd.motor() {
            if motors.len() >= 4 && !motors[0].is_empty() {
                let total_samples = motors[0].len();

                // Calculate throttle values (average of 4 motors, normalized to %)
                throttle_values = (0..total_samples)
                    .map(|i| {
                        let sum: f32 = motors.iter().filter_map(|m| m.get(i)).sum();
                        sum / 4.0 / 20.0 // Normalize to rough percentage (2000 = 100%)
                    })
                    .collect();

                if !throttle_values.is_empty() {
                    avg_throttle_pct =
                        throttle_values.iter().sum::<f32>() / throttle_values.len() as f32;
                    let variance: f32 = throttle_values
                        .iter()
                        .map(|t| (t - avg_throttle_pct).powi(2))
                        .sum::<f32>()
                        / throttle_values.len() as f32;
                    throttle_variance = variance.sqrt();
                }
            }
        }

        // Compute noise metrics (high-pass filter approximation)
        let mut gyro_noise_rms = [0.0f32; 3];
        let mut dterm_noise_rms = [0.0f32; 2];
        let mut gyro_noise_by_throttle = [[0.0f32; 3]; 3]; // [axis][band]

        if let Some(gyro) = fd.gyro_filtered() {
            for axis in 0..3 {
                if gyro[axis].len() > 10 {
                    // Simple high-pass: difference from moving average
                    let window_size = 5;
                    let noise: Vec<f32> = gyro[axis]
                        .windows(window_size)
                        .map(|w| {
                            let avg: f32 = w.iter().sum::<f32>() / window_size as f32;
                            (w[window_size / 2] - avg).abs()
                        })
                        .collect();

                    if !noise.is_empty() {
                        let sum_sq: f32 = noise.iter().map(|x| x * x).sum();
                        gyro_noise_rms[axis] = (sum_sq / noise.len() as f32).sqrt();

                        // Segment by throttle if available
                        if !throttle_values.is_empty() {
                            let mut band_samples = [Vec::new(), Vec::new(), Vec::new()];

                            for (i, val) in noise.iter().enumerate() {
                                if i < throttle_values.len() {
                                    let t = throttle_values[i];
                                    if t < 30.0 {
                                        band_samples[0].push(*val);
                                    } else if t < 70.0 {
                                        band_samples[1].push(*val);
                                    } else {
                                        band_samples[2].push(*val);
                                    }
                                }
                            }

                            for band in 0..3 {
                                if !band_samples[band].is_empty() {
                                    let sum_sq: f32 =
                                        band_samples[band].iter().map(|x| x * x).sum();
                                    gyro_noise_by_throttle[axis][band] =
                                        (sum_sq / band_samples[band].len() as f32).sqrt();
                                }
                            }
                        }
                    }
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

        // Aggregate throttle noise for legacy fields
        let gyro_noise_low_throttle = (gyro_noise_by_throttle[0][0]
            + gyro_noise_by_throttle[1][0]
            + gyro_noise_by_throttle[2][0])
            / 3.0;
        let gyro_noise_mid_throttle = (gyro_noise_by_throttle[0][1]
            + gyro_noise_by_throttle[1][1]
            + gyro_noise_by_throttle[2][1])
            / 3.0;
        let gyro_noise_high_throttle = (gyro_noise_by_throttle[0][2]
            + gyro_noise_by_throttle[1][2]
            + gyro_noise_by_throttle[2][2])
            / 3.0;

        // Motor saturation analysis
        let mut motor_saturation_pct = 0.0f32;
        let mut motor_desync_risk = false;
        let mut motor_saturation_sustained = false;

        if let Some(motors) = fd.motor() {
            if motors.len() >= 4 {
                let total_samples = motors[0].len();
                let mut saturated_count = 0;
                let mut sustained_counter = 0;

                for i in 0..total_samples {
                    let motor_vals: Vec<f32> =
                        motors.iter().filter_map(|m| m.get(i).copied()).collect();

                    // Check if any motor at max (typically ~2000 for DShot)
                    if motor_vals.iter().any(|&v| v > 1950.0) {
                        saturated_count += 1;
                        sustained_counter += 1;
                    } else {
                        sustained_counter = 0;
                    }

                    if sustained_counter > 50 {
                        // >50ms approx at 1kHz
                        motor_saturation_sustained = true;
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

        // Heat risk estimation
        let max_dterm_noise = dterm_noise_rms.iter().cloned().fold(0.0f32, f32::max);
        let motor_heat_risk =
            max_dterm_noise * (avg_throttle_pct / 100.0) * (flight_duration as f32 / 60.0);

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
            step_rise_time,
            step_bounce_back,
            gyro_noise_rms,
            dterm_noise_rms,
            gyro_noise_low_throttle,
            gyro_noise_mid_throttle,
            gyro_noise_high_throttle,
            gyro_noise_by_throttle,
            motor_saturation_pct,
            motor_desync_risk,
            motor_saturation_sustained,
            motor_heat_risk,
            tracking_error_rms,
            avg_throttle_pct,
            throttle_variance,
            flight_duration,
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

    // Analysis focus option
    analysis_focus: AnalysisFocus,
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
        let selected_model_id = saved_settings
            .ai
            .selected_model_id
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
            analysis_focus: AnalysisFocus::default(),
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

        // Parse current PIDs & Settings
        let mut p_gains = [0.0f32; 3];
        let mut d_gains = [0.0f32; 3];
        let mut f_gains = [0.0f32; 3];

        for i in 0..3 {
            let parse_compound_pid = |key: &str, index: usize| -> Option<f32> {
                headers
                    .get(key)
                    .and_then(|s| s.split(',').nth(index))
                    .and_then(|v| v.trim().parse::<f32>().ok())
            };
            let axis_key = match i {
                0 => "rollPID",
                1 => "pitchPID",
                _ => "yawPID",
            };

            p_gains[i] = get_val(&[
                &format!("{}_p", cli_axis_names[i]),
                &format!("pid_{}_p", i),
                &format!("pid[{}][0]", i),
            ])
            .or_else(|| parse_compound_pid(axis_key, 0))
            .unwrap_or(0.0);
            d_gains[i] = get_val(&[
                &format!("{}_d", cli_axis_names[i]),
                &format!("pid_{}_d", i),
                &format!("pid[{}][2]", i),
            ])
            .or_else(|| parse_compound_pid(axis_key, 2))
            .unwrap_or(0.0);
            f_gains[i] = get_val(&[
                &format!("{}_f", cli_axis_names[i]),
                &format!("pid_{}_f", i),
                &format!("pid[{}][3]", i),
            ])
            .or_else(|| parse_compound_pid(axis_key, 3))
            .unwrap_or(0.0);
        }

        // Filter Settings
        let gyro_lpf1_hz = get_val(&["gyro_lpf1_static_hz", "gyro_lowpass_hz"]).unwrap_or(0.0);
        let gyro_lpf2_hz = get_val(&["gyro_lpf2_static_hz", "gyro_lowpass2_hz"]).unwrap_or(0.0);
        let dterm_lpf1_hz = get_val(&["dterm_lpf1_static_hz", "dterm_lowpass_hz"]).unwrap_or(0.0);

        // === STEP RESPONSE ANALYSIS (P & D Tuning) ===
        for (axis, name) in axis_names.iter().enumerate() {
            let overshoot = analysis.step_overshoot[axis];
            let rise_time = analysis.step_rise_time[axis];
            let bounce_back = analysis.step_bounce_back[axis];
            let _settling_time = analysis.step_settling_time[axis];

            // P-Gain Tuning
            if overshoot > 0.15 {
                // High overshoot -> Lower P
                suggestions.push(TuningSuggestion {
                    category: "P Gain".to_string(),
                    title: format!("{} P-gain too high (Overshoot)", name),
                    description: format!(
                        "Overshoot: {:.0}%. Quad is over-reacting.",
                        overshoot * 100.0
                    ),
                    recommendation:
                        "Reduce P-gain by 10-15%. If D is low, consider raising D instead."
                            .to_string(),
                    cli_command: if p_gains[axis] > 0.0 {
                        Some(format!(
                            "set {}_p = {}",
                            cli_axis_names[axis],
                            (p_gains[axis] * 0.9) as i32
                        ))
                    } else {
                        None
                    },
                    severity: if overshoot > 0.25 {
                        Severity::Warning
                    } else {
                        Severity::Info
                    },
                });
            } else if rise_time > 30.0 && overshoot < 0.05 {
                // Slow rise w/o overshoot -> Raise P (or FF)
                suggestions.push(TuningSuggestion {
                    category: "P Gain / Feedforward".to_string(),
                    title: format!("{} response is sluggish", name),
                    description: format!(
                        "Rise time: {:.1}ms. Quad is slow to reach setpoint.",
                        rise_time
                    ),
                    recommendation: "Increase P-gain or Feedforward for snappier response."
                        .to_string(),
                    cli_command: if p_gains[axis] > 0.0 {
                        Some(format!(
                            "set {}_p = {}",
                            cli_axis_names[axis],
                            (p_gains[axis] * 1.1) as i32
                        ))
                    } else {
                        None
                    },
                    severity: Severity::Info,
                });
            }

            // D-Gain Tuning (Bounce Back)
            if bounce_back > 0.10 {
                suggestions.push(TuningSuggestion {
                    category: "D Gain".to_string(),
                    title: format!("{} bounce-back detected", name),
                    description: format!(
                        "Bounce-back magnitude: {:.2}. Propwash or lack of damping.",
                        bounce_back
                    ),
                    recommendation: "Increase D-gain to dampen the stop. Check D-min/D-max ratio."
                        .to_string(),
                    cli_command: if d_gains[axis] > 0.0 {
                        Some(format!(
                            "set {}_d = {}",
                            cli_axis_names[axis],
                            (d_gains[axis] * 1.15) as i32
                        ))
                    } else {
                        None
                    },
                    severity: Severity::Warning,
                });
            }
        }

        // === NOISE & FILTERING ANALYSIS ===
        let max_dterm_noise = analysis
            .dterm_noise_rms
            .iter()
            .cloned()
            .fold(0.0f32, f32::max);
        if analysis.motor_heat_risk > 80.0 {
            suggestions.push(TuningSuggestion {
                category: "Motor Risk".to_string(),
                title: "CRITICAL: High Motor Heat Risk".to_string(),
                description: format!(
                    "Heat risk score: {:.0} (Extremely High). D-term noise + high throttle usage.",
                    analysis.motor_heat_risk
                ),
                recommendation: "Land immediately if motors are hot! Lower D-gains by 30-50%."
                    .to_string(),
                cli_command: Some(format!("set d_term_multiplier = 0.7")),
                severity: Severity::Critical,
            });
        }

        // Check for mid-throttle resonance
        if analysis.gyro_noise_mid_throttle > 5.0 {
            suggestions.push(TuningSuggestion {
                category: "Filters".to_string(),
                title: "Mid-throttle oscillations detected".to_string(),
                description: format!(
                    "Gyro noise peaks at mid-throttle ({:.1}). Possible mechanical resonance.",
                    analysis.gyro_noise_mid_throttle
                ),
                recommendation:
                    "Check for loose screws, frame resonance, or add a dynamic notch filter."
                        .to_string(),
                cli_command: None,
                severity: Severity::Warning,
            });
        }

        // Filter Overlap Check
        if gyro_lpf1_hz > 0.0 && gyro_lpf2_hz > 0.0 && (gyro_lpf1_hz - gyro_lpf2_hz).abs() < 20.0 {
            suggestions.push(TuningSuggestion {
                category: "Filtration".to_string(),
                title: "Redundant Gyro Filters".to_string(),
                description: "Gyro LPF1 and LPF2 are set to nearly the same frequency.".to_string(),
                recommendation: "Separate cutoffs or disable one filter to reduce latency."
                    .to_string(),
                cli_command: None,
                severity: Severity::Info,
            });
        }

        // D-Term Filter Latency Warning
        if dterm_lpf1_hz > 0.0 && dterm_lpf1_hz < 60.0 {
            suggestions.push(TuningSuggestion {
                category: "Filtration".to_string(),
                title: "D-Term Filter Latency".to_string(),
                description: format!(
                    "D-Term LPF1 is very low ({:.0}Hz), increasing latency.",
                    dterm_lpf1_hz
                ),
                recommendation: "Raise D-Term LPF to 90-100Hz if noise permits.".to_string(),
                cli_command: Some(format!("set dterm_lpf1_static_hz = 95")),
                severity: Severity::Warning,
            });
        }

        // Feedforward Analysis
        for (axis, name) in axis_names.iter().enumerate() {
            let error = analysis.tracking_error_rms[axis];
            if error > 20.0 {
                suggestions.push(TuningSuggestion {
                    category: "Feedforward".to_string(),
                    title: format!("{} poor tracking accuracy", name),
                    description: format!(
                        "Tracking error RMS: {:.1}. Drones is not following sticks well.",
                        error
                    ),
                    recommendation: "Increase Feedforward or P-gain.".to_string(),
                    cli_command: if f_gains[axis] > 0.0 {
                        Some(format!(
                            "set {}_f = {}",
                            cli_axis_names[axis],
                            (f_gains[axis] * 1.2) as i32
                        ))
                    } else {
                        None
                    },
                    severity: Severity::Info,
                });
            }
        }

        // Motor Saturation
        if analysis.motor_saturation_sustained {
            suggestions.push(TuningSuggestion {
                category: "Motors".to_string(),
                title: "Sustained Motor Saturation".to_string(),
                description: "Motors are hitting 100% throttle frequently.".to_string(),
                recommendation: "Drone is underpowered or PID gains are too high. Lower Master Multiplier or P/D gains.".to_string(),
                cli_command: Some("set pid_process_denom = 1".to_string()), // Placeholder
                severity: Severity::Critical,
            });
        }

        // === PROPWASH INDICATION ===
        // Propwash tends to show as oscillation + high D noise together
        if analysis.step_oscillations.iter().any(|&o| o) && max_dterm_noise > 60.0 {
            suggestions.push(TuningSuggestion {
                category: "Propwash".to_string(),
                title: "Possible propwash oscillation".to_string(),
                description:
                    "Oscillations detected with high D-term noise. Classic propwash signature."
                        .to_string(),
                recommendation:
                    "Try: lower D, raise D-min, enable anti_gravity, use feedforward smoothing."
                        .to_string(),
                cli_command: Some(
                    "set feedforward_smooth_factor = 25\nset feedforward_jitter_factor = 10"
                        .to_string(),
                ),
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
                let variance: f32 = setpoint[0].iter().map(|x| (x - mean).powi(2)).sum::<f32>()
                    / setpoint[0].len() as f32;
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
                description: format!(
                    "High stick input variance ({:.0}). Freestyle/racing tune recommended.",
                    setpoint_variance
                ),
                recommendation:
                    "For aggressive flying: slightly higher P, moderate D, lower I may feel better."
                        .to_string(),
                cli_command: None,
                severity: Severity::Info,
            });
        } else if setpoint_variance < 50.0 && setpoint_variance > 0.0 {
            suggestions.push(TuningSuggestion {
                category: "Flight Style".to_string(),
                title: "Smooth flying style detected".to_string(),
                description: format!(
                    "Low stick variance ({:.0}). Cinematic tune may be preferred.",
                    setpoint_variance
                ),
                recommendation:
                    "For cinematic: lower P for smoother feel, higher I for precise holds."
                        .to_string(),
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
                    description: format!(
                        "Only {} samples. Analysis may be less accurate.",
                        total_samples
                    ),
                    recommendation: "Longer flights (30+ seconds) provide more accurate analysis."
                        .to_string(),
                    cli_command: None,
                    severity: Severity::Info,
                });
            }

            // Check sample rate consistency
            if fd.times.len() > 100 {
                let dt_samples: Vec<f64> =
                    fd.times.windows(2).take(100).map(|w| w[1] - w[0]).collect();
                let avg_dt = dt_samples.iter().sum::<f64>() / dt_samples.len() as f64;
                let dt_variance: f64 = dt_samples
                    .iter()
                    .map(|dt| (dt - avg_dt).powi(2))
                    .sum::<f64>()
                    / dt_samples.len() as f64;
                let dt_std = dt_variance.sqrt();

                // High variance indicates logging issues
                if dt_std > avg_dt * 0.2 {
                    suggestions.push(TuningSuggestion {
                        category: "Log Quality".to_string(),
                        title: "Inconsistent sample timing".to_string(),
                        description: format!(
                            "Sample timing variance: {:.2}ms. May indicate SD card issues.",
                            dt_std * 1000.0
                        ),
                        recommendation:
                            "Use a fast SD card (U3/V30) for consistent blackbox logging."
                                .to_string(),
                        cli_command: None,
                        severity: Severity::Warning,
                    });
                }
            }
        }

        // === MOTOR TEMP ESTIMATION ===
        // Combine D-term noise, throttle, and duration for heat warning
        let flight_duration =
            fd.times.last().copied().unwrap_or(0.0) - fd.times.first().copied().unwrap_or(0.0);
        let heat_risk = max_dterm_noise * (avg_throttle / 1500.0) * (flight_duration as f32 / 60.0);
        if heat_risk > 50.0 {
            suggestions.push(TuningSuggestion {
                category: "Motor Health".to_string(),
                title: "High motor heat risk".to_string(),
                description: format!(
                    "Estimated heat factor: {:.0}. Based on D-noise + throttle + duration.",
                    heat_risk
                ),
                recommendation: "Let motors cool between flights. Consider lower D or RPM filter."
                    .to_string(),
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
            suggestions.insert(
                0,
                TuningSuggestion {
                    category: "Current Settings".to_string(),
                    title: "Detected PID values".to_string(),
                    description: settings,
                    recommendation: "These are the PID values extracted from the log header."
                        .to_string(),
                    cli_command: None,
                    severity: Severity::Info,
                },
            );
        }

        suggestions
    }

    fn get_current_settings(fd: &FlightData) -> Option<String> {
        let headers = &fd.unknown_headers;

        // Helper to get value or "N/A"
        let get_or_na = |keys: &[&str]| -> String {
            for key in keys {
                if let Some(v) = headers.get(*key) {
                    return v.clone();
                }
            }
            "N/A".to_string()
        };

        // Format anti-gravity with decimal (80 -> 8.0)
        let format_anti_gravity = || -> String {
            headers
                .get("anti_gravity_gain")
                .and_then(|s| s.parse::<f32>().ok())
                .map(|v| format!("{:.1}", v / 10.0))
                .unwrap_or_else(|| "N/A".to_string())
        };

        // ========== PIDs ==========
        let roll_pid = get_or_na(&["rollPID"]);
        let pitch_pid = get_or_na(&["pitchPID"]);
        let yaw_pid = get_or_na(&["yawPID"]);
        let d_min = get_or_na(&["d_min"]);
        let ff_weight = get_or_na(&["ff_weight"]);

        // ========== PID Controller Settings ==========
        let ff_jitter = get_or_na(&["feedforward_jitter_factor"]);
        let ff_smooth = get_or_na(&["feedforward_smooth_factor"]);
        let ff_averaging = get_or_na(&["feedforward_averaging"]);
        let ff_boost = get_or_na(&["feedforward_boost"]);
        let ff_max_rate = get_or_na(&["feedforward_max_rate_limit"]);
        let ff_transition = get_or_na(&["feedforward_transition"]);

        let iterm_relax = get_or_na(&["iterm_relax"]);
        let iterm_relax_type = get_or_na(&["iterm_relax_type"]);
        let iterm_relax_cutoff = get_or_na(&["iterm_relax_cutoff"]);
        let iterm_rotation = get_or_na(&["iterm_rotation"]);

        let anti_gravity = format_anti_gravity();
        let d_max_gain = get_or_na(&["d_max_gain"]);
        let d_max_advance = get_or_na(&["d_max_advance"]);

        let tpa_mode = get_or_na(&["tpa_mode"]);
        let tpa_rate = get_or_na(&["tpa_rate"]);
        let tpa_breakpoint = get_or_na(&["tpa_breakpoint"]);

        let throttle_boost = get_or_na(&["throttle_boost"]);
        let motor_limit = get_or_na(&["motor_output_limit"]);
        let dyn_idle = get_or_na(&["dyn_idle_min_rpm"]);
        let vbat_sag = get_or_na(&["vbat_sag_compensation"]);
        let thrust_linear = get_or_na(&["thrust_linear"]);

        let integrated_yaw = get_or_na(&["use_integrated_yaw"]);
        let abs_control = get_or_na(&["abs_control_gain"]);

        // ========== Rates ==========
        let rates_type = get_or_na(&["rates_type"]);
        let rc_rates = get_or_na(&["rc_rates"]);
        let rates = get_or_na(&["rates"]);
        let rc_expo = get_or_na(&["rc_expo"]);
        let throttle_limit = get_or_na(&["throttle_limit_percent"]);

        // ========== Gyro Filters ==========
        let gyro_lpf1_static = get_or_na(&["gyro_lpf1_static_hz"]);
        let gyro_lpf1_dyn_min = get_or_na(&["gyro_lpf1_dyn_min_hz"]);
        let gyro_lpf1_dyn_max = get_or_na(&["gyro_lpf1_dyn_max_hz"]);
        let gyro_lpf2_static = get_or_na(&["gyro_lpf2_static_hz"]);
        let gyro_lpf2_type = get_or_na(&["gyro_lpf2_type"]);

        // ========== D-Term Filters ==========
        let dterm_lpf1_static = get_or_na(&["dterm_lpf1_static_hz"]);
        let dterm_lpf1_dyn_min = get_or_na(&["dterm_lpf1_dyn_min_hz"]);
        let dterm_lpf1_dyn_max = get_or_na(&["dterm_lpf1_dyn_max_hz"]);
        let dterm_lpf2_static = get_or_na(&["dterm_lpf2_static_hz"]);
        let dterm_lpf2_type = get_or_na(&["dterm_lpf2_type"]);
        let yaw_lpf = get_or_na(&["yaw_lowpass_hz"]);

        // ========== Dynamic Notch ==========
        let dyn_notch_count = get_or_na(&["dyn_notch_count"]);
        let dyn_notch_q = get_or_na(&["dyn_notch_q"]);
        let dyn_notch_min = get_or_na(&["dyn_notch_min_hz"]);
        let dyn_notch_max = get_or_na(&["dyn_notch_max_hz"]);

        // ========== RPM Filter ==========
        let rpm_harmonics = get_or_na(&["rpm_filter_harmonics"]);
        let rpm_min_hz = get_or_na(&["rpm_filter_min_hz"]);

        // ========== Filter Multipliers ==========
        let gyro_filter_mult = get_or_na(&["simplified_gyro_filter_multiplier"]);
        let dterm_filter_mult = get_or_na(&["simplified_dterm_filter_multiplier"]);

        if roll_pid != "N/A" || pitch_pid != "N/A" {
            Some(format!(
                "══════════ PID SETTINGS ══════════\n\
                 Roll PID: {}  |  Pitch PID: {}  |  Yaw PID: {}\n\
                 D-Min: {}  |  FF Weight: {}\n\n\
                 ══════════ FEEDFORWARD ══════════\n\
                 Jitter: {} | Smooth: {} | Averaging: {} | Boost: {}\n\
                 Max Rate Limit: {} | Transition: {}\n\n\
                 ══════════ I-TERM ══════════\n\
                 Relax: {} (Type={}, Cutoff={}) | Rotation: {}\n\n\
                 ══════════ D-MAX & ANTI-GRAVITY ══════════\n\
                 Anti-Gravity Gain: {} | D-Max Gain: {} | D-Max Advance: {}\n\n\
                 ══════════ TPA ══════════\n\
                 Mode: {} | Rate: {}% | Breakpoint: {}µs\n\n\
                 ══════════ MOTOR & THROTTLE ══════════\n\
                 Throttle Boost: {} | Motor Limit: {}% | Dynamic Idle: {} RPM\n\
                 VBat Sag: {}% | Thrust Linear: {}%\n\
                 Integrated Yaw: {} | Abs Control: {}\n\n\
                 ══════════ RATES ══════════\n\
                 Type: {} | RC Rates: {} | Rates: {} | Expo: {}\n\
                 Throttle Limit: {}%\n\n\
                 ══════════ GYRO FILTERS ══════════\n\
                 LPF1 Static: {}Hz | Dynamic: {}-{}Hz\n\
                 LPF2 Static: {}Hz (Type={})\n\
                 Filter Multiplier: {}\n\n\
                 ══════════ D-TERM FILTERS ══════════\n\
                 LPF1 Static: {}Hz | Dynamic: {}-{}Hz\n\
                 LPF2 Static: {}Hz (Type={}) | Yaw LPF: {}Hz\n\
                 Filter Multiplier: {}\n\n\
                 ══════════ NOTCH FILTERS ══════════\n\
                 Dynamic Notch: Count={}, Q={}, Range={}-{}Hz\n\
                 RPM Filter: {} harmonics, Min={}Hz",
                roll_pid,
                pitch_pid,
                yaw_pid,
                d_min,
                ff_weight,
                ff_jitter,
                ff_smooth,
                ff_averaging,
                ff_boost,
                ff_max_rate,
                ff_transition,
                iterm_relax,
                iterm_relax_type,
                iterm_relax_cutoff,
                iterm_rotation,
                anti_gravity,
                d_max_gain,
                d_max_advance,
                tpa_mode,
                tpa_rate,
                tpa_breakpoint,
                throttle_boost,
                motor_limit,
                dyn_idle,
                vbat_sag,
                thrust_linear,
                integrated_yaw,
                abs_control,
                rates_type,
                rc_rates,
                rates,
                rc_expo,
                throttle_limit,
                gyro_lpf1_static,
                gyro_lpf1_dyn_min,
                gyro_lpf1_dyn_max,
                gyro_lpf2_static,
                gyro_lpf2_type,
                gyro_filter_mult,
                dterm_lpf1_static,
                dterm_lpf1_dyn_min,
                dterm_lpf1_dyn_max,
                dterm_lpf2_static,
                dterm_lpf2_type,
                yaw_lpf,
                dterm_filter_mult,
                dyn_notch_count,
                dyn_notch_q,
                dyn_notch_min,
                dyn_notch_max,
                rpm_harmonics,
                rpm_min_hz
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
                        // Track successful AI analysis
                        if let Some(ref model_id) = self.selected_model_id {
                            analytics::log_ai_analysis_completed(model_id);
                        }
                    }
                    AIAnalysisResult::Error(err) => {
                        // Track failed AI analysis
                        analytics::log_ai_analysis_failed(&err);
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
                        ui.add(
                            egui::TextEdit::singleline(&mut self.api_key)
                                .password(true)
                                .hint_text("sk-or-...")
                                .desired_width(300.0),
                        );
                    });

                    // Model filter and selection
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        ui.label("🔍 Filter:");
                        ui.add(
                            egui::TextEdit::singleline(&mut self.model_filter)
                                .hint_text("Type to filter models...")
                                .desired_width(200.0),
                        );

                        if self.models_loading {
                            ui.spinner();
                            ui.label("Loading models...");
                        } else {
                            ui.label(format!("{} models available", self.models.len()));
                        }
                    });

                    // Filter models by name (case-insensitive contains)
                    let filter_lower = self.model_filter.to_lowercase();
                    let filtered_models: Vec<&AIModel> = self
                        .models
                        .iter()
                        .filter(|m| {
                            if filter_lower.is_empty() {
                                true
                            } else {
                                m.name.to_lowercase().contains(&filter_lower)
                                    || m.id.to_lowercase().contains(&filter_lower)
                            }
                        })
                        .collect();

                    // Get selected model name for display
                    let selected_name = self
                        .selected_model_id
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
                                            let is_selected =
                                                self.selected_model_id.as_ref() == Some(&model.id);
                                            if ui
                                                .selectable_label(is_selected, &model.name)
                                                .clicked()
                                            {
                                                self.selected_model_id = Some(model.id.clone());
                                            }
                                        }

                                        if filtered_models.is_empty() {
                                            ui.label("No models match filter");
                                        }
                                    });
                            });

                        // Save Settings button
                        if ui
                            .button("💾 Save Settings")
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
                            analytics::log_ai_settings_saved();
                            log::info!("AI settings saved");
                        }
                    });

                    ui.add_space(8.0);

                    // Analysis focus dropdown
                    ui.horizontal(|ui| {
                        ui.label("Analysis Focus:");
                        egui::ComboBox::from_id_source("analysis_focus")
                            .selected_text(format!("{}", self.analysis_focus))
                            .width(180.0)
                            .show_ui(ui, |ui| {
                                for focus in AnalysisFocus::ALL {
                                    ui.selectable_value(
                                        &mut self.analysis_focus,
                                        focus,
                                        format!("{}", focus),
                                    );
                                }
                            });
                    });

                    ui.add_space(8.0);

                    ui.horizontal(|ui| {
                        let can_analyze = !self.api_key.is_empty()
                            && !self.ai_loading
                            && self.selected_model_id.is_some();

                        if ui
                            .add_enabled(can_analyze, egui::Button::new("🚀 Analyze with AI"))
                            .clicked()
                        {
                            // Build metrics from analysis
                            let metrics = self.build_flight_metrics();
                            let model_id = self.selected_model_id.clone().unwrap_or_default();
                            let api_key = self.api_key.clone();

                            // Track AI analysis started
                            analytics::log_ai_analysis_started(&model_id, &format!("{}", self.analysis_focus));

                            // Start async analysis
                            self.ai_loading = true;
                            self.ai_error = None;
                            self.ai_receiver =
                                Some(OpenRouterClient::analyze_async(api_key, model_id, metrics));
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
                    let mut clear_ai_response = false;
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
                                                    ui.monospace(
                                                        RichText::new(cmd)
                                                            .color(Color32::LIGHT_GREEN),
                                                    );
                                                    if ui
                                                        .small_button("📋")
                                                        .on_hover_text("Copy")
                                                        .clicked()
                                                    {
                                                        ui.output_mut(|o| {
                                                            o.copied_text = cmd.to_string()
                                                        });
                                                    }
                                                });
                                            } else if line.starts_with("- ")
                                                || line.starts_with("* ")
                                            {
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
                                analytics::log_ai_response_copied();
                            }
                            if ui.button("🗑 Clear").clicked() {
                                clear_ai_response = true;
                            }
                        });
                    }
                    if clear_ai_response {
                        self.ai_response = None;
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
                if ui
                    .button("📋 Copy All CLI Commands")
                    .on_hover_text("Copy all CLI commands to clipboard (Critical → Warning → Info)")
                    .clicked()
                {
                    let all_commands = self.generate_cli_export();
                    if !all_commands.is_empty() {
                        let cmd_count = self.suggestions.iter().filter(|s| s.cli_command.is_some()).count();
                        analytics::log_cli_commands_copied(cmd_count, None);
                        ui.output_mut(|o| o.copied_text = all_commands);
                    }
                }

                // Copy Critical Only button
                let critical_count = self
                    .suggestions
                    .iter()
                    .filter(|s| s.severity == Severity::Critical && s.cli_command.is_some())
                    .count();
                if critical_count > 0 {
                    if ui
                        .button(format!("🔴 Copy Critical Only ({})", critical_count))
                        .on_hover_text("Copy only critical priority commands")
                        .clicked()
                    {
                        let commands = self.generate_cli_export_filtered(Some(Severity::Critical));
                        analytics::log_cli_commands_copied(critical_count, Some("critical"));
                        ui.output_mut(|o| o.copied_text = commands);
                    }
                }

                // Count how many have CLI commands
                let cmd_count = self
                    .suggestions
                    .iter()
                    .filter(|s| s.cli_command.is_some())
                    .count();
                let warning_count = self
                    .suggestions
                    .iter()
                    .filter(|s| s.severity == Severity::Warning && s.cli_command.is_some())
                    .count();
                ui.label(
                    RichText::new(format!(
                        "{} total ({} critical, {} warning)",
                        cmd_count, critical_count, warning_count
                    ))
                    .weak(),
                );
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
                                        ui.monospace(
                                            RichText::new(&preview)
                                                .size(11.0)
                                                .color(egui::Color32::LIGHT_GREEN),
                                        );
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
                                ui.style_mut().visuals.extreme_bg_color =
                                    egui::Color32::from_black_alpha(100);
                                egui::Frame::group(ui.style())
                                    .rounding(4.0)
                                    .inner_margin(8.0)
                                    .stroke(egui::Stroke::new(1.0, egui::Color32::from_gray(60)))
                                    .show(ui, |ui| {
                                        ui.horizontal(|ui| {
                                            ui.monospace(
                                                RichText::new(cmd)
                                                    .color(egui::Color32::LIGHT_GREEN),
                                            );
                                            ui.with_layout(
                                                egui::Layout::right_to_left(egui::Align::Center),
                                                |ui| {
                                                    if ui
                                                        .button("📋 Copy")
                                                        .on_hover_text("Copy to clipboard")
                                                        .clicked()
                                                    {
                                                        ui.output_mut(|o| {
                                                            o.copied_text = cmd.clone()
                                                        });
                                                    }
                                                },
                                            );
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
                RichText::new(
                    "⚠️ Disclaimer: These are automated suggestions based on log analysis. \
                               Always test changes carefully and make small adjustments.",
                )
                .small()
                .weak(),
            );
        });
    }

    /// Build FlightMetrics for AI analysis
    fn build_flight_metrics(&self) -> FlightMetrics {
        let headers = &self.fd.unknown_headers;

        // Parse PIDs from headers (format: "P,I,D")
        let parse_pid_3 = |key: &str| -> [f32; 3] {
            headers
                .get(key)
                .map(|s| {
                    let parts: Vec<f32> =
                        s.split(',').filter_map(|v| v.trim().parse().ok()).collect();
                    [
                        parts.first().copied().unwrap_or(0.0),
                        parts.get(1).copied().unwrap_or(0.0),
                        parts.get(2).copied().unwrap_or(0.0),
                    ]
                })
                .unwrap_or([0.0; 3])
        };

        // Parse ff_weight (format: "roll,pitch,yaw" e.g., "100,105,80")
        let ff_weights: Vec<f32> = headers
            .get("ff_weight")
            .map(|s| s.split(',').filter_map(|v| v.trim().parse().ok()).collect())
            .unwrap_or_default();

        // Combine PID + FF for the 4-element array expected by FlightMetrics
        let roll_pid_3 = parse_pid_3("rollPID");
        let pitch_pid_3 = parse_pid_3("pitchPID");
        let yaw_pid_3 = parse_pid_3("yawPID");

        let roll_pid = [
            roll_pid_3[0],
            roll_pid_3[1],
            roll_pid_3[2],
            ff_weights.first().copied().unwrap_or(0.0),
        ];
        let pitch_pid = [
            pitch_pid_3[0],
            pitch_pid_3[1],
            pitch_pid_3[2],
            ff_weights.get(1).copied().unwrap_or(0.0),
        ];
        let yaw_pid = [
            yaw_pid_3[0],
            yaw_pid_3[1],
            yaw_pid_3[2],
            ff_weights.get(2).copied().unwrap_or(0.0),
        ];

        // Parse D-min values from d_min header (format: "50,65,0" for roll,pitch,yaw)
        let d_min_values: Vec<f32> = headers
            .get("d_min")
            .map(|s| s.split(',').filter_map(|v| v.trim().parse().ok()).collect())
            .unwrap_or_default();

        let duration = self.fd.times.last().copied().unwrap_or(0.0);

        // Helper to parse u32 from header
        let parse_u32 =
            |key: &str| -> Option<u32> { headers.get(key).and_then(|s| s.parse().ok()) };

        // Helper to parse string enums
        let parse_iterm_relax_type = |val: &str| -> String {
            match val {
                "0" => "Gyro".to_string(),
                "1" => "Setpoint".to_string(),
                _ => val.to_string(),
            }
        };

        let parse_ff_averaging = |val: &str| -> String {
            match val {
                "0" => "OFF".to_string(),
                "1" => "2 Point".to_string(),
                "2" => "3 Point".to_string(),
                "3" => "4 Point".to_string(),
                _ => val.to_string(),
            }
        };

        let parse_tpa_mode = |val: &str| -> String {
            match val {
                "0" => "OFF".to_string(),
                "1" => "D".to_string(),
                "2" => "PD".to_string(),
                _ => val.to_string(),
            }
        };

        FlightMetrics {
            firmware: format!("{} {}", self.fd.firmware.name(), self.fd.firmware.version()),
            craft_name: self
                .fd
                .craft_name
                .clone()
                .unwrap_or_else(|| "Unknown".to_string()),
            duration_sec: duration,
            roll_pid,
            pitch_pid,
            yaw_pid,
            d_min: [
                d_min_values.first().copied().unwrap_or(0.0),
                d_min_values.get(1).copied().unwrap_or(0.0),
                d_min_values.get(2).copied().unwrap_or(0.0),
            ],
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
            dterm_noise_rms: [
                self.analysis.dterm_noise_rms[0],
                self.analysis.dterm_noise_rms[1],
                0.0,
            ],
            tracking_error_rms: self.analysis.tracking_error_rms,
            motor_saturation_pct: self.analysis.motor_saturation_pct,
            gyro_lpf_hz: headers
                .get("gyro_lpf1_static_hz")
                .or_else(|| headers.get("gyro_lowpass_hz"))
                .and_then(|s| s.parse().ok()),
            gyro_lpf2_hz: headers
                .get("gyro_lpf2_static_hz")
                .and_then(|s| s.parse().ok()),
            dterm_lpf_hz: headers
                .get("dterm_lpf1_static_hz")
                .or_else(|| headers.get("dterm_lowpass_hz"))
                .and_then(|s| s.parse().ok()),
            dterm_lpf2_hz: headers
                .get("dterm_lpf2_static_hz")
                .and_then(|s| s.parse().ok()),
            yaw_lpf_hz: headers
                .get("yaw_lpf_hz")
                .or_else(|| headers.get("yaw_lowpass_hz"))
                .and_then(|s| s.parse().ok()),
            dyn_notch_count: headers.get("dyn_notch_count").and_then(|s| s.parse().ok()),
            dyn_notch_min: headers.get("dyn_notch_min_hz").and_then(|s| s.parse().ok()),
            dyn_notch_max: headers.get("dyn_notch_max_hz").and_then(|s| s.parse().ok()),
            rpm_filter_harmonics: headers
                .get("rpm_filter_harmonics")
                .and_then(|s| s.parse().ok()),

            // Rate settings (parse from comma-separated values like rc_rates:3,3,1)
            rc_rate: {
                headers.get("rc_rates").map(|s| {
                    let parts: Vec<f32> = s.split(',').filter_map(|v| v.parse().ok()).collect();
                    [
                        parts.first().copied().unwrap_or(1.0),
                        parts.get(1).copied().unwrap_or(1.0),
                        parts.get(2).copied().unwrap_or(1.0),
                    ]
                })
            },
            rc_expo: {
                headers.get("rc_expo").map(|s| {
                    let parts: Vec<f32> = s.split(',').filter_map(|v| v.parse().ok()).collect();
                    [
                        parts.first().copied().unwrap_or(0.0),
                        parts.get(1).copied().unwrap_or(0.0),
                        parts.get(2).copied().unwrap_or(0.0),
                    ]
                })
            },
            super_rate: {
                headers.get("rates").map(|s| {
                    let parts: Vec<f32> = s.split(',').filter_map(|v| v.parse().ok()).collect();
                    [
                        parts.first().copied().unwrap_or(0.0),
                        parts.get(1).copied().unwrap_or(0.0),
                        parts.get(2).copied().unwrap_or(0.0),
                    ]
                })
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

            // ===== PID Controller Settings =====

            // Feedforward settings
            feedforward_jitter_reduction: parse_u32("feedforward_jitter_factor"),
            feedforward_smoothness: parse_u32("feedforward_smooth_factor"),
            feedforward_averaging: headers
                .get("feedforward_averaging")
                .map(|s| parse_ff_averaging(s)),
            feedforward_boost: parse_u32("feedforward_boost"),
            feedforward_max_rate_limit: parse_u32("feedforward_max_rate_limit"),
            feedforward_transition: parse_u32("feedforward_transition"),

            // I-Term settings
            iterm_relax_enabled: headers.get("iterm_relax").map(|s| s != "0").unwrap_or(true),
            iterm_relax_type: headers
                .get("iterm_relax_type")
                .map(|s| parse_iterm_relax_type(s)),
            iterm_relax_cutoff: parse_u32("iterm_relax_cutoff"),
            iterm_rotation: headers
                .get("iterm_rotation")
                .map(|s| s != "0")
                .unwrap_or(false),

            // Anti-Gravity
            anti_gravity_gain: parse_u32("anti_gravity_gain"),

            // Dynamic Damping (D-Max)
            d_max_gain: parse_u32("d_max_gain"),
            d_max_advance: parse_u32("d_max_advance"),

            // Throttle and Motor Settings
            throttle_boost: parse_u32("throttle_boost"),
            motor_output_limit: parse_u32("motor_output_limit"),
            dyn_idle_min_rpm: parse_u32("dyn_idle_min_rpm"),
            vbat_sag_compensation: parse_u32("vbat_sag_compensation"),
            thrust_linear: parse_u32("thrust_linear"),

            // TPA
            tpa_mode: headers.get("tpa_mode").map(|s| parse_tpa_mode(s)),
            tpa_rate: parse_u32("tpa_rate"),
            tpa_breakpoint: parse_u32("tpa_breakpoint"),

            // Misc
            integrated_yaw: headers
                .get("use_integrated_yaw")
                .map(|s| s != "0")
                .unwrap_or(false),
            abs_control_gain: parse_u32("abs_control_gain"),

            // Analysis focus
            analysis_focus: self.analysis_focus,
        }
    }

    /// Generate CLI export string with all commands grouped by severity
    fn generate_cli_export(&self) -> String {
        self.generate_cli_export_filtered(None)
    }

    /// Generate CLI export string, optionally filtered by severity
    fn generate_cli_export_filtered(&self, severity_filter: Option<Severity>) -> String {
        let mut output = String::from("# PID Lab Tuning Suggestions\n");
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

            let severity_suggestions: Vec<_> = self
                .suggestions
                .iter()
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

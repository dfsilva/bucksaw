use std::sync::mpsc::Receiver;
use std::sync::Arc;

use egui::{Color32, RichText, Ui};
use egui_phosphor::regular as icons;

use crate::ai_integration::{
    AIAnalysisResult, AIModel, AnalysisFocus, FlightMetrics, ModelFetchResult, OpenRouterClient,
    DEFAULT_MODELS,
};
use crate::analytics;
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
            Severity::Info => icons::INFO,
            Severity::Warning => icons::WARNING,
            Severity::Critical => icons::X_CIRCLE,
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

    // Throttle-segmented PID analysis [axis][throttle_band: 0=low, 1=mid, 2=high]
    tracking_error_by_throttle: [[f32; 3]; 3], // Per-axis, per-throttle-band tracking error
    worst_throttle_band: usize, // Which throttle range has worst tracking (0=low, 1=mid, 2=high)

    // Propwash analysis
    propwash_severity: f32,        // 0-100 score, higher = worse propwash
    propwash_events: usize,        // Number of detected propwash events
    propwash_worst_axis: usize,    // 0=Roll, 1=Pitch, 2=Yaw - most affected axis
    propwash_avg_oscillation: f32, // Average oscillation amplitude during events

    // Lag analysis
    system_latency_ms: [f32; 3], // Per-axis latency from setpoint to gyro response
    avg_latency_ms: f32,         // Average system latency

    // I-term analysis
    iterm_windup_events: usize, // Count of I-term saturation events
    iterm_drift: [f32; 3],      // I-term slow drift per axis

    // Motor balance analysis
    motor_avg_output: [f32; 4],   // Per-motor average output percentage
    motor_imbalance_pct: f32,     // Overall imbalance percentage (0=perfect, >10=concerning)
    motor_imbalance_pair: String, // Which motor pair shows most imbalance ("Front-Back" or "Left-Right")
    motor_worst_idx: usize,       // Motor index with highest average (0-3)
    motor_differential: f32,      // Max - Min motor difference

    // Feedforward analysis
    ff_effectiveness: f32, // 0-100 score: how well FF anticipates setpoint changes
    setpoint_jitter: f32,  // Average high-freq noise in setpoint (indicates RC link issues)
    max_stick_rate: [f32; 3], // Maximum observed stick rate per axis (deg/s²)
    ff_delay_compensation: f32, // Estimated delay being compensated by FF (ms)
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

        // === THROTTLE-SEGMENTED TRACKING ERROR ===
        // Compute tracking error (setpoint - gyro) for each axis at each throttle band
        let mut tracking_error_by_throttle = [[0.0f32; 3]; 3];
        let mut throttle_band_sample_counts = [[0usize; 3]; 3];

        if let (Some(setpoint), Some(gyro)) = (fd.setpoint(), fd.gyro_filtered()) {
            let throttle = &setpoint[3]; // Throttle is 4th channel

            for axis in 0..3 {
                for i in 0..setpoint[axis]
                    .len()
                    .min(gyro[axis].len())
                    .min(throttle.len())
                {
                    // Determine throttle band (0=low <30%, 1=mid 30-70%, 2=high >70%)
                    let thr_pct = (throttle[i] / 1000.0) * 100.0; // Assuming 0-1000 scale
                    let band = if thr_pct < 30.0 {
                        0
                    } else if thr_pct < 70.0 {
                        1
                    } else {
                        2
                    };

                    let error = (setpoint[axis][i] - gyro[axis][i]).abs();
                    tracking_error_by_throttle[axis][band] += error * error; // Sum of squares
                    throttle_band_sample_counts[axis][band] += 1;
                }
            }

            // Convert to RMS
            for axis in 0..3 {
                for band in 0..3 {
                    if throttle_band_sample_counts[axis][band] > 0 {
                        tracking_error_by_throttle[axis][band] = (tracking_error_by_throttle[axis]
                            [band]
                            / throttle_band_sample_counts[axis][band] as f32)
                            .sqrt();
                    }
                }
            }
        }

        // Find worst throttle band (highest average error across axes)
        let band_errors: Vec<f32> = (0..3)
            .map(|band| {
                (tracking_error_by_throttle[0][band]
                    + tracking_error_by_throttle[1][band]
                    + tracking_error_by_throttle[2][band])
                    / 3.0
            })
            .collect();
        let worst_throttle_band = band_errors
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(1);

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

        // === PROPWASH DETECTION ===
        // Propwash occurs when drone descends through its own turbulent air
        // Signature: rapid throttle reduction followed by oscillations
        let mut propwash_events = 0usize;
        let mut propwash_oscillation_sum = 0.0f32;
        let mut propwash_axis_counts = [0usize; 3];

        if let Some(motors) = fd.motor() {
            if motors.len() >= 4 && !motors[0].is_empty() {
                let window_size = (sample_rate * 0.1) as usize; // 100ms window
                let gyro = fd.gyro_filtered();

                for i in window_size..throttle_values.len().saturating_sub(window_size) {
                    // Detect rapid throttle drop (>20% reduction in 100ms)
                    let throttle_before = throttle_values[i.saturating_sub(window_size)];
                    let throttle_now = throttle_values[i];
                    let throttle_drop = throttle_before - throttle_now;

                    if throttle_drop > 20.0 {
                        // Check for oscillations in the following window
                        if let Some(gyro_data) = gyro {
                            for axis in 0..3 {
                                if i + window_size < gyro_data[axis].len() {
                                    // Calculate oscillation amplitude in post-drop window
                                    let post_window: Vec<f32> =
                                        gyro_data[axis][i..i + window_size].to_vec();
                                    if post_window.len() > 2 {
                                        let mean: f32 = post_window.iter().sum::<f32>()
                                            / post_window.len() as f32;
                                        let oscillation: f32 = post_window
                                            .iter()
                                            .map(|v| (v - mean).abs())
                                            .sum::<f32>()
                                            / post_window.len() as f32;

                                        // If oscillation exceeds threshold, count as propwash event
                                        if oscillation > 15.0 {
                                            propwash_events += 1;
                                            propwash_oscillation_sum += oscillation;
                                            propwash_axis_counts[axis] += 1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Calculate propwash severity score (0-100)
        let propwash_avg_oscillation = if propwash_events > 0 {
            propwash_oscillation_sum / propwash_events as f32
        } else {
            0.0
        };

        let propwash_worst_axis = propwash_axis_counts
            .iter()
            .enumerate()
            .max_by_key(|(_, count)| *count)
            .map(|(i, _)| i)
            .unwrap_or(0);

        // Severity: events per minute * average oscillation amplitude, capped at 100
        let events_per_minute = if flight_duration > 0.0 {
            propwash_events as f32 / (flight_duration as f32 / 60.0)
        } else {
            0.0
        };
        let propwash_severity = (events_per_minute * propwash_avg_oscillation / 10.0).min(100.0);

        // === LAG ANALYSIS ===
        // Use normalized cross-correlation to find delay between setpoint and gyro
        let mut system_latency_ms = [0.0f32; 3];
        if let (Some(setpoint), Some(gyro)) = (fd.setpoint(), fd.gyro_filtered()) {
            for axis in 0..3 {
                let len = setpoint[axis].len().min(gyro[axis].len());
                if len > 100 {
                    // Normalized cross-correlation for amplitude-independent lag estimation
                    let max_lag_samples = (sample_rate * 0.05) as usize; // Max 50ms
                    let mut best_lag = 0;
                    let mut best_correlation = f32::MIN;

                    // Pre-compute setpoint energy for normalization
                    let setpoint_energy: f32 = setpoint[axis].iter().map(|x| x * x).sum();

                    for lag in 0..max_lag_samples.min(len / 4) {
                        let test_len = len - lag;
                        let mut correlation = 0.0f32;
                        let mut gyro_energy = 0.0f32;

                        for i in 0..test_len {
                            correlation += setpoint[axis][i] * gyro[axis][i + lag];
                            gyro_energy += gyro[axis][i + lag] * gyro[axis][i + lag];
                        }

                        // Normalize by energy to make correlation amplitude-independent
                        let normalization = (setpoint_energy * gyro_energy).sqrt().max(1e-10);
                        let normalized_corr = correlation / normalization;

                        if normalized_corr > best_correlation {
                            best_correlation = normalized_corr;
                            best_lag = lag;
                        }
                    }

                    system_latency_ms[axis] =
                        (best_lag as f32 / sample_rate as f32 * 1000.0) as f32;
                }
            }
        }
        let avg_latency_ms = system_latency_ms.iter().sum::<f32>() / 3.0;

        // === I-TERM ANALYSIS ===
        // Detect I-term windup (saturated I-term) and drift
        let mut iterm_windup_events = 0usize;
        let mut iterm_drift = [0.0f32; 3];

        if let Some(i_terms) = fd.i() {
            for axis in 0..3 {
                if i_terms[axis].len() > 100 {
                    // I-term windup: detect when I-term magnitude exceeds threshold
                    let i_threshold = 400.0; // Typical max I-term contribution
                    for val in i_terms[axis].iter() {
                        if val.abs() > i_threshold {
                            iterm_windup_events += 1;
                        }
                    }

                    // I-term drift: compare first and last 10% of values
                    let segment_len = i_terms[axis].len() / 10;
                    if segment_len > 0 {
                        let first_avg: f32 =
                            i_terms[axis][..segment_len].iter().sum::<f32>() / segment_len as f32;
                        let last_avg: f32 = i_terms[axis][i_terms[axis].len() - segment_len..]
                            .iter()
                            .sum::<f32>()
                            / segment_len as f32;
                        iterm_drift[axis] = last_avg - first_avg;
                    }
                }
            }
        }

        // === MOTOR BALANCE ANALYSIS ===
        // Analyze motor outputs to detect imbalance (CG issues, bent props, motor problems)
        let mut motor_avg_output = [0.0f32; 4];
        let mut motor_imbalance_pct = 0.0f32;
        let mut motor_imbalance_pair = "None".to_string();
        let mut motor_worst_idx = 0usize;
        let mut motor_differential = 0.0f32;

        if let Some(motors) = fd.motor() {
            if motors.len() >= 4 && !motors[0].is_empty() {
                // Calculate per-motor averages
                for (i, motor_data) in motors.iter().enumerate().take(4) {
                    if !motor_data.is_empty() {
                        let sum: f32 = motor_data.iter().sum();
                        motor_avg_output[i] = sum / motor_data.len() as f32;
                    }
                }

                // Normalize to percentage (assuming 2000 max or 100 for normalized)
                let max_scale = motor_avg_output.iter().cloned().fold(0.0f32, f32::max);
                let scale_factor = if max_scale > 100.0 { 2000.0 } else { 100.0 };
                for avg in motor_avg_output.iter_mut() {
                    *avg = (*avg / scale_factor) * 100.0;
                }

                // Find min/max and differential
                let max_motor = motor_avg_output.iter().cloned().fold(0.0f32, f32::max);
                let min_motor = motor_avg_output.iter().cloned().fold(f32::MAX, f32::min);
                motor_differential = max_motor - min_motor;

                // Find worst (highest) motor
                motor_worst_idx = motor_avg_output
                    .iter()
                    .enumerate()
                    .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
                    .map(|(i, _)| i)
                    .unwrap_or(0);

                // Calculate overall imbalance percentage
                let avg_all = motor_avg_output.iter().sum::<f32>() / 4.0;
                // Use max(1.0) to prevent division issues with very low values
                motor_imbalance_pct = (motor_differential / avg_all.max(1.0)) * 100.0;

                // Identify front-back vs left-right imbalance
                // Standard quad motor order: 0=FR, 1=RR, 2=RL, 3=FL (or similar)
                // Front-back: compare (M0+M3) vs (M1+M2)
                // Left-right: compare (M2+M3) vs (M0+M1)
                let front = motor_avg_output[0] + motor_avg_output[3];
                let back = motor_avg_output[1] + motor_avg_output[2];
                let left = motor_avg_output[2] + motor_avg_output[3];
                let right = motor_avg_output[0] + motor_avg_output[1];

                let fb_diff = (front - back).abs();
                let lr_diff = (left - right).abs();

                if fb_diff > lr_diff && fb_diff > 5.0 {
                    motor_imbalance_pair = if front > back {
                        "Nose Heavy (CG forward)".to_string()
                    } else {
                        "Tail Heavy (CG back)".to_string()
                    };
                } else if lr_diff > 5.0 {
                    motor_imbalance_pair = if left > right {
                        "Right Heavy (CG right)".to_string()
                    } else {
                        "Left Heavy (CG left)".to_string()
                    };
                }
            }
        }

        // === FEEDFORWARD ANALYSIS ===
        // Analyze how well feedforward anticipates stick inputs
        let mut ff_effectiveness = 50.0f32; // Default: average
        let mut setpoint_jitter = 0.0f32;
        let mut max_stick_rate = [0.0f32; 3];
        let mut ff_delay_compensation = 0.0f32;

        if let Some(setpoint) = fd.setpoint() {
            // Calculate setpoint derivative (stick rate) and jitter for each axis
            for axis in 0..3 {
                if setpoint[axis].len() > 10 {
                    // Calculate stick rate (derivative of setpoint)
                    let dt = 1.0 / sample_rate as f32;
                    let mut stick_rates: Vec<f32> = Vec::new();

                    for i in 1..setpoint[axis].len() {
                        let rate = (setpoint[axis][i] - setpoint[axis][i - 1]) / dt;
                        stick_rates.push(rate);
                        if rate.abs() > max_stick_rate[axis] {
                            max_stick_rate[axis] = rate.abs();
                        }
                    }

                    // Jitter: high-frequency noise in setpoint (using simple variance of second derivative)
                    if stick_rates.len() > 10 {
                        let mean: f32 = stick_rates.iter().sum::<f32>() / stick_rates.len() as f32;
                        let variance: f32 =
                            stick_rates.iter().map(|r| (r - mean).powi(2)).sum::<f32>()
                                / stick_rates.len() as f32;
                        setpoint_jitter += variance.sqrt() / 100.0; // Normalize
                    }
                }
            }
            setpoint_jitter /= 3.0; // Average across axes

            // FF effectiveness: compare leading edge of setpoint response
            // If gyro starts moving before or with setpoint changes, FF is effective
            if let Some(gyro) = fd.gyro_filtered() {
                let mut effective_transitions = 0;
                let mut total_transitions = 0;
                let threshold = 50.0; // Deg/s threshold for detecting transitions

                for axis in 0..3 {
                    let len = setpoint[axis].len().min(gyro[axis].len());
                    for i in 5..len.saturating_sub(5) {
                        // Detect setpoint transitions
                        let sp_change =
                            (setpoint[axis][i] - setpoint[axis][i.saturating_sub(3)]).abs();
                        if sp_change > threshold {
                            total_transitions += 1;
                            // Check if gyro is also changing (good FF anticipation)
                            let gyro_change =
                                (gyro[axis][i] - gyro[axis][i.saturating_sub(3)]).abs();
                            if gyro_change > threshold * 0.5 {
                                effective_transitions += 1;
                            }
                        }
                    }
                }

                if total_transitions > 10 {
                    ff_effectiveness =
                        (effective_transitions as f32 / total_transitions as f32) * 100.0;
                }
            }

            // Estimate delay compensation based on step response rise time
            let avg_rise_time = (step_rise_time[0] + step_rise_time[1]) / 2.0;
            ff_delay_compensation = if avg_rise_time > 0.0 {
                avg_rise_time as f32 * 0.5
            } else {
                2.0
            };
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
            // Throttle-segmented tracking
            tracking_error_by_throttle,
            worst_throttle_band,
            // New propwash analysis
            propwash_severity,
            propwash_events,
            propwash_worst_axis,
            propwash_avg_oscillation,
            // New lag analysis
            system_latency_ms,
            avg_latency_ms,
            // New I-term analysis
            iterm_windup_events,
            iterm_drift,
            // Motor balance analysis
            motor_avg_output,
            motor_imbalance_pct,
            motor_imbalance_pair,
            motor_worst_idx,
            motor_differential,
            // Feedforward analysis
            ff_effectiveness,
            setpoint_jitter,
            max_stick_rate,
            ff_delay_compensation,
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

        // === MOTOR BALANCE ANALYSIS ===
        if analysis.motor_imbalance_pct > 5.0 {
            let motor_names = ["FR (1)", "RR (2)", "RL (3)", "FL (4)"];
            let worst_motor_name = motor_names[analysis.motor_worst_idx];

            let (severity, severity_text) = if analysis.motor_imbalance_pct > 20.0 {
                (Severity::Critical, "SEVERE")
            } else if analysis.motor_imbalance_pct > 10.0 {
                (Severity::Warning, "MODERATE")
            } else {
                (Severity::Info, "MILD")
            };

            suggestions.push(TuningSuggestion {
                category: "Motors".to_string(),
                title: format!("{} motor imbalance detected ({:.1}%)", severity_text, analysis.motor_imbalance_pct),
                description: format!(
                    "Motor outputs: M1={:.1}%, M2={:.1}%, M3={:.1}%, M4={:.1}%. {} is working hardest. Diagnosis: {}",
                    analysis.motor_avg_output[0],
                    analysis.motor_avg_output[1],
                    analysis.motor_avg_output[2],
                    analysis.motor_avg_output[3],
                    worst_motor_name,
                    analysis.motor_imbalance_pair
                ),
                recommendation: if analysis.motor_imbalance_pct > 20.0 {
                    format!(
                        "Significant CG or mechanical issue. Check: 1) {} prop for damage/balance, 2) Adjust battery position ({}), 3) Check motor bearings on {}", 
                        worst_motor_name,
                        analysis.motor_imbalance_pair,
                        worst_motor_name
                    )
                } else if analysis.motor_imbalance_pct > 10.0 {
                    format!(
                        "Moderate imbalance suggesting {}. Adjust battery position or check prop balance on motor {}.",
                        analysis.motor_imbalance_pair,
                        analysis.motor_worst_idx + 1
                    )
                } else {
                    "Minor imbalance - likely acceptable. Fine-tune battery position if needed.".to_string()
                },
                cli_command: None, // No CLI fix - physical adjustment needed
                severity,
            });
        }

        // === FEEDFORWARD ANALYSIS ===
        if analysis.ff_effectiveness < 50.0 && analysis.max_stick_rate.iter().any(|&r| r > 500.0) {
            let (severity, severity_text) = if analysis.ff_effectiveness < 30.0 {
                (Severity::Warning, "POOR")
            } else {
                (Severity::Info, "SUBOPTIMAL")
            };

            suggestions.push(TuningSuggestion {
                category: "Feedforward".to_string(),
                title: format!("{} FF tracking (Score: {:.0}/100)", severity_text, analysis.ff_effectiveness),
                description: format!(
                    "Gyro is lagging behind setpoint during fast moves. FF Effectiveness: {:.0}%. Avg Stick Rate: {:.0}deg/s.",
                    analysis.ff_effectiveness,
                    analysis.max_stick_rate.iter().sum::<f32>() / 3.0
                ),
                recommendation: "Feedforward gain is likely too low. 1) Increase FF gain by 15-20%. 2) Increase FF transition if moves feel jumpy.".to_string(),
                cli_command: None,
                severity,
            });
        }

        if analysis.setpoint_jitter > 10.0 {
            suggestions.push(TuningSuggestion {
                category: "RC Link / Feedforward".to_string(),
                title: "High Setpoint Jitter Detected".to_string(),
                description: format!(
                    "RC smoothing may be insufficient or radio link noisy. Jitter score: {:.1} (Normal < 5.0).",
                    analysis.setpoint_jitter
                ),
                recommendation: "High RC jitter causes hot motors and noise. 1) Increase RC Smoothing values. 2) Check radio link quality.".to_string(),
                cli_command: Some("set rc_smoothing_auto_factor = 40\nset rc_smoothing_setpoint_rate = 0".to_string()),
                severity: Severity::Warning,
            });
        }

        // === THROTTLE-BAND PERFORMANCE ANALYSIS ===
        // Suggest if tracking varies significantly across throttle ranges
        let throttle_band_names = ["Low (<30%)", "Mid (30-70%)", "High (>70%)"];
        let worst_band_name = throttle_band_names[analysis.worst_throttle_band];

        // Calculate average error per band
        let avg_error_low = (analysis.tracking_error_by_throttle[0][0]
            + analysis.tracking_error_by_throttle[1][0]
            + analysis.tracking_error_by_throttle[2][0])
            / 3.0;
        let avg_error_mid = (analysis.tracking_error_by_throttle[0][1]
            + analysis.tracking_error_by_throttle[1][1]
            + analysis.tracking_error_by_throttle[2][1])
            / 3.0;
        let avg_error_high = (analysis.tracking_error_by_throttle[0][2]
            + analysis.tracking_error_by_throttle[1][2]
            + analysis.tracking_error_by_throttle[2][2])
            / 3.0;

        let max_error = avg_error_low.max(avg_error_mid).max(avg_error_high);
        let min_error = avg_error_low.min(avg_error_mid).min(avg_error_high);
        let error_variance = if min_error > 0.0 {
            (max_error - min_error) / min_error * 100.0
        } else {
            0.0
        };

        if error_variance > 30.0 && max_error > 10.0 {
            let (severity, severity_text) = if error_variance > 80.0 {
                (Severity::Warning, "SIGNIFICANT")
            } else {
                (Severity::Info, "MODERATE")
            };

            suggestions.push(TuningSuggestion {
                category: "Throttle Performance".to_string(),
                title: format!("{} tracking variation across throttle ({:.0}% difference)", severity_text, error_variance),
                description: format!(
                    "Tracking error varies by throttle: Low={:.1}°/s, Mid={:.1}°/s, High={:.1}°/s. Worst at {} throttle.",
                    avg_error_low, avg_error_mid, avg_error_high, worst_band_name
                ),
                recommendation: match analysis.worst_throttle_band {
                    0 => "Poor tracking at low throttle suggests need for higher Antigravity gain or I-term tuning. Try: set anti_gravity_gain = 100".to_string(),
                    2 => "Poor tracking at high throttle suggests TPA is too aggressive or motors saturating. Try: set tpa_rate = 50".to_string(),
                    _ => "Inconsistent mid-throttle tracking. Check filter settings and D-gain for resonance issues.".to_string(),
                },
                cli_command: match analysis.worst_throttle_band {
                    0 => Some("set anti_gravity_gain = 100".to_string()),
                    2 => Some("set tpa_rate = 50\nset tpa_breakpoint = 1350".to_string()),
                    _ => None,
                },
                severity,
            });
        }

        // === ENHANCED PROPWASH ANALYSIS ===

        let axis_names_lower = ["roll", "pitch", "yaw"];

        if analysis.propwash_severity > 0.0 {
            let worst_axis_name = axis_names[analysis.propwash_worst_axis];

            // Determine severity level based on score
            let (severity, severity_text) = if analysis.propwash_severity > 60.0 {
                (Severity::Critical, "SEVERE")
            } else if analysis.propwash_severity > 30.0 {
                (Severity::Warning, "MODERATE")
            } else {
                (Severity::Info, "MILD")
            };

            suggestions.push(TuningSuggestion {
                category: "Propwash".to_string(),
                title: format!("{} propwash detected ({} events)", severity_text, analysis.propwash_events),
                description: format!(
                    "Propwash severity: {:.0}/100. {} axis most affected. Avg oscillation: {:.1}°/s during events.",
                    analysis.propwash_severity,
                    worst_axis_name,
                    analysis.propwash_avg_oscillation
                ),
                recommendation: if analysis.propwash_severity > 60.0 {
                    format!(
                        "Critical propwash on {} axis. Try: 1) Enable RPM filter if not already, 2) Increase D-gain by 15-20%, 3) Enable Dynamic Idle, 4) Check for bent props.",
                        worst_axis_name
                    )
                } else if analysis.propwash_severity > 30.0 {
                    format!(
                        "Moderate propwash on {} axis. Try: 1) Raise {} D-gain slightly, 2) Enable/tune Dynamic Idle (dyn_idle_min_rpm = 30), 3) Increase feedforward smoothing.",
                        worst_axis_name, axis_names_lower[analysis.propwash_worst_axis]
                    )
                } else {
                    "Mild propwash - may be acceptable. Fine-tune D-gain if desired.".to_string()
                },
                cli_command: if analysis.propwash_severity > 30.0 {
                    Some(format!(
                        "set {}_d = {}\nset dyn_idle_min_rpm = 30\nset feedforward_smooth_factor = 25",
                        axis_names_lower[analysis.propwash_worst_axis],
                        (d_gains[analysis.propwash_worst_axis] * 1.15) as i32
                    ))
                } else {
                    None
                },
                severity,
            });
        }

        // Fallback: also check traditional propwash signature
        if analysis.propwash_events == 0
            && analysis.step_oscillations.iter().any(|&o| o)
            && max_dterm_noise > 60.0
        {
            suggestions.push(TuningSuggestion {
                category: "Propwash".to_string(),
                title: "Possible propwash oscillation (indirect)".to_string(),
                description: "Oscillations detected with high D-term noise. May indicate propwash."
                    .to_string(),
                recommendation:
                    "Try: lower D, raise D-min, enable anti_gravity, use feedforward smoothing."
                        .to_string(),
                cli_command: Some(
                    "set feedforward_smooth_factor = 25\nset feedforward_jitter_factor = 10"
                        .to_string(),
                ),
                severity: Severity::Info,
            });
        }

        // === SYSTEM LATENCY ANALYSIS ===
        if analysis.avg_latency_ms > 5.0 {
            let slowest_axis = analysis
                .system_latency_ms
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(i, _)| i)
                .unwrap_or(0);

            let severity = if analysis.avg_latency_ms > 15.0 {
                Severity::Warning
            } else {
                Severity::Info
            };

            suggestions.push(TuningSuggestion {
                category: "Latency".to_string(),
                title: format!("System latency: {:.1}ms average", analysis.avg_latency_ms),
                description: format!(
                    "Per-axis latency: Roll {:.1}ms, Pitch {:.1}ms, Yaw {:.1}ms. {} axis is slowest.",
                    analysis.system_latency_ms[0],
                    analysis.system_latency_ms[1],
                    analysis.system_latency_ms[2],
                    axis_names[slowest_axis]
                ),
                recommendation: if analysis.avg_latency_ms > 15.0 {
                    "High latency detected. Check: 1) Raise gyro/D-term LPF frequencies if noise permits, 2) Reduce dynamic notch count, 3) Ensure RPM filter is working."
                } else {
                    "Moderate latency. Consider raising filter cutoffs if you want snappier response."
                }.to_string(),
                cli_command: None,
                severity,
            });
        }

        // === I-TERM ANALYSIS ===
        if analysis.iterm_windup_events > 50 {
            suggestions.push(TuningSuggestion {
                category: "I-Term".to_string(),
                title: format!("I-term windup detected ({} events)", analysis.iterm_windup_events),
                description: "I-term is saturating during sustained maneuvers. This can cause delayed recovery.".to_string(),
                recommendation: "Lower I-term limit or reduce I-gain. Consider enabling iterm_relax for aggressive flying.".to_string(),
                cli_command: Some("set iterm_relax = RP\nset iterm_relax_type = SETPOINT".to_string()),
                severity: if analysis.iterm_windup_events > 200 {
                    Severity::Warning
                } else {
                    Severity::Info
                },
            });
        }

        // Check for I-term drift (mechanical issue indicator)
        let max_drift = analysis
            .iterm_drift
            .iter()
            .map(|d| d.abs())
            .fold(0.0f32, f32::max);
        if max_drift > 50.0 {
            let drift_axis = analysis
                .iterm_drift
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| {
                    a.abs()
                        .partial_cmp(&b.abs())
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .map(|(i, _)| i)
                .unwrap_or(0);

            suggestions.push(TuningSuggestion {
                category: "I-Term".to_string(),
                title: format!("I-term drift on {} axis", axis_names[drift_axis]),
                description: format!(
                    "I-term drifted by {:.0} during flight. This often indicates a mechanical issue (unbalanced props, bent motor shaft).",
                    analysis.iterm_drift[drift_axis]
                ),
                recommendation: format!(
                    "Check {} axis for: 1) Unbalanced or damaged prop, 2) Loose motor mount, 3) Bent motor shaft, 4) CG offset.",
                    axis_names[drift_axis]
                ),
                cli_command: None,
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

        // === SIMPLIFIED TUNING DETECTION ===
        let using_simplified = headers
            .get("simplified_pids_mode")
            .map(|v| v != "0" && v.to_lowercase() != "off")
            .unwrap_or(false);

        if using_simplified {
            suggestions.push(TuningSuggestion {
                category: "Simplified Tuning".to_string(),
                title: "Simplified PID Tuning Active".to_string(),
                description: "You're using Betaflight's simplified tuning sliders. Individual PID CLI commands may override them.".to_string(),
                recommendation: "Consider using Master Multiplier slider instead of individual PID adjustments. After CLI changes, run 'simplified_tuning apply' to recalculate.".to_string(),
                cli_command: None,
                severity: Severity::Info,
            });
        }

        // === DYNAMIC NOTCH ANALYSIS ===
        let dyn_notch_count = get_val(&["dyn_notch_count"]).unwrap_or(3.0) as i32;
        let dyn_notch_q = get_val(&["dyn_notch_q"]).unwrap_or(300.0);
        let dyn_notch_min = get_val(&["dyn_notch_min_hz"]).unwrap_or(100.0);
        let dyn_notch_max = get_val(&["dyn_notch_max_hz"]).unwrap_or(600.0);

        // Check if dyn notch count is too low for noisy quad
        if dyn_notch_count < 3 && analysis.gyro_noise_mid_throttle > 8.0 {
            suggestions.push(TuningSuggestion {
                category: "Dynamic Notch".to_string(),
                title: "Dynamic notch count may be too low".to_string(),
                description: format!(
                    "dyn_notch_count = {}. With noise level {:.1}, multiple motor peaks may not be filtered.",
                    dyn_notch_count, analysis.gyro_noise_mid_throttle
                ),
                recommendation: "Increase dynamic notch count for better peak tracking.".to_string(),
                cli_command: Some("set dyn_notch_count = 3".to_string()),
                severity: Severity::Info,
            });
        }

        // Check dyn notch Q (lower Q = wider notch = more latency)
        if dyn_notch_q < 200.0 && analysis.gyro_noise_rms.iter().all(|&n| n < 5.0) {
            suggestions.push(TuningSuggestion {
                category: "Dynamic Notch".to_string(),
                title: "Dynamic notch Q is low".to_string(),
                description: format!(
                    "dyn_notch_q = {:.0}. Low Q means wider notches and more latency. Your noise is low enough to raise it.",
                    dyn_notch_q
                ),
                recommendation: "Raise Q value for narrower, lower-latency notches.".to_string(),
                cli_command: Some("set dyn_notch_q = 350".to_string()),
                severity: Severity::Info,
            });
        }

        // Check dyn notch range for high-KV motors
        if dyn_notch_max < 500.0
            && analysis.gyro_noise_high_throttle > analysis.gyro_noise_mid_throttle * 1.3
        {
            suggestions.push(TuningSuggestion {
                category: "Dynamic Notch".to_string(),
                title: "Dynamic notch max frequency may be too low".to_string(),
                description: format!(
                    "dyn_notch_max = {:.0}Hz. High-throttle noise ({:.1}) exceeds mid-throttle ({:.1}) - motor peaks may exceed current range.",
                    dyn_notch_max, analysis.gyro_noise_high_throttle, analysis.gyro_noise_mid_throttle
                ),
                recommendation: "Raise dyn_notch_max_hz for high-KV motors or high-throttle use.".to_string(),
                cli_command: Some("set dyn_notch_max_hz = 600".to_string()),
                severity: Severity::Info,
            });
        }

        // === RPM FILTER ANALYSIS ===
        let rpm_harmonics = get_val(&["rpm_filter_harmonics"]).unwrap_or(0.0) as i32;
        let rpm_min_hz = get_val(&["rpm_filter_min_hz"]).unwrap_or(100.0);
        let rpm_enabled = rpm_harmonics > 0;

        if !rpm_enabled && analysis.motor_heat_risk > 40.0 {
            suggestions.push(TuningSuggestion {
                category: "RPM Filter".to_string(),
                title: "RPM filter not enabled".to_string(),
                description: format!(
                    "Motor heat risk: {:.0}. RPM filter provides superior motor harmonic rejection with minimal latency.",
                    analysis.motor_heat_risk
                ),
                recommendation: "Enable RPM filter with 3 harmonics (requires bidirectional DShot).".to_string(),
                cli_command: Some("set dshot_bidir = ON\nset rpm_filter_harmonics = 3\nset rpm_filter_min_hz = 100".to_string()),
                severity: Severity::Warning,
            });
        }

        // Check if RPM harmonics can be reduced for lower latency
        if rpm_harmonics > 2 && analysis.gyro_noise_rms.iter().all(|&n| n < 3.0) {
            suggestions.push(TuningSuggestion {
                category: "RPM Filter".to_string(),
                title: "RPM filter may use fewer harmonics".to_string(),
                description: format!(
                    "rpm_filter_harmonics = {}. With low noise levels, 2 harmonics may suffice and reduce latency.",
                    rpm_harmonics
                ),
                recommendation: "Consider reducing to 2 harmonics for lower latency.".to_string(),
                cli_command: Some("set rpm_filter_harmonics = 2".to_string()),
                severity: Severity::Info,
            });
        }

        // === D-MAX ANALYSIS ===
        let d_max_gain = get_val(&["d_max_gain"]).unwrap_or(37.0);
        let d_max_advance = get_val(&["d_max_advance"]).unwrap_or(20.0);

        // D-Max tuning based on step response
        let avg_bounce_back = (analysis.step_bounce_back[0] + analysis.step_bounce_back[1]) / 2.0;
        if avg_bounce_back > 0.12 && d_max_gain < 50.0 {
            suggestions.push(TuningSuggestion {
                category: "D-Max".to_string(),
                title: "Increase D-Max boost for propwash".to_string(),
                description: format!(
                    "Bounce-back: {:.1}%. D-Max gain = {:.0}. D-Max provides additional D during fast movements to counter propwash.",
                    avg_bounce_back * 100.0, d_max_gain
                ),
                recommendation: "Raise D-Max gain for more propwash damping during quick stops.".to_string(),
                cli_command: Some(format!("set d_max_gain = {}", (d_max_gain * 1.25).min(100.0) as i32)),
                severity: Severity::Info,
            });
        }

        // D-Max advance tuning for anticipation
        if analysis.ff_effectiveness < 60.0 && d_max_advance < 30.0 {
            suggestions.push(TuningSuggestion {
                category: "D-Max".to_string(),
                title: "Consider increasing D-Max advance".to_string(),
                description: format!(
                    "FF effectiveness: {:.0}%. D-Max advance = {:.0}. Higher advance uses setpoint to anticipate D-boost earlier.",
                    analysis.ff_effectiveness, d_max_advance
                ),
                recommendation: "Increase D-Max advance for faster D-boost response to stick inputs.".to_string(),
                cli_command: Some("set d_max_advance = 30".to_string()),
                severity: Severity::Info,
            });
        }

        // === ANTI-GRAVITY P-BOOST ANALYSIS ===
        let anti_gravity_gain = get_val(&["anti_gravity_gain"]).unwrap_or(80.0);
        let anti_gravity_p_gain = get_val(&["anti_gravity_p_gain"]).unwrap_or(100.0);
        let anti_gravity_cutoff = get_val(&["anti_gravity_cutoff_hz"]).unwrap_or(5.0);

        // Check if low-throttle tracking is poor (punch-outs/dive recovery)
        let low_throttle_error = (analysis.tracking_error_by_throttle[0][0]
            + analysis.tracking_error_by_throttle[1][0])
            / 2.0;

        if low_throttle_error > 12.0 && analysis.throttle_variance > 15.0 {
            if anti_gravity_p_gain < 120.0 {
                suggestions.push(TuningSuggestion {
                    category: "Anti-Gravity".to_string(),
                    title: "Consider Anti-Gravity P-boost".to_string(),
                    description: format!(
                        "Low-throttle tracking error: {:.1}°/s with throttle variance {:.1}%. AG P-gain ({:.0}%) can help during transitions.",
                        low_throttle_error, analysis.throttle_variance, anti_gravity_p_gain
                    ),
                    recommendation: "Increase anti_gravity_p_gain for better punch-out/dive tracking.".to_string(),
                    cli_command: Some("set anti_gravity_p_gain = 120".to_string()),
                    severity: Severity::Info,
                });
            }

            if anti_gravity_gain < 100.0 {
                suggestions.push(TuningSuggestion {
                    category: "Anti-Gravity".to_string(),
                    title: "Increase Anti-Gravity I-term boost".to_string(),
                    description: format!(
                        "Anti-Gravity gain = {:.0}. With significant throttle changes, higher AG helps I-term during transitions.",
                        anti_gravity_gain
                    ),
                    recommendation: "Raise Anti-Gravity gain for aggressive throttle flying.".to_string(),
                    cli_command: Some("set anti_gravity_gain = 100".to_string()),
                    severity: Severity::Info,
                });
            }
        }

        // === I-TERM RELAX OPTIMIZATION ===
        let iterm_relax = headers
            .get("iterm_relax")
            .map(|s| s.as_str())
            .unwrap_or("RP");
        let iterm_relax_type = headers
            .get("iterm_relax_type")
            .map(|s| s.as_str())
            .unwrap_or("1");
        let iterm_relax_cutoff = get_val(&["iterm_relax_cutoff"]).unwrap_or(15.0);

        if analysis.iterm_windup_events > 80 {
            let suggested_type = if analysis.avg_throttle_pct > 55.0 {
                "SETPOINT"
            } else {
                "GYRO"
            };
            let suggested_cutoff = if analysis.avg_throttle_pct > 55.0 {
                20
            } else {
                12
            };
            let flying_style = if analysis.avg_throttle_pct > 55.0 {
                "aggressive"
            } else {
                "smooth"
            };

            suggestions.push(TuningSuggestion {
                category: "I-Term Relax".to_string(),
                title: format!("Optimize I-term relax for {} flying", flying_style),
                description: format!(
                    "Current: type={}, cutoff={:.0}. Detected {} I-term windup events. Average throttle: {:.0}%.",
                    iterm_relax_type, iterm_relax_cutoff, analysis.iterm_windup_events, analysis.avg_throttle_pct
                ),
                recommendation: format!(
                    "For {} flying style, {} type with cutoff {} works better.",
                    flying_style, suggested_type, suggested_cutoff
                ),
                cli_command: Some(format!(
                    "set iterm_relax_type = {}\nset iterm_relax_cutoff = {}",
                    suggested_type, suggested_cutoff
                )),
                severity: if analysis.iterm_windup_events > 150 { Severity::Warning } else { Severity::Info },
            });
        }

        // Check if I-term relax is disabled for yaw
        if iterm_relax == "RP" && analysis.iterm_drift[2].abs() > 30.0 {
            suggestions.push(TuningSuggestion {
                category: "I-Term Relax".to_string(),
                title: "Enable I-term relax for Yaw".to_string(),
                description: format!(
                    "I-term relax only on RP. Yaw I-term drift: {:.1}. Yaw can benefit from relax during fast spins.",
                    analysis.iterm_drift[2]
                ),
                recommendation: "Enable I-term relax on all axes for less yaw I-term buildup.".to_string(),
                cli_command: Some("set iterm_relax = RPY".to_string()),
                severity: Severity::Info,
            });
        }

        // === TPA ANALYSIS ===
        let tpa_mode = headers.get("tpa_mode").map(|s| s.as_str()).unwrap_or("1");
        let tpa_rate = get_val(&["tpa_rate"]).unwrap_or(65.0);
        let tpa_breakpoint = get_val(&["tpa_breakpoint"]).unwrap_or(1350.0);
        let tpa_low_rate = get_val(&["tpa_low_rate"]).unwrap_or(20.0);
        let tpa_low_breakpoint = get_val(&["tpa_low_breakpoint"]).unwrap_or(1050.0);

        // High-throttle tracking issues - TPA may be too aggressive
        let high_throttle_error = (analysis.tracking_error_by_throttle[0][2]
            + analysis.tracking_error_by_throttle[1][2])
            / 2.0;

        if high_throttle_error > 18.0 && tpa_rate > 60.0 {
            suggestions.push(TuningSuggestion {
                category: "TPA".to_string(),
                title: "TPA may be too aggressive".to_string(),
                description: format!(
                    "High-throttle tracking error: {:.1}°/s. TPA rate = {:.0}% starting at {:.0}µs. PIDs may be cut too much.",
                    high_throttle_error, tpa_rate, tpa_breakpoint
                ),
                recommendation: "Lower TPA rate or raise breakpoint for better high-throttle control.".to_string(),
                cli_command: Some(format!(
                    "set tpa_rate = {}\nset tpa_breakpoint = {}",
                    (tpa_rate * 0.75) as i32, (tpa_breakpoint + 100.0).min(1500.0) as i32
                )),
                severity: Severity::Warning,
            });
        }

        // TPA Low Rate suggestion for low-throttle oscillations
        if analysis.gyro_noise_low_throttle > analysis.gyro_noise_mid_throttle * 1.3
            && tpa_low_rate < 10.0
        {
            suggestions.push(TuningSuggestion {
                category: "TPA".to_string(),
                title: "Consider TPA Low for low-throttle noise".to_string(),
                description: format!(
                    "Low-throttle noise ({:.1}) exceeds mid-throttle ({:.1}). TPA Low can reduce P/D at idle to prevent oscillations.",
                    analysis.gyro_noise_low_throttle, analysis.gyro_noise_mid_throttle
                ),
                recommendation: "Enable TPA Low to reduce gains at low throttle.".to_string(),
                cli_command: Some("set tpa_low_rate = 20\nset tpa_low_breakpoint = 1050".to_string()),
                severity: Severity::Info,
            });
        }

        // === DYNAMIC IDLE ANALYSIS ===
        let dyn_idle_min_rpm = get_val(&["dyn_idle_min_rpm"]).unwrap_or(0.0);
        let dyn_idle_p_gain = get_val(&["dyn_idle_p_gain"]).unwrap_or(50.0);
        let dyn_idle_i_gain = get_val(&["dyn_idle_i_gain"]).unwrap_or(50.0);
        let dyn_idle_d_gain = get_val(&["dyn_idle_d_gain"]).unwrap_or(50.0);

        if analysis.propwash_severity > 25.0 && dyn_idle_min_rpm < 20.0 {
            suggestions.push(TuningSuggestion {
                category: "Dynamic Idle".to_string(),
                title: "Enable Dynamic Idle for propwash".to_string(),
                description: format!(
                    "Propwash severity: {:.0}. Dynamic idle keeps motors spinning during descents, reducing propwash significantly.",
                    analysis.propwash_severity
                ),
                recommendation: "Enable dynamic idle with 25-35 RPM minimum for cleaner descents.".to_string(),
                cli_command: Some(
                    "set dyn_idle_min_rpm = 30\nset dyn_idle_p_gain = 50\nset dyn_idle_i_gain = 50\nset dyn_idle_d_gain = 50".to_string()
                ),
                severity: Severity::Warning,
            });
        } else if dyn_idle_min_rpm > 0.0 && analysis.propwash_severity > 40.0 {
            // Already enabled but still propwash - suggest tuning
            suggestions.push(TuningSuggestion {
                category: "Dynamic Idle".to_string(),
                title: "Tune Dynamic Idle for better propwash".to_string(),
                description: format!(
                    "Dynamic idle enabled at {} RPM but propwash severity still {:.0}. May need higher RPM or gain tuning.",
                    dyn_idle_min_rpm as i32, analysis.propwash_severity
                ),
                recommendation: "Increase minimum RPM or adjust dynamic idle gains.".to_string(),
                cli_command: Some(format!(
                    "set dyn_idle_min_rpm = {}\nset dyn_idle_p_gain = 60\nset dyn_idle_i_gain = 60",
                    (dyn_idle_min_rpm + 10.0).min(50.0) as i32
                )),
                severity: Severity::Info,
            });
        }

        // === FEEDFORWARD JITTER OPTIMIZATION ===
        let ff_jitter = get_val(&["feedforward_jitter_factor"]).unwrap_or(7.0);
        let ff_smooth = get_val(&["feedforward_smooth_factor"]).unwrap_or(25.0);
        let ff_boost = get_val(&["feedforward_boost"]).unwrap_or(15.0);

        if analysis.setpoint_jitter > 8.0 {
            let suggested_jitter = if analysis.setpoint_jitter > 15.0 {
                15
            } else {
                12
            };

            if ff_jitter < suggested_jitter as f32 {
                suggestions.push(TuningSuggestion {
                    category: "Feedforward".to_string(),
                    title: "Increase FF jitter reduction".to_string(),
                    description: format!(
                        "Setpoint jitter: {:.1}. Current ff_jitter_factor = {:.0}. High RC noise causes hot motors and oscillations.",
                        analysis.setpoint_jitter, ff_jitter
                    ),
                    recommendation: "Raise feedforward_jitter_factor to reduce motor heat from RC noise.".to_string(),
                    cli_command: Some(format!("set feedforward_jitter_factor = {}", suggested_jitter)),
                    severity: Severity::Warning,
                });
            }
        }

        // FF Smooth factor optimization
        if analysis.propwash_severity > 30.0 && ff_smooth < 35.0 {
            suggestions.push(TuningSuggestion {
                category: "Feedforward".to_string(),
                title: "Increase FF smoothing for propwash".to_string(),
                description: format!(
                    "Propwash severity: {:.0}. feedforward_smooth_factor = {:.0}. Higher smoothing can reduce propwash oscillations.",
                    analysis.propwash_severity, ff_smooth
                ),
                recommendation: "Increase feedforward smoothing to soften step transitions.".to_string(),
                cli_command: Some("set feedforward_smooth_factor = 45".to_string()),
                severity: Severity::Info,
            });
        }

        // FF Boost suggestion for sluggish response
        let avg_rise_time = (analysis.step_rise_time[0] + analysis.step_rise_time[1]) / 2.0;
        if avg_rise_time > 35.0 && ff_boost < 20.0 {
            suggestions.push(TuningSuggestion {
                category: "Feedforward".to_string(),
                title: "Consider increasing FF boost".to_string(),
                description: format!(
                    "Rise time: {:.1}ms. feedforward_boost = {:.0}. FF boost adds acceleration component for snappier response.",
                    avg_rise_time, ff_boost
                ),
                recommendation: "Increase feedforward_boost for faster initial response.".to_string(),
                cli_command: Some("set feedforward_boost = 20".to_string()),
                severity: Severity::Info,
            });
        }

        // === YAW FEEDFORWARD HOLD ===
        let ff_yaw_hold_gain = get_val(&["feedforward_yaw_hold_gain"]).unwrap_or(15.0);
        let ff_yaw_hold_time = get_val(&["feedforward_yaw_hold_time"]).unwrap_or(100.0);

        // Yaw drift detection
        if analysis.iterm_drift[2].abs() > 25.0 && ff_yaw_hold_gain < 10.0 {
            suggestions.push(TuningSuggestion {
                category: "Yaw Feedforward".to_string(),
                title: "Enable yaw feedforward hold".to_string(),
                description: format!(
                    "Yaw I-term drift: {:.1}. Yaw FF hold helps maintain yaw authority during sustained rotations.",
                    analysis.iterm_drift[2]
                ),
                recommendation: "Enable feedforward_yaw_hold for better sustained yaw control.".to_string(),
                cli_command: Some("set feedforward_yaw_hold_gain = 15\nset feedforward_yaw_hold_time = 100".to_string()),
                severity: Severity::Info,
            });
        }

        // === CRASH RECOVERY & SAFETY ===
        let crash_recovery = headers
            .get("crash_recovery")
            .map(|s| s.as_str())
            .unwrap_or("0");
        let ez_landing_threshold = get_val(&["ez_landing_threshold"]).unwrap_or(25.0);
        let landing_disarm = get_val(&["landing_disarm_threshold"]).unwrap_or(0.0);

        // Suggest crash recovery for high-risk flying
        if crash_recovery == "0" || crash_recovery.to_lowercase() == "off" {
            let high_risk = analysis.motor_saturation_pct > 15.0
                || analysis.gyro_noise_rms.iter().any(|&n| n > 50.0);

            if high_risk {
                suggestions.push(TuningSuggestion {
                    category: "Safety".to_string(),
                    title: "Consider enabling Crash Recovery".to_string(),
                    description: "High motor saturation or noise detected. Crash recovery helps regain control after impacts.".to_string(),
                    recommendation: "Enable crash recovery for safer flying in confined spaces.".to_string(),
                    cli_command: Some(
                        "set crash_recovery = ON\nset crash_recovery_angle = 10\nset crash_recovery_rate = 100".to_string()
                    ),
                    severity: Severity::Info,
                });
            }
        }

        // Landing disarm suggestion for pilots who land frequently
        if landing_disarm < 50.0 && analysis.avg_throttle_pct < 40.0 {
            suggestions.push(TuningSuggestion {
                category: "Safety".to_string(),
                title: "Consider landing disarm threshold".to_string(),
                description: "Low average throttle suggests frequent landing/hovering. Auto-disarm on landing impact can prevent prop injuries.".to_string(),
                recommendation: "Set landing_disarm_threshold for automatic disarm on landing impact.".to_string(),
                cli_command: Some("set landing_disarm_threshold = 100".to_string()),
                severity: Severity::Info,
            });
        }

        // === THRUST LINEARIZATION ===
        let thrust_linear = get_val(&["thrust_linear", "thrustLinearization"]).unwrap_or(0.0);

        // Detect non-linear throttle response (poor tracking at varying throttle)
        let throttle_tracking_variance = (high_throttle_error - low_throttle_error).abs();
        if throttle_tracking_variance > 10.0 && thrust_linear < 20.0 {
            suggestions.push(TuningSuggestion {
                category: "Motor Linearization".to_string(),
                title: "Consider thrust linearization".to_string(),
                description: format!(
                    "Tracking error varies by {:.1}°/s across throttle range. Thrust linearization compensates for motor non-linearity.",
                    throttle_tracking_variance
                ),
                recommendation: "For whoops or brushless with non-linear motors, add thrust linearization.".to_string(),
                cli_command: Some("set thrust_linear = 20".to_string()),
                severity: Severity::Info,
            });
        }

        // === VBAT SAG COMPENSATION ===
        let vbat_sag = get_val(&["vbat_sag_compensation"]).unwrap_or(0.0);

        // If flight is long and tracking degrades at end (heuristic: high variance)
        if flight_duration > 120.0 && vbat_sag < 50.0 && analysis.throttle_variance > 20.0 {
            suggestions.push(TuningSuggestion {
                category: "Battery".to_string(),
                title: "Consider VBat sag compensation".to_string(),
                description: format!(
                    "Flight duration: {:.0}s with high throttle variance. VBat sag compensation maintains consistent feel as battery drains.",
                    flight_duration
                ),
                recommendation: "Enable vbat_sag_compensation for consistent power throughout battery.".to_string(),
                cli_command: Some("set vbat_sag_compensation = 100".to_string()),
                severity: Severity::Info,
            });
        }

        // === ABSOLUTE CONTROL ===
        let abs_control_gain = get_val(&["abs_control_gain"]).unwrap_or(0.0);

        // Suggest abs control for precision flying (low throttle variance = cinematic)
        if analysis.throttle_variance < 10.0 && abs_control_gain < 5.0 {
            suggestions.push(TuningSuggestion {
                category: "Absolute Control".to_string(),
                title: "Consider absolute control for precision".to_string(),
                description: "Smooth flying style detected. Absolute control provides path correction for drift-free cinematic shots.".to_string(),
                recommendation: "Enable abs_control for better position holding during smooth flying.".to_string(),
                cli_command: Some("set abs_control_gain = 10".to_string()),
                severity: Severity::Info,
            });
        }

        // === INTEGRATED YAW ===
        let integrated_yaw = headers
            .get("use_integrated_yaw")
            .map(|s| s != "0")
            .unwrap_or(false);
        let integrated_yaw_relax = get_val(&["integrated_yaw_relax"]).unwrap_or(200.0);

        // Yaw tracking issues with tail-heavy quads
        if !integrated_yaw && analysis.tracking_error_rms[2] > 25.0 {
            suggestions.push(TuningSuggestion {
                category: "Yaw Control".to_string(),
                title: "Consider integrated yaw".to_string(),
                description: format!(
                    "Yaw tracking error: {:.1}°/s. Integrated yaw improves yaw authority, especially on quads with high yaw inertia.",
                    analysis.tracking_error_rms[2]
                ),
                recommendation: "Enable integrated yaw for better yaw response.".to_string(),
                cli_command: Some("set use_integrated_yaw = ON\nset integrated_yaw_relax = 200".to_string()),
                severity: Severity::Info,
            });
        }

        // === MOTOR OUTPUT LIMIT ===
        let motor_output_limit = get_val(&["motor_output_limit"]).unwrap_or(100.0);

        if analysis.motor_saturation_pct > 25.0 && motor_output_limit > 95.0 {
            suggestions.push(TuningSuggestion {
                category: "Motor Limit".to_string(),
                title: "Frequent motor saturation detected".to_string(),
                description: format!(
                    "Motors at 100% for {:.1}% of flight. Consider if your tune is too aggressive or quad is underpowered.",
                    analysis.motor_saturation_pct
                ),
                recommendation: "If intentional (racing), no action needed. Otherwise, lower PID gains or check prop/motor sizing.".to_string(),
                cli_command: None,
                severity: if analysis.motor_saturation_pct > 40.0 { Severity::Warning } else { Severity::Info },
            });
        }

        // === THROTTLE BOOST ===
        let throttle_boost = get_val(&["throttle_boost"]).unwrap_or(5.0);
        let throttle_boost_cutoff = get_val(&["throttle_boost_cutoff"]).unwrap_or(15.0);

        if low_throttle_error > 15.0 && throttle_boost < 8.0 {
            suggestions.push(TuningSuggestion {
                category: "Throttle Boost".to_string(),
                title: "Consider increasing throttle boost".to_string(),
                description: format!(
                    "Low-throttle tracking error: {:.1}°/s. Throttle boost ({:.0}) adds immediate motor response during throttle changes.",
                    low_throttle_error, throttle_boost
                ),
                recommendation: "Increase throttle_boost for snappier throttle response.".to_string(),
                cli_command: Some("set throttle_boost = 10".to_string()),
                severity: Severity::Info,
            });
        }

        // === FILTER TYPE OPTIMIZATION ===
        let dterm_lpf1_type = headers
            .get("dterm_lpf1_type")
            .map(|s| s.as_str())
            .unwrap_or("0");
        let gyro_lpf1_type = headers
            .get("gyro_lpf1_type")
            .map(|s| s.as_str())
            .unwrap_or("0");

        // Suggest PT2/PT3 for better phase response if latency is acceptable
        if dterm_lpf1_type == "0" && analysis.avg_latency_ms < 8.0 && max_dterm_noise > 40.0 {
            suggestions.push(TuningSuggestion {
                category: "Filter Type".to_string(),
                title: "Consider PT2/PT3 D-term filter".to_string(),
                description: format!(
                    "D-term using PT1 filter. D-term noise: {:.1}. PT2/PT3 provides steeper rolloff with similar latency for cleaner D.",
                    max_dterm_noise
                ),
                recommendation: "Try PT2 or PT3 filter type for D-term for better noise rejection.".to_string(),
                cli_command: Some("set dterm_lpf1_type = PT2".to_string()),
                severity: Severity::Info,
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
        let tpa_low_rate = get_or_na(&["tpa_low_rate"]);
        let tpa_low_breakpoint = get_or_na(&["tpa_low_breakpoint"]);

        let anti_gravity_p = get_or_na(&["anti_gravity_p_gain"]);
        let anti_gravity_cutoff = get_or_na(&["anti_gravity_cutoff_hz"]);

        let throttle_boost = get_or_na(&["throttle_boost"]);
        let motor_limit = get_or_na(&["motor_output_limit"]);
        let dyn_idle = get_or_na(&["dyn_idle_min_rpm"]);
        let vbat_sag = get_or_na(&["vbat_sag_compensation"]);
        let thrust_linear = get_or_na(&["thrust_linear"]);

        let integrated_yaw = get_or_na(&["use_integrated_yaw"]);
        let integrated_yaw_relax = get_or_na(&["integrated_yaw_relax"]);
        let abs_control = get_or_na(&["abs_control_gain"]);
        let crash_recovery = get_or_na(&["crash_recovery"]);
        let ez_landing = get_or_na(&["ez_landing_threshold"]);

        // Feedforward Yaw Hold
        let ff_yaw_hold_gain = get_or_na(&["feedforward_yaw_hold_gain"]);
        let ff_yaw_hold_time = get_or_na(&["feedforward_yaw_hold_time"]);

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
                 Max Rate Limit: {} | Transition: {}\n\
                 Yaw Hold: Gain={}, Time={}ms\n\n\
                 ══════════ I-TERM ══════════\n\
                 Relax: {} (Type={}, Cutoff={}) | Rotation: {}\n\n\
                 ══════════ D-MAX & ANTI-GRAVITY ══════════\n\
                 Anti-Gravity: Gain={} | P-Gain={} | Cutoff={}Hz\n\
                 D-Max Gain: {} | D-Max Advance: {}\n\n\
                 ══════════ TPA ══════════\n\
                 Mode: {} | Rate: {}% | Breakpoint: {}µs\n\
                 TPA Low: Rate={}% | Breakpoint={}µs\n\n\
                 ══════════ MOTOR & THROTTLE ══════════\n\
                 Throttle Boost: {} | Motor Limit: {}% | Dynamic Idle: {} RPM\n\
                 VBat Sag: {}% | Thrust Linear: {}%\n\
                 Integrated Yaw: {} (Relax={}) | Abs Control: {}\n\n\
                 ══════════ SAFETY ══════════\n\
                 Crash Recovery: {} | EZ Landing: {}\n\n\
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
                ff_yaw_hold_gain,
                ff_yaw_hold_time,
                iterm_relax,
                iterm_relax_type,
                iterm_relax_cutoff,
                iterm_rotation,
                anti_gravity,
                anti_gravity_p,
                anti_gravity_cutoff,
                d_max_gain,
                d_max_advance,
                tpa_mode,
                tpa_rate,
                tpa_breakpoint,
                tpa_low_rate,
                tpa_low_breakpoint,
                throttle_boost,
                motor_limit,
                dyn_idle,
                vbat_sag,
                thrust_linear,
                integrated_yaw,
                integrated_yaw_relax,
                abs_control,
                crash_recovery,
                ez_landing,
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
            ui.heading("⚒ PID & Filter Tuning Suggestions");
            ui.add_space(8.0);

            // === AI ANALYSIS SECTION ===
            ui.add_space(8.0);
            egui::CollapsingHeader::new(RichText::new("⌘ AI Analysis (OpenRouter)").strong())
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
                        ui.label("⌕ Filter:");
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
                            .button(format!("{} Save Settings", icons::FLOPPY_DISK))
                            .on_hover_text("Save API key and model selection for next session")
                            .clicked()
                        {
                            let settings = crate::settings::AppSettings {
                                ai: crate::settings::AISettings {
                                    api_key: self.api_key.clone(),
                                    selected_model_id: self.selected_model_id.clone(),
                                },
                                ui: Default::default(),
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
                            .add_enabled(can_analyze, egui::Button::new("▸ Analyze with AI"))
                            .clicked()
                        {
                            // Build metrics from analysis
                            let metrics = self.build_flight_metrics();
                            let model_id = self.selected_model_id.clone().unwrap_or_default();
                            let api_key = self.api_key.clone();

                            // Track AI analysis started
                            analytics::log_ai_analysis_started(
                                &model_id,
                                &format!("{}", self.analysis_focus),
                            );

                            // Start async analysis
                            self.ai_loading = true;
                            self.ai_error = None;
                            self.ai_receiver =
                                Some(OpenRouterClient::analyze_async(api_key, model_id, metrics));
                        }

                        if ui.button("↻ Refresh Models").clicked() && !self.models_loading {
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
                        ui.colored_label(Color32::RED, format!("× {}", err));
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
                                                        .small_button(icons::CLIPBOARD)
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
                            if ui.button(format!("{} Copy Response", icons::CLIPBOARD)).clicked() {
                                ui.output_mut(|o| o.copied_text = response.clone());
                                analytics::log_ai_response_copied();
                            }
                            if ui.button(format!("{} Clear", icons::TRASH)).clicked() {
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

            // === QUICK ACTIONS PANEL ===
            ui.label(
                RichText::new(format!("{} Quick Tuning Status", icons::GAUGE))
                    .strong()
                    .size(16.0),
            );
            ui.add_space(4.0);

            // Calculate status for each tuning area
            let p_status = self.get_category_status("P-Gain");
            let d_status = self.get_category_status("D-Gain");
            let filter_status = self.get_combined_status(&["Filtering", "Noise"]);
            let propwash_status = self.get_category_status("Propwash");
            let latency_status = self.get_category_status("Latency");
            let iterm_status = self.get_category_status("I-Term");
            let motor_status = self.get_category_status("Motors");

            egui::Frame::none()
                .fill(egui::Color32::from_black_alpha(40))
                .rounding(6.0)
                .inner_margin(12.0)
                .show(ui, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        self.draw_status_indicator(ui, "P-Gain", &p_status);
                        ui.add_space(16.0);
                        self.draw_status_indicator(ui, "D-Gain", &d_status);
                        ui.add_space(16.0);
                        self.draw_status_indicator(ui, "Filters", &filter_status);
                        ui.add_space(16.0);
                        self.draw_status_indicator(ui, "Propwash", &propwash_status);
                        ui.add_space(16.0);
                        self.draw_status_indicator(ui, "Latency", &latency_status);
                        ui.add_space(16.0);
                        self.draw_status_indicator(ui, "I-Term", &iterm_status);
                        ui.add_space(16.0);
                        self.draw_status_indicator(ui, "Motors", &motor_status);
                    });
                });

            ui.add_space(4.0);
            ui.label(
                RichText::new(format!(
                    "{} = Good  {} = Needs Attention  {} = Critical",
                    icons::CHECK_CIRCLE,
                    icons::WARNING_CIRCLE,
                    icons::X_CIRCLE
                ))
                .weak()
                .size(14.0),
            );

            ui.add_space(16.0);
            ui.separator();
            ui.add_space(8.0);

            // === RULE-BASED SUGGESTIONS ===
            ui.label(
                RichText::new(format!("{} Automated Analysis", icons::ROBOT))
                    .strong()
                    .size(16.0),
            );
            ui.label("Based on analysis of your flight log, here are tuning recommendations:");
            ui.add_space(8.0);

            // Export All CLI Commands section
            ui.horizontal(|ui| {
                // Copy All button - groups by severity for safe application order
                if ui
                    .button(format!("{} Copy All CLI Commands", icons::CLIPBOARD))
                    .on_hover_text(
                        "Copy all CLI commands to clipboard (Critical -> Warning -> Info)",
                    )
                    .clicked()
                {
                    let all_commands = self.generate_cli_export();
                    if !all_commands.is_empty() {
                        let cmd_count = self
                            .suggestions
                            .iter()
                            .filter(|s| s.cli_command.is_some())
                            .count();
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
                        .button(format!("● Copy Critical Only ({})", critical_count))
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
                egui::CollapsingHeader::new(RichText::new("= CLI Commands Preview").size(13.0))
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
                            ui.label(RichText::new("★ ").size(14.0));
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
                                                        .button(format!("{} Copy", icons::CLIPBOARD))
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
                    "⚠ Note: These are automated suggestions based on log analysis. \
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

            // === NEW ANALYSIS METRICS ===

            // Propwash analysis
            propwash_severity: self.analysis.propwash_severity,
            propwash_events: self.analysis.propwash_events,
            propwash_worst_axis: ["Roll", "Pitch", "Yaw"][self.analysis.propwash_worst_axis]
                .to_string(),

            // System latency
            system_latency_ms: self.analysis.system_latency_ms,
            avg_latency_ms: self.analysis.avg_latency_ms,

            // I-term analysis
            iterm_windup_events: self.analysis.iterm_windup_events,
            iterm_drift: self.analysis.iterm_drift,
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

    /// Get status (severity) for a category based on suggestions
    fn get_category_status(&self, category: &str) -> Severity {
        let matching: Vec<&TuningSuggestion> = self
            .suggestions
            .iter()
            .filter(|s| s.category.to_lowercase().contains(&category.to_lowercase()))
            .collect();

        if matching.iter().any(|s| s.severity == Severity::Critical) {
            Severity::Critical
        } else if matching.iter().any(|s| s.severity == Severity::Warning) {
            Severity::Warning
        } else if matching.is_empty() {
            Severity::Info // Green = no issues
        } else {
            Severity::Info
        }
    }

    /// Get combined status for multiple categories
    fn get_combined_status(&self, categories: &[&str]) -> Severity {
        let mut worst = Severity::Info;
        for cat in categories {
            let status = self.get_category_status(cat);
            if status == Severity::Critical {
                return Severity::Critical;
            } else if status == Severity::Warning {
                worst = Severity::Warning;
            }
        }
        worst
    }

    /// Draw a traffic light status indicator
    fn draw_status_indicator(&self, ui: &mut egui::Ui, label: &str, status: &Severity) {
        let (icon, color, tooltip) = match status {
            Severity::Critical => (
                icons::X_CIRCLE,
                egui::Color32::from_rgb(255, 80, 80),
                "Critical issues found",
            ),
            Severity::Warning => (
                icons::WARNING_CIRCLE,
                egui::Color32::from_rgb(255, 200, 80),
                "Needs attention",
            ),
            Severity::Info => (
                icons::CHECK_CIRCLE,
                egui::Color32::from_rgb(80, 255, 80),
                "Looking good",
            ),
        };

        let response = ui
            .horizontal(|ui| {
                ui.label(icon);
                ui.label(RichText::new(label).color(color).strong());
            })
            .response;

        response.on_hover_text(tooltip);
    }
}

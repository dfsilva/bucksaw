use realfft::num_complex::Complex32;

fn fft_forward(data: &[f32]) -> Vec<Complex32> {
    let mut input = data.to_vec();
    let planner = realfft::RealFftPlanner::<f32>::new().plan_fft_forward(input.len());
    let mut output = planner.make_output_vec();
    planner.process(&mut input, &mut output).unwrap();
    output
}

fn fft_inverse(data: &[Complex32], original_len: usize) -> Vec<f32> {
    if data.is_empty() || original_len == 0 {
        return vec![];
    }
    let mut input = data.to_vec();
    // Use the original signal length for inverse FFT planning
    let planner = realfft::RealFftPlanner::<f32>::new().plan_fft_inverse(original_len);
    let mut output = planner.make_output_vec();
    if planner.process(&mut input, &mut output).is_ok() {
        output
    } else {
        vec![0.0; original_len]
    }
}

/// Smoothing level for step response calculation
/// Values match PID Toolbox: [1, 20, 40, 60]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SmoothingLevel {
    /// No smoothing applied (span = 1)
    #[default]
    Off,
    /// Low smoothing (span = 20)
    Low,
    /// Medium smoothing (span = 40)
    Medium,
    /// High smoothing (span = 60)
    High,
}

impl SmoothingLevel {
    pub const ALL: [SmoothingLevel; 4] = [
        SmoothingLevel::Off,
        SmoothingLevel::Low,
        SmoothingLevel::Medium,
        SmoothingLevel::High,
    ];

    /// Returns the window size for smoothing (matches PID Toolbox smoothVals)
    pub fn window_size(self) -> usize {
        match self {
            SmoothingLevel::Off => 1,
            SmoothingLevel::Low => 20,
            SmoothingLevel::Medium => 40,
            SmoothingLevel::High => 60,
        }
    }
}

impl std::fmt::Display for SmoothingLevel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SmoothingLevel::Off => write!(f, "Off"),
            SmoothingLevel::Low => write!(f, "Low"),
            SmoothingLevel::Medium => write!(f, "Medium"),
            SmoothingLevel::High => write!(f, "High"),
        }
    }
}

/// Apply LOWESS (Locally Weighted Scatterplot Smoothing) to the data
/// This matches PIDtoolbox's smoothing approach for step response analysis
fn apply_lowess_smoothing(data: &[f32], window_size: usize) -> Vec<f32> {
    if window_size <= 1 || data.len() < window_size {
        return data.to_vec();
    }

    let half_window = window_size / 2;
    let mut smoothed = Vec::with_capacity(data.len());

    for i in 0..data.len() {
        let start = i.saturating_sub(half_window);
        let end = (i + half_window + 1).min(data.len());
        let window = &data[start..end];

        // LOWESS uses tricube weighting centered at the current point
        let center = i as f32;
        let max_dist = half_window as f32;

        let mut weighted_sum = 0.0f32;
        let mut weight_sum = 0.0f32;

        for (j, &val) in window.iter().enumerate() {
            let pos = (start + j) as f32;
            let dist = (pos - center).abs() / (max_dist + 1.0);
            // Tricube weight function: (1 - |d|^3)^3
            let weight = if dist < 1.0 {
                let t = 1.0 - dist.powi(3);
                t.powi(3)
            } else {
                0.0
            };
            weighted_sum += val * weight;
            weight_sum += weight;
        }

        smoothed.push(if weight_sum > 0.0 {
            weighted_sum / weight_sum
        } else {
            data[i]
        });
    }

    smoothed
}

/// Apply simple moving average smoothing (fallback)
fn apply_smoothing(data: &[f32], window_size: usize) -> Vec<f32> {
    if window_size <= 1 || data.len() < window_size {
        return data.to_vec();
    }

    let half_window = window_size / 2;
    let mut smoothed = Vec::with_capacity(data.len());

    for i in 0..data.len() {
        let start = i.saturating_sub(half_window);
        let end = (i + half_window + 1).min(data.len());
        let window = &data[start..end];
        let avg = window.iter().sum::<f32>() / window.len() as f32;
        smoothed.push(avg);
    }

    smoothed
}

// ========================================================================
// Betaflight-style filter implementations for simulation and visualization
// ========================================================================

// PTn cutoff correction factors from Betaflight (to achieve -3dB at specified frequency)
const CUTOFF_CORRECTION_PT2: f64 = 1.553773974;
const CUTOFF_CORRECTION_PT3: f64 = 1.961459177;

/// Betaflight-style PT1 (first-order lowpass) filter
pub struct Pt1Filter {
    state: f64,
    k: f64,
}

impl Pt1Filter {
    /// Create a new PT1 filter with given cutoff frequency and sample rate
    pub fn new(cutoff_hz: f64, sample_rate: f64) -> Self {
        let dt = 1.0 / sample_rate;
        let omega = 2.0 * std::f64::consts::PI * cutoff_hz * dt;
        let k = omega / (omega + 1.0);
        Self { state: 0.0, k }
    }

    /// Create from time constant (delay to reach 63.2% of step input) in ms
    pub fn from_time_constant_ms(time_constant_ms: f64, sample_rate: f64) -> Self {
        let dt = 1.0 / sample_rate;
        let delay = time_constant_ms / 1000.0;
        let k = if delay > 0.0 { dt / (dt + delay) } else { 1.0 };
        Self { state: 0.0, k }
    }

    pub fn apply(&mut self, input: f64) -> f64 {
        self.state = self.state + self.k * (input - self.state);
        self.state
    }

    pub fn reset(&mut self) {
        self.state = 0.0;
    }
}

/// Betaflight-style PT2 (second-order lowpass) filter
pub struct Pt2Filter {
    state1: f64,
    state: f64,
    k: f64,
}

impl Pt2Filter {
    /// Create a new PT2 filter with given cutoff frequency and sample rate
    pub fn new(cutoff_hz: f64, sample_rate: f64) -> Self {
        let dt = 1.0 / sample_rate;
        // Apply cutoff correction for -3dB at specified frequency
        let omega = 2.0 * std::f64::consts::PI * cutoff_hz * CUTOFF_CORRECTION_PT2 * dt;
        let k = omega / (omega + 1.0);
        Self {
            state1: 0.0,
            state: 0.0,
            k,
        }
    }

    /// Create from time constant (delay to reach 63.2% of step input) in ms
    /// Note: Cutoff correction is NOT applied here since we're working with time constant directly
    pub fn from_time_constant_ms(time_constant_ms: f64, sample_rate: f64) -> Self {
        let dt = 1.0 / sample_rate;
        let delay = time_constant_ms / 1000.0;
        let k = if delay > 0.0 { dt / (dt + delay) } else { 1.0 };
        Self {
            state1: 0.0,
            state: 0.0,
            k,
        }
    }

    pub fn apply(&mut self, input: f64) -> f64 {
        self.state1 = self.state1 + self.k * (input - self.state1);
        self.state = self.state + self.k * (self.state1 - self.state);
        self.state
    }

    pub fn reset(&mut self) {
        self.state1 = 0.0;
        self.state = 0.0;
    }
}

/// Betaflight-style PT3 (third-order lowpass) filter
pub struct Pt3Filter {
    state1: f64,
    state2: f64,
    state: f64,
    k: f64,
}

impl Pt3Filter {
    /// Create a new PT3 filter with given cutoff frequency and sample rate
    pub fn new(cutoff_hz: f64, sample_rate: f64) -> Self {
        let dt = 1.0 / sample_rate;
        // Apply cutoff correction for -3dB at specified frequency
        let omega = 2.0 * std::f64::consts::PI * cutoff_hz * CUTOFF_CORRECTION_PT3 * dt;
        let k = omega / (omega + 1.0);
        Self {
            state1: 0.0,
            state2: 0.0,
            state: 0.0,
            k,
        }
    }

    pub fn apply(&mut self, input: f64) -> f64 {
        self.state1 = self.state1 + self.k * (input - self.state1);
        self.state2 = self.state2 + self.k * (self.state1 - self.state2);
        self.state = self.state + self.k * (self.state2 - self.state);
        self.state
    }

    pub fn reset(&mut self) {
        self.state1 = 0.0;
        self.state2 = 0.0;
        self.state = 0.0;
    }
}

/// Filter type matching Betaflight's filter options
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum FilterType {
    #[default]
    Pt1,
    Pt2,
    Pt3,
}

impl FilterType {
    pub const ALL: [FilterType; 3] = [FilterType::Pt1, FilterType::Pt2, FilterType::Pt3];

    /// Returns the phase delay in degrees at the cutoff frequency
    pub fn phase_delay_at_cutoff(&self) -> f64 {
        match self {
            FilterType::Pt1 => 45.0,  // -45° at cutoff
            FilterType::Pt2 => 90.0,  // -90° at cutoff
            FilterType::Pt3 => 135.0, // -135° at cutoff
        }
    }

    /// Calculate phase delay in milliseconds at a given frequency
    pub fn phase_delay_ms(&self, freq_hz: f64) -> f64 {
        let phase_deg = self.phase_delay_at_cutoff();
        // Phase delay = phase / (360 * frequency)
        (phase_deg / 360.0) / freq_hz * 1000.0
    }
}

impl std::fmt::Display for FilterType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FilterType::Pt1 => write!(f, "PT1 (1st order)"),
            FilterType::Pt2 => write!(f, "PT2 (2nd order)"),
            FilterType::Pt3 => write!(f, "PT3 (3rd order)"),
        }
    }
}

/// Generate an ideal step response for a given filter type and time constant
/// Returns points as (time_ms, normalized_value)
pub fn generate_ideal_step_response(
    filter_type: FilterType,
    time_constant_ms: f64,
    max_time_ms: f64,
    sample_rate: f64,
) -> Vec<(f64, f64)> {
    let num_samples = ((max_time_ms / 1000.0) * sample_rate) as usize;
    let dt_ms = 1000.0 / sample_rate;

    let mut result = Vec::with_capacity(num_samples);

    match filter_type {
        FilterType::Pt1 => {
            // Analytical solution: y(t) = 1 - e^(-t/τ)
            let tau = time_constant_ms;
            for i in 0..=num_samples {
                let t = i as f64 * dt_ms;
                let y = 1.0 - (-t / tau).exp();
                result.push((t, y));
            }
        }
        FilterType::Pt2 => {
            // Simulate PT2 response (cascaded PT1)
            let mut filter = Pt2Filter::from_time_constant_ms(time_constant_ms, sample_rate);
            for i in 0..=num_samples {
                let t = i as f64 * dt_ms;
                let y = filter.apply(1.0);
                result.push((t, y));
            }
        }
        FilterType::Pt3 => {
            // Simulate PT3 response (cascaded PT1 x 3)
            let mut filter = Pt3Filter::new(
                1000.0 / (2.0 * std::f64::consts::PI * time_constant_ms),
                sample_rate,
            );
            for i in 0..=num_samples {
                let t = i as f64 * dt_ms;
                let y = filter.apply(1.0);
                result.push((t, y));
            }
        }
    }

    result
}

/// Extracted metrics from a step response curve
#[derive(Clone, Copy, Debug, Default)]
pub struct StepResponseMetrics {
    /// Time to reach 90% of steady-state value (ms)
    pub rise_time_ms: f32,
    /// Time to stay within 2% of final value (ms)
    pub settling_time_ms: f32,
    /// Maximum overshoot as percentage (e.g., 15.0 = 15% overshoot)
    pub overshoot_pct: f32,
    /// Maximum undershoot as percentage
    pub undershoot_pct: f32,
    /// Final steady-state error from 1.0 (ideal)
    pub steady_state_error: f32,
    /// Whether oscillations are present after settling
    pub has_oscillations: bool,
    /// Peak response time (time to reach maximum value, ms)
    pub peak_time_ms: f32,
}

impl StepResponseMetrics {
    /// Analyze a step response curve and extract metrics
    /// The response should be normalized so steady-state = 1.0
    pub fn analyze(response: &[(f64, f64)]) -> Self {
        if response.is_empty() {
            return Self::default();
        }

        let mut metrics = Self::default();

        // Find peak value and time
        let (peak_time, _peak_value) = response
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .copied()
            .unwrap_or((0.0, 0.0));

        metrics.peak_time_ms = peak_time as f32;

        // Calculate overshoot (max above 1.0)
        let max_value = response.iter().map(|(_, y)| *y).fold(0.0f64, f64::max);
        if max_value > 1.0 {
            metrics.overshoot_pct = ((max_value - 1.0) * 100.0) as f32;
        }

        // Calculate undershoot (within first 100ms, how far below 1.0)
        let first_100ms: Vec<_> = response.iter().filter(|(t, _)| *t <= 100.0).collect();
        if !first_100ms.is_empty() {
            let min_early = first_100ms.iter().map(|(_, y)| *y).fold(f64::MAX, f64::min);
            if min_early < 1.0 && min_early > 0.0 {
                metrics.undershoot_pct = ((1.0 - min_early) * 100.0) as f32;
            }
        }

        // Calculate rise time (time to reach 90% of steady-state)
        for (t, y) in response {
            if *y >= 0.9 {
                metrics.rise_time_ms = *t as f32;
                break;
            }
        }

        // Calculate settling time (time to stay within 2% of final value)
        let tolerance = 0.02;
        let mut settled_idx = None;
        for (i, (_, y)) in response.iter().enumerate().rev() {
            if (*y - 1.0).abs() > tolerance {
                settled_idx = Some(i);
                break;
            }
        }
        if let Some(idx) = settled_idx {
            if idx + 1 < response.len() {
                metrics.settling_time_ms = response[idx + 1].0 as f32;
            }
        }

        // Calculate steady-state error (average of last 20% of response)
        let tail_start = (response.len() as f64 * 0.8) as usize;
        if tail_start < response.len() {
            let tail_values: Vec<f64> = response[tail_start..].iter().map(|(_, y)| *y).collect();
            let avg = tail_values.iter().sum::<f64>() / tail_values.len() as f64;
            metrics.steady_state_error = (avg - 1.0).abs() as f32;
        }

        // Check for oscillations (does the response cross 1.0 multiple times after peak?)
        let mut crossings = 0;
        let peak_idx = response
            .iter()
            .position(|(t, _)| *t >= peak_time as f64)
            .unwrap_or(0);
        if peak_idx + 1 < response.len() {
            let mut was_above = response[peak_idx].1 > 1.0;
            for (_, y) in &response[peak_idx..] {
                let is_above = *y > 1.0;
                if is_above != was_above {
                    crossings += 1;
                    was_above = is_above;
                }
            }
        }
        metrics.has_oscillations = crossings >= 3;

        metrics
    }
}

/// Configuration for step response calculation
#[derive(Clone, Copy, Debug)]
pub struct StepResponseConfig {
    /// Minimum input threshold in deg/s (default: 20)
    pub min_input_threshold: f32,
    /// Segment duration in seconds (default: 2.0)
    pub segment_duration_sec: f32,
    /// Step response window in milliseconds (default: 500)
    pub response_window_ms: f32,
    /// Segment overlap/slide in seconds (default: 0.2)
    pub segment_slide_sec: f32,
}

impl Default for StepResponseConfig {
    fn default() -> Self {
        Self {
            min_input_threshold: 20.0,
            segment_duration_sec: 2.0,
            response_window_ms: 500.0,
            segment_slide_sec: 0.2,
        }
    }
}

pub fn calculate_step_response(
    _times: &[f64],
    setpoint: &[f32],
    gyro_filtered: &[f32],
    sample_rate: f64,
    smoothing: SmoothingLevel,
) -> Vec<(f64, f64)> {
    let input_spectrum = fft_forward(setpoint);
    let output_spectrum = fft_forward(gyro_filtered);

    let input_spec_conj: Vec<_> = input_spectrum.iter().map(|c| c.conj()).collect();

    // Epsilon threshold to prevent division by near-zero values (prevents NaN/infinity)
    const EPSILON: f32 = 1e-10;

    let frequency_response: Vec<_> = input_spectrum
        .iter()
        .zip(output_spectrum.iter())
        .zip(input_spec_conj.iter())
        .map(|((i, o), i_conj)| {
            let denominator = i_conj * i;
            if denominator.norm() < EPSILON {
                Complex32::new(0.0, 0.0)
            } else {
                (i_conj * o) / denominator
            }
        })
        .collect();

    let impulse_response = fft_inverse(&frequency_response, setpoint.len());

    // Calculate step response with NaN filtering
    let step_response: Vec<_> = impulse_response
        .iter()
        .scan(0.0f32, |cum_sum, x| {
            // Skip NaN or infinite values to prevent propagation
            if x.is_finite() {
                *cum_sum += *x;
            }
            Some(*cum_sum)
        })
        .collect();

    // Take first 500ms worth of samples (sample_rate is in Hz)
    // 500ms = 0.5 seconds, so samples = sample_rate * 0.5
    let samples_500ms = (sample_rate * 0.5) as usize;
    let response_len = samples_500ms.min(step_response.len());

    if response_len == 0 {
        return vec![];
    }

    // Apply LOWESS smoothing to the step response for better results (like PID Toolbox)
    let response_slice = &step_response[..response_len];
    let smoothed_response = apply_lowess_smoothing(response_slice, smoothing.window_size());

    // Normalize by steady-state value (average of last 30%)
    let steady_start = (response_len as f64 * 0.7) as usize;
    let steady_state = if steady_start < smoothed_response.len() {
        let steady_slice = &smoothed_response[steady_start..];
        steady_slice.iter().sum::<f32>() / steady_slice.len() as f32
    } else {
        smoothed_response.iter().sum::<f32>() / smoothed_response.len() as f32
    };

    if steady_state.abs() < 1e-10 {
        return vec![];
    }

    // Output: time in milliseconds, value normalized so steady-state = 1.0
    // time_step_ms = 1000 / sample_rate (converting from samples to milliseconds)
    let time_step_ms = 1000.0 / sample_rate;
    smoothed_response
        .iter()
        .enumerate()
        .map(|(i, &val)| (i as f64 * time_step_ms, (val / steady_state) as f64))
        .collect()
}

/// Calculate step response with custom configuration
/// Note: Currently uses the same algorithm as calculate_step_response,
/// but config.min_input_threshold is available for future segment-based filtering
pub fn calculate_step_response_with_config(
    times: &[f64],
    setpoint: &[f32],
    gyro_filtered: &[f32],
    sample_rate: f64,
    smoothing: SmoothingLevel,
    _config: StepResponseConfig,
) -> Vec<(f64, f64)> {
    // For now, use the same proven algorithm
    // The config is kept for API compatibility and future enhancements
    calculate_step_response(times, setpoint, gyro_filtered, sample_rate, smoothing)
}

use realfft::num_complex::Complex32;

fn fft_forward(data: &[f32]) -> Vec<Complex32> {
    let mut input = data.to_vec();
    let planner = realfft::RealFftPlanner::<f32>::new().plan_fft_forward(input.len());
    let mut output = planner.make_output_vec();
    planner.process(&mut input, &mut output).unwrap();
    output
}

fn fft_inverse(data: &[Complex32]) -> Vec<f32> {
    let mut input = data.to_vec();
    let planner = realfft::RealFftPlanner::<f32>::new().plan_fft_inverse(input.len() * 2 - 1);
    let mut output = planner.make_output_vec();
    if planner.process(&mut input, &mut output).is_ok() {
        output
    } else {
        vec![0.0; input.len()]
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

/// Apply moving average smoothing to the data
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
    let frequency_response: Vec<_> = input_spectrum
        .iter()
        .zip(output_spectrum.iter())
        .zip(input_spec_conj.iter())
        .map(|((i, o), i_conj)| (i_conj * o) / (i_conj * i))
        .collect();

    let impulse_response = fft_inverse(&frequency_response);
    let step_response: Vec<_> = impulse_response
        .iter()
        .scan(0.0, |cum_sum, x| {
            *cum_sum += *x;
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

    // Apply smoothing to the step response (this is what PID Toolbox does)
    let response_slice = &step_response[..response_len];
    let smoothed_response = apply_smoothing(response_slice, smoothing.window_size());

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

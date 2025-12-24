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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SmoothingLevel {
    /// No smoothing applied
    #[default]
    Off,
    /// Window size of 5 samples
    Low,
    /// Window size of 15 samples
    Medium,
    /// Window size of 31 samples
    High,
}

impl SmoothingLevel {
    pub const ALL: [SmoothingLevel; 4] = [
        SmoothingLevel::Off,
        SmoothingLevel::Low,
        SmoothingLevel::Medium,
        SmoothingLevel::High,
    ];

    /// Returns the window size for moving average smoothing
    pub fn window_size(self) -> usize {
        match self {
            SmoothingLevel::Off => 1,
            SmoothingLevel::Low => 5,
            SmoothingLevel::Medium => 15,
            SmoothingLevel::High => 31,
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

pub fn calculate_step_response(
    times: &[f64],
    setpoint: &[f32],
    gyro_filtered: &[f32],
    sample_rate: f64,
    smoothing: SmoothingLevel,
) -> Vec<(f64, f64)> {
    // Apply smoothing to gyro data before FFT
    let smoothed_gyro = apply_smoothing(gyro_filtered, smoothing.window_size());

    let input_spectrum = fft_forward(setpoint);
    let output_spectrum = fft_forward(&smoothed_gyro);

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

    let avg = step_response.iter().sum::<f32>() / (step_response.len() as f32);
    let normalized = step_response
        .iter()
        .take((sample_rate / 2.0) as usize) // limit to last 500ms
        .map(|x| x / avg);

    let start = times.first().cloned().unwrap_or(0.0);
    times
        .iter()
        .zip(normalized)
        .map(|(t, s)| (*t - start, s as f64))
        .collect()
}

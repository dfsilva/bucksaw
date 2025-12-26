use std::collections::HashMap;
use std::f32::consts::PI;
use std::sync::mpsc::{channel, Receiver, TryRecvError};
use std::sync::{Arc, Mutex, OnceLock};

use realfft::RealToComplex;

use egui::Color32;
use egui_phosphor::regular as icons;
use itertools::Itertools;

use crate::flight_data::FlightData;
use crate::iter::IterExt;
use crate::utils::execute_in_background;

use super::PLOT_HEIGHT;

const COLORGRAD_LOOKUP_SIZE: usize = 128;
const TIME_DOMAIN_TEX_WIDTH: usize = 1024;
// PIDtoolbox uses 100 throttle buckets (0-100% throttle percentage)
const THROTTLE_DOMAIN_BUCKETS: usize = 100;
const FFT_SIZE_OPTIONS: [usize; 4] = [256, 512, 1024, 2048];

#[derive(PartialEq, Clone, Copy)]
enum VibeDomain {
    Time,
    Throttle,
}

#[derive(PartialEq, Clone, Copy, Default, Debug)]
enum Colorscheme {
    Turbo,
    Viridis,
    Inferno,
    Magma,
    Plasma,
    Cividis,
    Spectral,
    #[default]
    Hot,
}

impl Colorscheme {
    const ALL: [Colorscheme; 8] = [
        Colorscheme::Turbo,
        Colorscheme::Viridis,
        Colorscheme::Inferno,
        Colorscheme::Magma,
        Colorscheme::Plasma,
        Colorscheme::Cividis,
        Colorscheme::Spectral,
        Colorscheme::Hot,
    ];
}

impl From<Colorscheme> for colorgrad::Gradient {
    fn from(val: Colorscheme) -> Self {
        match val {
            Colorscheme::Turbo => colorgrad::turbo(),
            Colorscheme::Viridis => colorgrad::viridis(),
            Colorscheme::Inferno => colorgrad::inferno(),
            Colorscheme::Magma => colorgrad::magma(),
            Colorscheme::Plasma => colorgrad::plasma(),
            Colorscheme::Cividis => colorgrad::cividis(),
            Colorscheme::Spectral => colorgrad::spectral(),
            Colorscheme::Hot => colorgrad::CustomGradient::new()
                .colors(&[
                    colorgrad::Color::from_rgba8(0, 0, 0, 255),
                    colorgrad::Color::from_rgba8(128, 0, 0, 255), // Dark red
                    colorgrad::Color::from_rgba8(255, 0, 0, 255),
                    colorgrad::Color::from_rgba8(255, 165, 0, 255), // Orange
                    colorgrad::Color::from_rgba8(255, 255, 0, 255),
                    colorgrad::Color::from_rgba8(255, 255, 255, 255),
                ])
                .domain(&[0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
                .build()
                .unwrap(),
        }
    }
}

/// Frequency band overlay for highlighting specific frequency ranges
#[derive(Clone, PartialEq)]
struct FrequencyBand {
    pub enabled: bool,
    pub name: String,
    pub min_hz: f32,
    pub max_hz: f32,
    pub color: Color32,
}

impl FrequencyBand {
    fn new(name: &str, min_hz: f32, max_hz: f32, color: Color32) -> Self {
        Self {
            enabled: false,
            name: name.to_string(),
            min_hz,
            max_hz,
            color,
        }
    }
}

/// Analysis preset for quick configuration
#[derive(Clone, Copy, PartialEq, Debug)]
enum AnalysisPreset {
    Custom,
    MotorTuning,
    PropWash,
    FullSpectrum,
}

impl std::fmt::Display for AnalysisPreset {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AnalysisPreset::Custom => write!(f, "Custom"),
            AnalysisPreset::MotorTuning => write!(f, "Motor Tuning"),
            AnalysisPreset::PropWash => write!(f, "Prop Wash"),
            AnalysisPreset::FullSpectrum => write!(f, "Full Spectrum"),
        }
    }
}

/// Scale mode for amplitude display
#[derive(PartialEq, Clone, Copy, Default, Debug)]
enum AmplitudeScale {
    #[default]
    Linear,
    /// Power Spectral Density in dB (like PIDtoolbox PSD mode)
    DeciBel,
}

#[derive(PartialEq, Clone)]
struct FftSettings {
    pub size: usize,
    pub step_size: usize,
    pub plot_colorscheme: Colorscheme,
    pub plot_max: f32,
    pub auto_scale: bool,
    pub min_freq_hz: f32,
    pub max_freq_hz: f32,
    /// Motor pole count for eRPM to Hz conversion (default 14 for typical 5" motors)
    pub motor_poles: u8,
    /// Amplitude scale mode (linear or dB)
    pub amplitude_scale: AmplitudeScale,
    /// Smoothing factor for Gaussian kernel (1-5, higher = more smoothing)
    pub smooth_factor: u8,
    color_lookup_table: Option<(Colorscheme, [Color32; COLORGRAD_LOOKUP_SIZE])>,
}

impl FftSettings {
    fn color_lookup_table(&mut self) -> &[Color32; COLORGRAD_LOOKUP_SIZE] {
        if self
            .color_lookup_table
            .map(|(t, _)| t != self.plot_colorscheme)
            .unwrap_or(true)
        {
            let gradient: colorgrad::Gradient = self.plot_colorscheme.into();
            let table = (0..COLORGRAD_LOOKUP_SIZE)
                .map(move |i| {
                    let f = (i as f64) / (COLORGRAD_LOOKUP_SIZE as f64);
                    let rgba = gradient.at(f).to_rgba8();
                    Color32::from_rgb(rgba[0], rgba[1], rgba[2])
                })
                .collect::<Vec<_>>()
                .try_into()
                .unwrap();

            self.color_lookup_table = Some((self.plot_colorscheme, table));
        }

        self.color_lookup_table.as_ref().map(|(_, t)| t).unwrap()
    }

    pub fn color_at(&mut self, f: f32) -> Color32 {
        let i = (f * (COLORGRAD_LOOKUP_SIZE as f32)) as usize;
        let i = usize::min(i, COLORGRAD_LOOKUP_SIZE - 1);
        self.color_lookup_table()[i]
    }

    pub fn needs_recalculating(&self, other: &Self) -> bool {
        self.size != other.size || self.step_size != other.step_size
    }

    pub fn needs_redrawing(&self, other: &Self) -> bool {
        self.needs_recalculating(other)
            || self.plot_colorscheme != other.plot_colorscheme
            || self.plot_max != other.plot_max
            || self.auto_scale != other.auto_scale
            || self.amplitude_scale != other.amplitude_scale
            || self.smooth_factor != other.smooth_factor
    }

    /// Draw a horizontal colorbar legend showing the color scale
    pub fn draw_colorbar(&mut self, ui: &mut egui::Ui, width: f32) {
        let height = 15.0;
        let (rect, _response) =
            ui.allocate_exact_size(egui::vec2(width, height + 15.0), egui::Sense::hover());

        // Draw the gradient bar
        let bar_rect =
            egui::Rect::from_min_size(rect.min + egui::vec2(0.0, 0.0), egui::vec2(width, height));

        let painter = ui.painter_at(rect);

        // Draw gradient using multiple vertical strips
        let num_strips = 64;
        let strip_width = width / num_strips as f32;
        for i in 0..num_strips {
            let t = i as f32 / (num_strips - 1) as f32;
            let color = self.color_at(t);
            let strip_rect = egui::Rect::from_min_size(
                bar_rect.min + egui::vec2(i as f32 * strip_width, 0.0),
                egui::vec2(strip_width + 0.5, height),
            );
            painter.rect_filled(strip_rect, 0.0, color);
        }

        // Draw border
        painter.rect_stroke(bar_rect, 0.0, egui::Stroke::new(1.0, egui::Color32::GRAY));

        // Draw scale labels
        let label_y = rect.min.y + height + 2.0;
        let labels = ["0", "0.25", "0.5", "0.75", "1.0"];
        for (i, label) in labels.iter().enumerate() {
            let x = rect.min.x + (i as f32 / 4.0) * width;
            painter.text(
                egui::pos2(x, label_y),
                egui::Align2::CENTER_TOP,
                *label,
                egui::FontId::proportional(9.0),
                egui::Color32::LIGHT_GRAY,
            );
        }
    }
}

impl Default for FftSettings {
    fn default() -> Self {
        Self {
            size: 256,
            step_size: 8,
            plot_colorscheme: Colorscheme::default(),
            // Default max for amplitude scale matching PIDtoolbox climScale default of 0.5
            plot_max: 0.5,
            auto_scale: true, // Auto-normalize for best contrast by default
            min_freq_hz: 0.0,
            max_freq_hz: 150.0, // Default to ~150Hz like PIDtoolbox sub-100Hz view
            motor_poles: 14,   // Common for 5" quad motors (7 pole pairs)
            amplitude_scale: AmplitudeScale::default(),
            smooth_factor: 2,  // Moderate smoothing by default
            color_lookup_table: None,
        }
    }
}

impl FftSettings {
    /// Convert eRPM to mechanical frequency in Hz
    /// Formula: Hz = eRPM / 60 / (poles / 2)
    pub fn erpm_to_hz(&self, erpm: f32) -> f32 {
        erpm / 60.0 / (self.motor_poles as f32 / 2.0)
    }
    
    /// Get Gaussian kernel size based on smooth_factor
    /// Matches PIDtoolbox's fspecial('gaussian', [smoothFactor*5, smoothFactor], 4)
    pub fn gaussian_kernel_size(&self) -> (usize, usize) {
        let factor = self.smooth_factor.max(1) as usize;
        (factor * 5, factor)
    }
}

/// Detected frequency peak with sub-bin accuracy
#[derive(Clone, Debug)]
pub struct FrequencyPeak {
    /// Frequency in Hz (with parabolic interpolation for sub-bin accuracy)
    pub frequency_hz: f32,
    /// Peak amplitude (normalized)
    pub amplitude: f32,
    /// Quality factor - peak prominence relative to noise floor
    pub prominence: f32,
}

#[derive(Clone)]
struct FftChunk {
    time: f64,
    fft: Vec<f32>,
    throttle: f32,
}

impl FftChunk {
    pub fn hamming_window(fft_size: usize) -> &'static [f32] {
        // Standard Hamming window: w(n) = 0.54 - 0.46 * cos(2π * n / (N-1))
        // Matches MATLAB's hamming() function used by PIDtoolbox
        static LOOKUP: OnceLock<[Vec<f32>; FFT_SIZE_OPTIONS.len()]> = OnceLock::new();
        let lookup = LOOKUP.get_or_init(|| {
            FFT_SIZE_OPTIONS
                .into_iter()
                .map(|fft_size| {
                    (0..fft_size)
                        .map(|i| {
                            0.54 - 0.46 * (2.0 * PI * (i as f32) / ((fft_size - 1) as f32)).cos()
                        })
                        .collect()
                })
                .collect::<Vec<_>>()
                .try_into()
                .unwrap()
        });

        &lookup[FFT_SIZE_OPTIONS
            .iter()
            .position(|s| *s == fft_size)
            .unwrap()]
    }
    
    /// Find peaks in FFT spectrum using Betaflight's algorithm with parabolic interpolation
    /// Returns up to `max_peaks` peaks sorted by amplitude
    pub fn find_peaks(
        fft: &[f32],
        sample_rate: f32,
        min_freq_hz: f32,
        max_freq_hz: f32,
        max_peaks: usize,
    ) -> Vec<FrequencyPeak> {
        if fft.len() < 3 {
            return vec![];
        }
        
        let fft_size = (fft.len() + 1) * 2; // Reconstruct original FFT size
        let freq_resolution = sample_rate / fft_size as f32;
        
        // Calculate noise floor (2x average PSD, like Betaflight)
        let total_power: f32 = fft.iter().map(|&v| v * v).sum();
        let noise_floor = (total_power / fft.len() as f32).sqrt() * 2.0;
        
        // Find local maxima that exceed noise floor
        let mut peaks: Vec<(usize, f32)> = Vec::new();
        
        let min_bin = ((min_freq_hz / freq_resolution) as usize).max(1);
        let max_bin = ((max_freq_hz / freq_resolution) as usize).min(fft.len() - 2);
        
        for i in min_bin..=max_bin {
            let val = fft[i];
            // Check if local maximum and above noise floor
            if val > fft[i - 1] && val > fft[i + 1] && val > noise_floor {
                peaks.push((i, val));
            }
        }
        
        // Sort by amplitude (descending) and take top N
        peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        peaks.truncate(max_peaks);
        
        // Apply parabolic interpolation for sub-bin accuracy (Betaflight's method)
        peaks.into_iter().map(|(bin, amp)| {
            let y0 = fft[bin - 1];
            let y1 = fft[bin];
            let y2 = fft[bin + 1];
            
            // Parabolic interpolation: offset = (y0 - y2) / (2 * (y0 - 2*y1 + y2))
            let denom = 2.0 * (y0 - 2.0 * y1 + y2);
            let offset = if denom.abs() > 1e-10 {
                (y0 - y2) / denom
            } else {
                0.0
            };
            
            let precise_bin = bin as f32 + offset.clamp(-0.5, 0.5);
            let frequency_hz = (precise_bin + 1.0) * freq_resolution; // +1 because we skip DC
            
            FrequencyPeak {
                frequency_hz,
                amplitude: amp,
                prominence: amp / noise_floor.max(1e-10),
            }
        }).collect()
    }

    /// Get a cached FFT planner for the given size
    /// FFT planners are expensive to create, so we cache them per size
    fn cached_fft_planner(size: usize) -> Arc<dyn RealToComplex<f32>> {
        static PLANNERS: OnceLock<Mutex<HashMap<usize, Arc<dyn RealToComplex<f32>>>>> = OnceLock::new();
        let planners = PLANNERS.get_or_init(|| Mutex::new(HashMap::new()));
        let mut guard = planners.lock().unwrap();
        guard
            .entry(size)
            .or_insert_with(|| {
                Arc::from(realfft::RealFftPlanner::<f32>::new().plan_fft_forward(size))
            })
            .clone()
    }

    pub fn calculate(time: f64, data: &[f32], throttle: f32) -> Self {
        // Convert to complex and apply hamming window
        let window = Self::hamming_window(data.len());
        let mut input: Vec<_> = data.iter().zip(window.iter()).map(|(d, w)| d * w).collect();

        // Use cached planner for performance
        let planner = Self::cached_fft_planner(data.len());
        let mut output = planner.make_output_vec();
        planner.process(&mut input, &mut output).unwrap();

        // Calculate window sum for amplitude normalization (like PIDtoolbox amplitude mode)
        // PIDtoolbox uses abs(fft(signal .* window)) / sum(window) for amplitude
        let window_sum: f32 = window.iter().sum();

        // Compute amplitude spectrum (not dB!) matching PIDtoolbox default mode
        // PIDtoolbox normalizes by N/2 and applies window correction
        // Low frequencies at bottom (no rev), will flip during display
        let fft = output
            .into_iter()
            .skip(1) // Skip DC component
            .map(|c| {
                // Amplitude = magnitude normalized by window sum and FFT length
                // Multiply by 2 for one-sided spectrum (except DC and Nyquist)
                let amplitude = (c.re.powi(2) + c.im.powi(2)).sqrt() * 2.0 / window_sum;
                amplitude
            })
            .collect();

        Self {
            time,
            fft,
            throttle,
        }
    }
}

type FftAxisValueCallback = Box<fn(&FlightData) -> [Option<&Vec<f32>>; 3]>;

const AXIS_NAMES: [&str; 3] = ["Roll", "Pitch", "Yaw"];

struct FftAxis {
    ctx: egui::Context,
    fft_settings: FftSettings,

    i: usize,
    axis_name: &'static str,
    flight_data: Arc<FlightData>,
    value_callback: FftAxisValueCallback,

    chunks: Vec<FftChunk>,
    chunk_receiver: Option<Receiver<Vec<FftChunk>>>,
    time_textures: Vec<(f64, f64, egui::TextureHandle)>,
    time_texture_receiver: Option<Receiver<(f64, f64, egui::TextureHandle)>>,
    throttle_texture: Option<egui::TextureHandle>,
    throttle_texture_receiver: Option<Receiver<egui::TextureHandle>>,
    /// Detected peaks from aggregated FFT data (top 5 peaks)
    detected_peaks: Vec<FrequencyPeak>,
    /// Average spectrum across all FFT chunks for peak detection
    average_spectrum: Option<Vec<f32>>,
}

impl FftAxis {
    pub fn new(
        ctx: &egui::Context,
        fft_settings: FftSettings,
        i: usize,
        flight_data: Arc<FlightData>,
        value_callback: fn(&FlightData) -> [Option<&Vec<f32>>; 3],
    ) -> Self {
        let mut new = Self {
            ctx: ctx.clone(),
            fft_settings,

            i,
            axis_name: AXIS_NAMES[i],
            flight_data,
            value_callback: Box::new(value_callback),

            chunks: Vec::new(),
            chunk_receiver: None,
            time_textures: Vec::new(),
            time_texture_receiver: None,
            throttle_texture: None,
            throttle_texture_receiver: None,
            detected_peaks: Vec::new(),
            average_spectrum: None,
        };
        new.recalculate_ffts();
        new
    }

    /// Generate a 2D Gaussian kernel matching PIDtoolbox's fspecial('gaussian', [N*5, N], sigma)
    fn generate_gaussian_kernel(smooth_factor: u8) -> Vec<Vec<f32>> {
        let factor = smooth_factor.max(1) as usize;
        let height = factor * 5;  // Frequency axis (more smoothing)
        let width = factor;       // Time/throttle axis (less smoothing)
        let sigma = 4.0f32;       // PIDtoolbox default sigma
        
        let mut kernel = vec![vec![0.0f32; width]; height];
        let center_y = (height as f32 - 1.0) / 2.0;
        let center_x = (width as f32 - 1.0) / 2.0;
        
        let mut sum = 0.0f32;
        for y in 0..height {
            for x in 0..width {
                let dy = y as f32 - center_y;
                let dx = x as f32 - center_x;
                let val = (-((dx * dx + dy * dy) / (2.0 * sigma * sigma))).exp();
                kernel[y][x] = val;
                sum += val;
            }
        }
        
        // Normalize kernel
        for row in &mut kernel {
            for val in row {
                *val /= sum;
            }
        }
        
        kernel
    }
    
    /// Apply 2D Gaussian smoothing to FFT data
    fn apply_gaussian_smoothing(
        data: &[FftChunk],
        smooth_factor: u8,
    ) -> Vec<Vec<f32>> {
        let width = data.len();
        let height = data[0].fft.len();
        let kernel = Self::generate_gaussian_kernel(smooth_factor);
        let k_height = kernel.len() as i32;
        let k_width = kernel[0].len() as i32;
        let k_center_y = k_height / 2;
        let k_center_x = k_width / 2;
        
        let mut smoothed = vec![vec![0.0f32; height]; width];
        
        for x in 0..width {
            for y in 0..height {
                let mut sum = 0.0f32;
                let mut weight_sum = 0.0f32;
                
                for ky in 0..k_height {
                    for kx in 0..k_width {
                        let nx = x as i32 + kx - k_center_x;
                        let ny = y as i32 + ky - k_center_y;
                        
                        if nx >= 0 && nx < width as i32 && ny >= 0 && ny < height as i32 {
                            let w = kernel[ky as usize][kx as usize];
                            sum += data[nx as usize].fft[ny as usize] * w;
                            weight_sum += w;
                        }
                    }
                }
                
                smoothed[x][y] = if weight_sum > 0.0 { sum / weight_sum } else { 0.0 };
            }
        }
        
        smoothed
    }

    pub fn create_image(
        data: &[FftChunk],
        max: f32,
        fft_settings: &mut FftSettings,
    ) -> egui::ColorImage {
        let fft_len = data[0].fft.len();
        let mut image = egui::ColorImage::new([data.len(), fft_len], Color32::TRANSPARENT);

        let width = data.len();
        let height = fft_len;
        
        // Apply configurable Gaussian smoothing (PIDtoolbox style)
        let smoothed = Self::apply_gaussian_smoothing(data, fft_settings.smooth_factor);

        // Find the actual max value from smoothed data for normalization
        let actual_max = if fft_settings.auto_scale {
            smoothed.iter()
                .flat_map(|col| col.iter())
                .fold(0.0f32, |a, &b| a.max(b))
                .max(1e-10)
        } else {
            max.max(1e-10)
        };

        // Fill image with colors, flipping Y axis so low frequencies are at bottom
        for x in 0..width {
            for y in 0..height {
                let val = smoothed[x][y];
                
                // Apply amplitude scale mode
                let normalized = match fft_settings.amplitude_scale {
                    AmplitudeScale::Linear => (val / actual_max).clamp(0.0, 1.0),
                    AmplitudeScale::DeciBel => {
                        // PSD dB mode: range from -40dB to +10dB (like PIDtoolbox)
                        let db = 20.0 * (val.max(1e-10)).log10();
                        let min_db = -40.0f32;
                        let max_db = 10.0f32;
                        ((db - min_db) / (max_db - min_db)).clamp(0.0, 1.0)
                    }
                };
                
                // Flip Y: image y=0 is top, so put high freq at top, low freq at bottom
                let flipped_y = height - 1 - y;
                image[(x, flipped_y)] = fft_settings.color_at(normalized);
            }
        }

        image
    }

    pub fn recalculate_ffts(&mut self) {
        let (chunk_sender, chunk_receiver) = channel();

        self.chunks.truncate(0);
        self.chunk_receiver = Some(chunk_receiver);
        self.time_textures.truncate(0);
        self.throttle_texture = None;

        let fd = self.flight_data.clone();
        let cb = self.value_callback.clone();
        let i = self.i;
        let fft_size = self.fft_settings.size;
        let fft_step_size = self.fft_settings.step_size;
        let ctx = self.ctx.clone();
        execute_in_background(async move {
            let throttle = &fd.setpoint().unwrap()[3];
            let Some(values) = &cb(&fd)[i] else { return };
            let time_windows = fd
                .times
                .iter()
                .copied()
                .overlapping_windows(fft_size, fft_step_size);
            let data_windows = values
                .iter()
                .copied()
                .overlapping_windows(fft_size, fft_step_size);
            let throttle_windows = throttle
                .iter()
                .copied()
                .overlapping_windows(fft_size, fft_step_size);

            time_windows
                .zip(data_windows.zip(throttle_windows))
                .filter(|(time, _)| time.len() == fft_size)
                .map(|(time, (data, throttle))| {
                    FftChunk::calculate(time[0], &data, throttle[throttle.len() / 2])
                })
                .chunks(100)
                .into_iter()
                .for_each(|chunks| {
                    // Ignore send errors - happens when receiver is dropped (e.g., flight change)
                    if chunk_sender.send(chunks.collect()).is_ok() {
                        ctx.request_repaint();
                    }
                });
        });
    }

    pub fn redraw_textures(&mut self) {
        self.time_textures.truncate(0);
        self.throttle_texture = None;

        let (time_texture_sender, time_texture_receiver) = channel();
        let (throttle_texture_sender, throttle_texture_receiver) = channel();

        let fft_size = self.fft_settings.size;

        let chunks = self.chunks.clone(); // TODO
        let mut fft_settings = self.fft_settings.clone();
        let fft_max = self.fft_settings.plot_max;
        let ctx = self.ctx.clone();
        execute_in_background(async move {
            for (i, columns) in chunks.chunks(TIME_DOMAIN_TEX_WIDTH).enumerate() {
                let image = Self::create_image(columns, fft_max, &mut fft_settings);
                let tex_handle =
                    ctx.load_texture(format!("tex_{:?}", i), image, Default::default());
                let start = columns.first().unwrap().time;
                let end = columns.last().unwrap().time;
                // Ignore send errors - happens when receiver is dropped (e.g., flight change)
                if time_texture_sender.send((start, end, tex_handle)).is_ok() {
                    ctx.request_repaint();
                }
            }
        });

        let chunks = self.chunks.clone(); // TODO
        let mut fft_settings = self.fft_settings.clone();
        let fft_max = self.fft_settings.plot_max;
        let ctx = self.ctx.clone();
        execute_in_background(async move {
            const ARRAY_REPEAT_VALUE: Vec<FftChunk> = Vec::new();
            let mut throttle_buckets: [Vec<FftChunk>; THROTTLE_DOMAIN_BUCKETS] =
                [ARRAY_REPEAT_VALUE; THROTTLE_DOMAIN_BUCKETS];
            for chunk in chunks {
                let bucket_i =
                    ((chunk.throttle / 1000.0) * THROTTLE_DOMAIN_BUCKETS as f32) as usize;
                let bucket_i = usize::min(bucket_i, THROTTLE_DOMAIN_BUCKETS - 1);
                throttle_buckets[bucket_i].push(chunk);
            }

            let mut throttle_averages: Vec<Option<Vec<f32>>> = Vec::new();
            let fft_output_size = fft_size / 2; // realfft output is N/2+1, we skip DC so N/2

            for bucket in throttle_buckets.into_iter() {
                let size = bucket.len();
                if size == 0 {
                    throttle_averages.push(None);
                    continue;
                }

                // Get actual FFT size from first chunk in bucket
                let first_fft_len = bucket
                    .first()
                    .map(|c| c.fft.len())
                    .unwrap_or(fft_output_size);

                let avg = bucket
                    .into_iter()
                    .map(|chunk| chunk.fft)
                    .fold(vec![0f32; first_fft_len], |a, b| {
                        a.into_iter()
                            .zip(b.into_iter())
                            .map(|(a, b)| {
                                if a.is_normal() && b.is_normal() {
                                    a + b
                                } else if a.is_normal() {
                                    a
                                } else {
                                    b
                                }
                            })
                            .collect()
                    })
                    .into_iter()
                    .map(|v| v / (size as f32))
                    .collect::<Vec<_>>();
                throttle_averages.push(Some(avg));
            }

            // Interpolate missing buckets
            for i in 0..THROTTLE_DOMAIN_BUCKETS {
                if throttle_averages[i].is_some() {
                    continue;
                }

                // Find left neighbor
                let left = (0..i).rev().find(|&x| throttle_averages[x].is_some());
                // Find right neighbor
                let right =
                    (i + 1..THROTTLE_DOMAIN_BUCKETS).find(|&x| throttle_averages[x].is_some());

                let interpolated = match (left, right) {
                    (Some(l), Some(r)) => {
                        let left_vec = throttle_averages[l].as_ref().unwrap();
                        let right_vec = throttle_averages[r].as_ref().unwrap();
                        let t = (i - l) as f32 / (r - l) as f32;

                        Some(
                            left_vec
                                .iter()
                                .zip(right_vec.iter())
                                .map(|(a, b)| a + (b - a) * t)
                                .collect(),
                        )
                    }
                    (Some(l), None) => throttle_averages[l].clone(),
                    (None, Some(r)) => throttle_averages[r].clone(),
                    (None, None) => Some(vec![0.0; fft_output_size]),
                };

                throttle_averages[i] = interpolated;
            }

            let throttle_averages: Vec<Vec<f32>> =
                throttle_averages.into_iter().map(|o| o.unwrap()).collect();

            let width = THROTTLE_DOMAIN_BUCKETS;
            // Use actual height from the data
            let height = throttle_averages
                .first()
                .map(|v| v.len())
                .unwrap_or(fft_output_size);

            // Apply configurable Gaussian smoothing (PIDtoolbox style)
            let smooth_factor = fft_settings.smooth_factor.max(1) as usize;
            let k_height = smooth_factor * 5;  // Frequency axis (more smoothing)
            let k_width = smooth_factor;       // Throttle axis (less smoothing)
            let sigma = 4.0f32;
            
            // Generate Gaussian kernel
            let mut kernel = vec![vec![0.0f32; k_width]; k_height];
            let center_y = (k_height as f32 - 1.0) / 2.0;
            let center_x = (k_width as f32 - 1.0) / 2.0;
            let mut kernel_sum = 0.0f32;
            
            for ky in 0..k_height {
                for kx in 0..k_width {
                    let dy = ky as f32 - center_y;
                    let dx = kx as f32 - center_x;
                    let val = (-((dx * dx + dy * dy) / (2.0 * sigma * sigma))).exp();
                    kernel[ky][kx] = val;
                    kernel_sum += val;
                }
            }
            
            // Normalize kernel
            for row in &mut kernel {
                for val in row {
                    *val /= kernel_sum;
                }
            }
            
            let k_center_y = k_height as i32 / 2;
            let k_center_x = k_width as i32 / 2;
            let mut smoothed = vec![vec![0.0f32; height]; width];

            for x in 0..width {
                for y in 0..height {
                    let mut sum = 0.0f32;
                    let mut weight_sum = 0.0f32;

                    for ky in 0..k_height as i32 {
                        for kx in 0..k_width as i32 {
                            let nx = x as i32 + kx - k_center_x;
                            let ny = y as i32 + ky - k_center_y;

                            if nx >= 0 && nx < width as i32 && ny >= 0 && ny < height as i32 {
                                let w = kernel[ky as usize][kx as usize];
                                sum += throttle_averages[nx as usize][ny as usize] * w;
                                weight_sum += w;
                            }
                        }
                    }

                    smoothed[x][y] = if weight_sum > 0.0 { sum / weight_sum } else { 0.0 };
                }
            }

            let mut image = egui::ColorImage::new([width, height], Color32::TRANSPARENT);

            // Find actual max value from the smoothed data for normalization
            // Use fixed scale if auto_scale is disabled
            let actual_max = if fft_settings.auto_scale {
                smoothed
                    .iter()
                    .flat_map(|v| v.iter())
                    .fold(0.0f32, |a, &b| a.max(b))
                    .max(1e-10)
            } else {
                fft_max.max(1e-10)
            };

            // Fill image with colors, flipping Y axis so low frequencies are at bottom
            for x in 0..width {
                for y in 0..height {
                    let val = smoothed[x][y];
                    
                    // Apply amplitude scale mode
                    let normalized = match fft_settings.amplitude_scale {
                        AmplitudeScale::Linear => (val / actual_max).clamp(0.0, 1.0),
                        AmplitudeScale::DeciBel => {
                            // PSD dB mode: range from -40dB to +10dB (like PIDtoolbox)
                            let db = 20.0 * (val.max(1e-10)).log10();
                            let min_db = -40.0f32;
                            let max_db = 10.0f32;
                            ((db - min_db) / (max_db - min_db)).clamp(0.0, 1.0)
                        }
                    };
                    
                    let flipped_y = height - 1 - y;
                    image[(x, flipped_y)] = fft_settings.color_at(normalized);
                }
            }

            let tex_handle = ctx.load_texture("throttle_fft", image, Default::default());
            // Ignore send errors - happens when receiver is dropped (e.g., flight change)
            if throttle_texture_sender.send(tex_handle).is_ok() {
                ctx.request_repaint();
            }
        });

        self.time_texture_receiver = Some(time_texture_receiver);
        self.throttle_texture_receiver = Some(throttle_texture_receiver);
    }

    pub fn process_updates(&mut self) {
        let chunks_done = if let Some(receiver) = &self.chunk_receiver {
            loop {
                match receiver.try_recv() {
                    Ok(chunks) => {
                        self.chunks.extend(chunks);
                    }
                    Err(TryRecvError::Empty) => {
                        break false;
                    }
                    Err(TryRecvError::Disconnected) => {
                        break true;
                    }
                }
            }
        } else {
            false
        };

        if chunks_done {
            self.chunk_receiver = None;
            
            // Calculate average spectrum and detect peaks
            self.calculate_peaks();
            
            self.redraw_textures();
        }

        // Process time textures and clear receiver when done
        if let Some(receiver) = &self.time_texture_receiver {
            loop {
                match receiver.try_recv() {
                    Ok((t_start, t_end, tex)) => {
                        self.time_textures.push((t_start, t_end, tex));
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        self.time_texture_receiver = None;
                        break;
                    }
                }
            }
        }

        // Process throttle texture and clear receiver when done
        if let Some(receiver) = &self.throttle_texture_receiver {
            loop {
                match receiver.try_recv() {
                    Ok(texture) => {
                        self.throttle_texture = Some(texture);
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        self.throttle_texture_receiver = None;
                        break;
                    }
                }
            }
        }
    }
    
    /// Calculate average spectrum and detect peaks from all FFT chunks
    fn calculate_peaks(&mut self) {
        if self.chunks.is_empty() {
            self.detected_peaks.clear();
            self.average_spectrum = None;
            return;
        }
        
        let fft_len = self.chunks[0].fft.len();
        if fft_len < 3 {
            return;
        }
        
        // Calculate average spectrum across all chunks
        let mut avg_spectrum = vec![0.0f32; fft_len];
        for chunk in &self.chunks {
            for (i, &val) in chunk.fft.iter().enumerate() {
                if i < avg_spectrum.len() {
                    avg_spectrum[i] += val;
                }
            }
        }
        let chunk_count = self.chunks.len() as f32;
        for val in &mut avg_spectrum {
            *val /= chunk_count;
        }
        
        // Detect peaks in the average spectrum
        let sample_rate = self.flight_data.sample_rate() as f32;
        let min_freq = self.fft_settings.min_freq_hz;
        let max_freq = self.fft_settings.max_freq_hz.min(sample_rate / 2.0);
        
        self.detected_peaks = FftChunk::find_peaks(
            &avg_spectrum,
            sample_rate,
            min_freq.max(10.0), // Avoid very low frequencies
            max_freq,
            5, // Top 5 peaks
        );
        
        // Sort peaks by frequency for consistent display
        self.detected_peaks.sort_by(|a, b| a.frequency_hz.partial_cmp(&b.frequency_hz).unwrap_or(std::cmp::Ordering::Equal));
        
        self.average_spectrum = Some(avg_spectrum);
    }

    pub fn set_fft_settings(&mut self, fft_settings: FftSettings) {
        let old_fft_settings = self.fft_settings.clone();
        self.fft_settings = fft_settings;

        if self.fft_settings.needs_recalculating(&old_fft_settings) {
            self.recalculate_ffts();
        } else if self.fft_settings.needs_redrawing(&old_fft_settings) {
            self.redraw_textures();
        }
    }

    /// Returns true if FFT calculation or texture generation is in progress
    pub fn is_processing(&self) -> bool {
        self.chunk_receiver.is_some()
            || self.time_texture_receiver.is_some()
            || self.throttle_texture_receiver.is_some()
    }
    
    /// Get detected peaks for this axis
    pub fn get_peaks(&self) -> &[FrequencyPeak] {
        &self.detected_peaks
    }

    pub fn show_time(
        &mut self,
        ui: &mut egui::Ui,
        max_freq_override: Option<f32>,
        bands: &[FrequencyBand],
        show_rpm: bool,
        show_peaks: bool,
    ) -> egui::Response {
        self.process_updates();
        let nyquist = self.flight_data.sample_rate() / 2.0;

        let max_freq = nyquist;
        let display_max_freq = max_freq_override
            .unwrap_or(self.fft_settings.max_freq_hz)
            .min(nyquist as f32);
        let height = PLOT_HEIGHT;
        let width = ui.available_width();

        // Calculate normalized y bounds from frequency settings
        let min_y = (self.fft_settings.min_freq_hz as f64 / max_freq).min(0.99);
        let max_y = (display_max_freq as f64 / max_freq).clamp(min_y + 0.01, 1.0);
        
        // Clone peaks for use in closure
        let peaks_for_display: Vec<_> = if show_peaks {
            self.detected_peaks.iter()
                .filter(|p| p.frequency_hz >= self.fft_settings.min_freq_hz && p.frequency_hz <= display_max_freq)
                .cloned()
                .collect()
        } else {
            vec![]
        };

        egui_plot::Plot::new(ui.next_auto_id())
            .legend(egui_plot::Legend::default())
            .set_margin_fraction(egui::Vec2::new(0.0, 0.0))
            .show_grid(true)
            .allow_drag([true, false])
            .allow_zoom([true, false])
            .allow_scroll(false)
            .auto_bounds(egui::Vec2b::new(true, false))  // Auto-bounds X only, not Y
            .include_y(min_y)
            .include_y(max_y)
            .link_axis("time_vibes", true, true)
            .link_cursor("global_timeseries", true, false)
            .y_axis_position(egui_plot::HPlacement::Right)
            .y_axis_width(2)
            .y_axis_formatter(move |gm, _, _| format!("{:.0}Hz", gm.value * max_freq))
            .label_formatter(move |_name, val| format!("{:.0}Hz\n{:.3}s", val.y * max_freq, val.x))
            .height(height)
            .width(width)
            .show(ui, |plot_ui| {
                for (t_start, t_end, texture) in self.time_textures.iter() {
                    let center = (t_start + t_end) / 2.0;
                    let duration = t_end - t_start;
                    let plot_image = egui_plot::PlotImage::new(
                        texture,
                        egui_plot::PlotPoint::new(center, 0.5),
                        egui::Vec2::new(duration as f32, 1.0),
                    );

                    plot_ui.image(plot_image);
                }

                // Draw Annotations Overlay
                for band in bands {
                    if band.enabled {
                        let y_min = band.min_hz as f64 / max_freq as f64;
                        let y_max = band.max_hz as f64 / max_freq as f64;
                        let full_time = self.flight_data.times.last().copied().unwrap_or(100.0);
                        
                        // Fill band
                        let points = egui_plot::PlotPoints::new(vec![
                            [0.0, y_min], [full_time, y_min], [full_time, y_max], [0.0, y_max]
                        ]);
                        plot_ui.polygon(egui_plot::Polygon::new(points).fill_color(band.color).name(&band.name));
                    }
                }

                if show_rpm {
                     if let Some(rpm) = self.flight_data.electrical_rpm() {
                       if !rpm.is_empty() && !rpm[0].is_empty() {
                            let times = &self.flight_data.times;
                            let len = times.len().min(rpm[0].len());
                            
                            for harmonic in 1..=3 {
                                let points: egui_plot::PlotPoints = (0..len).step_by(5).map(|i| {
                                     let mut sum_erpm = 0.0;
                                     let mut count = 0;
                                     for m in 0..rpm.len() {
                                         if let Some(val) = rpm[m].get(i) {
                                             sum_erpm += val;
                                             count += 1;
                                         }
                                     }
                                     let avg_erpm = if count > 0 { sum_erpm / count as f32 } else { 0.0 };
                                     // Use configurable motor poles: Hz = eRPM / 60 / (poles/2)
                                     let motor_poles = self.fft_settings.motor_poles as f32;
                                     let mech_hz = avg_erpm / 60.0 / (motor_poles / 2.0);
                                     [times[i], (mech_hz * harmonic as f32 / max_freq as f32) as f64]
                                }).collect();
                                
                               plot_ui.line(egui_plot::Line::new(points)
                                   .color(egui::Color32::from_rgba_unmultiplied(255, 255, 0, 150))
                                   .width(1.5)
                                   .name(format!("{}x Motor", harmonic)));
                            }
                       }
                   }
                }
                
                // Draw detected peak frequency lines
                for (i, peak) in peaks_for_display.iter().enumerate() {
                    let y_norm = peak.frequency_hz as f64 / max_freq;
                    let full_time = self.flight_data.times.last().copied().unwrap_or(100.0);
                    
                    // Use cyan color with varying alpha based on prominence
                    let alpha = ((peak.prominence.min(10.0) / 10.0) * 180.0 + 75.0) as u8;
                    let color = egui::Color32::from_rgba_unmultiplied(0, 255, 255, alpha);
                    
                    let points = egui_plot::PlotPoints::new(vec![
                        [0.0, y_norm], [full_time, y_norm]
                    ]);
                    plot_ui.line(egui_plot::Line::new(points)
                        .color(color)
                        .width(1.0)
                        .style(egui_plot::LineStyle::dashed_dense())
                        .name(format!("Peak {}: {:.0}Hz", i + 1, peak.frequency_hz)));
                }
            })
            .response
    }

    pub fn show_throttle(
        &mut self,
        ui: &mut egui::Ui,
        max_freq_override: Option<f32>,
        bands: &[FrequencyBand],
        show_rpm: bool,
        show_peaks: bool,
    ) -> egui::Response {
        self.process_updates();
        let nyquist = self.flight_data.sample_rate() / 2.0;

        let max_freq = nyquist;
        let display_max_freq = max_freq_override
            .unwrap_or(self.fft_settings.max_freq_hz)
            .min(nyquist as f32);
        let height = PLOT_HEIGHT;
        let width = ui.available_width();

        // Calculate normalized y bounds from frequency settings
        let min_y = (self.fft_settings.min_freq_hz as f64 / max_freq).min(0.99);
        let max_y = (display_max_freq as f64 / max_freq).clamp(min_y + 0.01, 1.0);
        
        // Clone peaks for use in closure
        let peaks_for_display: Vec<_> = if show_peaks {
            self.detected_peaks.iter()
                .filter(|p| p.frequency_hz >= self.fft_settings.min_freq_hz && p.frequency_hz <= display_max_freq)
                .cloned()
                .collect()
        } else {
            vec![]
        };

        egui_plot::Plot::new(ui.next_auto_id())
            .legend(egui_plot::Legend::default())
            .set_margin_fraction(egui::Vec2::new(0.0, 0.0))
            .show_grid(true)
            .allow_drag([true, false])
            .allow_zoom([true, false])
            .allow_scroll(false)
            .auto_bounds(egui::Vec2b::new(true, false))  // Auto-bounds X only, not Y
            .include_y(min_y)
            .include_y(max_y)
            .link_axis("throttle_vibes", true, true)
            .link_cursor("throttle_vibes", true, true)
            .x_axis_formatter(move |gm, _, _| format!("{:.0}%", gm.value * 100.0))
            .y_axis_position(egui_plot::HPlacement::Right)
            .y_axis_width(2)
            .y_axis_formatter(move |gm, _, _| format!("{:.0}Hz", gm.value * max_freq))
            .label_formatter(move |_, val| {
                format!("{:.0}Hz\n{:.0}%", val.y * max_freq, val.x * 100.0)
            })
            .height(height)
            .width(width)
            .reset()
            .show(ui, |plot_ui| {
                if let Some(texture) = self.throttle_texture.as_mut() {
                    let plot_image = egui_plot::PlotImage::new(
                        texture,
                        egui_plot::PlotPoint::new(0.5, 0.5),
                        egui::Vec2::new(1.0, 1.0),
                    );

                    plot_ui.image(plot_image);
                }

                // Draw Annotations Overlay
                for band in bands {
                    if band.enabled {
                        let y_min = band.min_hz as f64 / max_freq as f64;
                        let y_max = band.max_hz as f64 / max_freq as f64;
                        // Full throttle range 0.0 to 1.0
                        let points = egui_plot::PlotPoints::new(vec![
                            [0.0, y_min], [1.0, y_min], [1.0, y_max], [0.0, y_max]
                        ]);
                        plot_ui.polygon(egui_plot::Polygon::new(points).fill_color(band.color).name(&band.name));
                    }
                }

                if show_rpm {
                     if let Some(rpm) = self.flight_data.electrical_rpm() {
                       // Find approximate max RPM
                       let mut max_erpm = 0.0f32;
                       for m in rpm {
                           for &val in m.iter().step_by(100) { // sparse check
                               if val > max_erpm { max_erpm = val; }
                           }
                       }
                       
                       if max_erpm > 0.0 {
                            // Use configurable motor poles: Hz = eRPM / 60 / (poles/2)
                            let motor_poles = self.fft_settings.motor_poles as f32;
                            let max_mech_hz = max_erpm / 60.0 / (motor_poles / 2.0);
                            for harmonic in 1..=3 {
                                // Linear approximation: Hz = MaxHz * Throttle
                                let points = egui_plot::PlotPoints::new(vec![
                                    [0.0, 0.0],
                                    [1.0, (max_mech_hz * harmonic as f32 / max_freq as f32) as f64]
                                ]);
                                
                               plot_ui.line(egui_plot::Line::new(points)
                                   .color(Color32::from_rgba_unmultiplied(255, 255, 0, 150))
                                   .width(1.5)
                                   .name(format!("{}x Motor (Est)", harmonic)));
                            }
                       }
                   }
                }
                
                // Draw detected peak frequency lines
                for (i, peak) in peaks_for_display.iter().enumerate() {
                    let y_norm = peak.frequency_hz as f64 / max_freq;
                    
                    // Use cyan color with varying alpha based on prominence
                    let alpha = ((peak.prominence.min(10.0) / 10.0) * 180.0 + 75.0) as u8;
                    let color = egui::Color32::from_rgba_unmultiplied(0, 255, 255, alpha);
                    
                    let points = egui_plot::PlotPoints::new(vec![
                        [0.0, y_norm], [1.0, y_norm]
                    ]);
                    plot_ui.line(egui_plot::Line::new(points)
                        .color(color)
                        .width(1.0)
                        .style(egui_plot::LineStyle::dashed_dense())
                        .name(format!("Peak {}: {:.0}Hz", i + 1, peak.frequency_hz)));
                }
            })


            .response
    }

    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        domain: VibeDomain,
        max_freq_override: Option<f32>,
        bands: &[FrequencyBand],
        show_rpm: bool,
        show_peaks: bool,
    ) -> egui::Response {
        self.process_updates();

        if self.chunks.is_empty() {
            ui.label("")
        } else {
            match domain {
                VibeDomain::Time => self.show_time(ui, max_freq_override, bands, show_rpm, show_peaks),
                VibeDomain::Throttle => self.show_throttle(ui, max_freq_override, bands, show_rpm, show_peaks),
            }
        }
    }

}

struct FftVectorSeries {
    axes: [FftAxis; 3],
    /// If true, limit display to sub-100Hz (like PIDtoolbox "<100Hz" option)
    sub_100hz: bool,
}

impl FftVectorSeries {
    pub fn new(
        ctx: &egui::Context,
        fft_settings: FftSettings,
        fd: Arc<FlightData>,
        value_callback: fn(&FlightData) -> [Option<&Vec<f32>>; 3],
    ) -> Self {
        let axes = [
            FftAxis::new(ctx, fft_settings.clone(), 0, fd.clone(), value_callback),
            FftAxis::new(ctx, fft_settings.clone(), 1, fd.clone(), value_callback),
            FftAxis::new(ctx, fft_settings.clone(), 2, fd, value_callback),
        ];

        Self {
            axes,
            sub_100hz: false, // Default to full spectrum
        }
    }

    pub fn set_fft_settings(&mut self, fft_settings: FftSettings) {
        self.axes[0].set_fft_settings(fft_settings.clone());
        self.axes[1].set_fft_settings(fft_settings.clone());
        self.axes[2].set_fft_settings(fft_settings);
    }

    /// Set the sub-100Hz display mode for this series
    pub fn set_sub_100hz(&mut self, sub_100hz: bool) {
        self.sub_100hz = sub_100hz;
    }

    /// Get the effective max frequency for display
    pub fn effective_max_freq(&self, nyquist: f32) -> f32 {
        if self.sub_100hz {
            100.0
        } else {
            nyquist.min(self.axes[0].fft_settings.max_freq_hz)
        }
    }

    /// Returns true if any axis is currently processing FFT data
    pub fn is_processing(&self) -> bool {
        self.axes.iter().any(|axis| axis.is_processing())
    }

    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        domain: VibeDomain,
        _show_labels: bool,
        bands: &[FrequencyBand],
        show_rpm: bool,
        show_peaks: bool,
    ) -> egui::Response {
        let available_height = ui.available_height();
        // Reserve space for X-axis label at bottom
        let x_label_height = 20.0;
        let plots_height = available_height - x_label_height;
        let row_height = (plots_height / 3.0).max(PLOT_HEIGHT);

        // Y-axis label width
        let y_label_width = 30.0;

        // Calculate max frequency override based on sub_100hz setting
        let max_freq_override = if self.sub_100hz { Some(100.0f32) } else { None };

        ui.vertical(|ui| {
            for (i, axis) in self.axes.iter_mut().enumerate() {
                ui.horizontal(|ui| {
                    // Y-axis label on the left (rotated text) - outside the chart
                    ui.allocate_ui_with_layout(
                        egui::vec2(y_label_width, row_height),
                        egui::Layout::centered_and_justified(egui::Direction::TopDown),
                        |ui| {
                            // Draw rotated text for Y-axis label
                            let label_text = axis.axis_name;
                            let (rect, _response) = ui.allocate_exact_size(
                                egui::vec2(y_label_width, row_height),
                                egui::Sense::hover(),
                            );
                            let painter = ui.painter();

                            // Draw vertical text (rotated 90 degrees)
                            let _galley = painter.layout_no_wrap(
                                label_text.to_string(),
                                egui::FontId::proportional(11.0),
                                egui::Color32::WHITE,
                            );

                            // Calculate position for centered rotated text
                            let text_pos = rect.center();
                            painter.text(
                                text_pos,
                                egui::Align2::CENTER_CENTER,
                                label_text,
                                egui::FontId::proportional(11.0),
                                egui::Color32::WHITE,
                            );
                        },
                    );

                    // The chart plot
                    ui.allocate_ui_with_layout(
                        egui::vec2(ui.available_width(), row_height),
                        egui::Layout::left_to_right(egui::Align::Center),
                        |ui| {
                            match domain {
                                VibeDomain::Time => axis.show_time(ui, max_freq_override, bands, show_rpm, show_peaks),
                                VibeDomain::Throttle => axis.show_throttle(ui, max_freq_override, bands, show_rpm, show_peaks),
                            };
                        },
                    );
                });

                // Add X-axis label only for the last row
                if i == 2 {
                    ui.horizontal(|ui| {
                        // Empty space for Y-axis alignment
                        ui.add_space(y_label_width);

                        // X-axis label centered below the chart
                        ui.allocate_ui_with_layout(
                            egui::vec2(ui.available_width(), x_label_height),
                            egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
                            |ui| {
                                let x_label = match domain {
                                    VibeDomain::Throttle => "% Throttle",
                                    VibeDomain::Time => "Time (s)",
                                };
                                ui.label(
                                    egui::RichText::new(x_label)
                                        .color(egui::Color32::LIGHT_GRAY)
                                        .size(10.0),
                                );
                            },
                        );
                    });
                }
            }
        })
        .response
    }
    
    /// Get detected peaks for all axes
    pub fn get_all_peaks(&self) -> [&[FrequencyPeak]; 3] {
        [
            self.axes[0].get_peaks(),
            self.axes[1].get_peaks(),
            self.axes[2].get_peaks(),
        ]
    }

}

pub struct VibeTab {
    domain: VibeDomain,

    gyro_raw_enabled: bool,
    gyro_filtered_enabled: bool,
    dterm_raw_enabled: bool,
    dterm_filtered_enabled: bool,
    setpoint_enabled: bool,
    pid_error_enabled: bool,
    pid_sum_enabled: bool,

    fft_settings: FftSettings,
    max_freq_hz: f32,
    min_freq_hz: f32,
    current_preset: AnalysisPreset,
    /// Show detected peak frequencies as dashed lines on spectrograms
    show_peak_frequencies: bool,
    #[allow(dead_code)]
    show_axis_labels: bool,
    show_rpm_harmonics: bool,

    // Frequency band overlays
    frequency_bands: Vec<FrequencyBand>,

    gyro_raw_ffts: FftVectorSeries,
    gyro_filtered_ffts: FftVectorSeries,
    dterm_raw_ffts: FftVectorSeries, // [NEW] Added D-Term Raw series
    dterm_filtered_ffts: FftVectorSeries,
    setpoint_ffts: FftVectorSeries,
    pid_error_ffts: FftVectorSeries,
    pid_sum_ffts: FftVectorSeries,

    // Cached computed data (needed for FFT callbacks)
    // removed pid_error_data and pid_sum_data as they are now in FlightData

    fd: Arc<FlightData>,
}

// Static storage for computed data (needed for FFT callbacks)

impl VibeTab {
    pub fn new(ctx: &egui::Context, fd: Arc<FlightData>) -> Self {
        let mut fft_settings = FftSettings::default();
        
        // Try to read motor pole count from log headers (default to 14)
        if let Some(poles) = fd.motor_poles() {
            fft_settings.motor_poles = poles;
        }

        // TODO: unwrap
        let gyro_raw_ffts =
            FftVectorSeries::new(ctx, fft_settings.clone(), fd.clone(), |fd: &FlightData| {
                match fd.gyro_unfiltered() {
                    Some(data) => [Some(data[0]), Some(data[1]), Some(data[2])],
                    None => [None, None, None]
                }
            });
        let gyro_filtered_ffts =
            FftVectorSeries::new(ctx, fft_settings.clone(), fd.clone(), |fd: &FlightData| {
                match fd.gyro_filtered() {
                    Some(data) => [Some(data[0]), Some(data[1]), Some(data[2])],
                    None => [None, None, None]
                }
            });

        // D-term pre-filter: try direct data first, then use calculated from gyro derivative
        let d_raw_direct = fd.d_unfiltered();
        let has_direct_d_raw = d_raw_direct.iter().any(|opt| opt.is_some());
        let has_calculated_d_raw = fd.d_unfiltered_calculated().is_some();

        let dterm_raw_ffts =
            FftVectorSeries::new(ctx, fft_settings.clone(), fd.clone(), |fd: &FlightData| {
                // First try direct data
                let direct = fd.d_unfiltered();
                if direct.iter().any(|opt| opt.is_some()) {
                    return direct;
                }
                
                // Fall back to calculated (now pre-computed in FlightData)
                match fd.d_unfiltered_calculated() {
                    Some(data) => [Some(data[0]), Some(data[1]), Some(data[2])],
                    None => [None, None, None],
                }
            });

        let dterm_filtered_ffts =
            FftVectorSeries::new(ctx, fft_settings.clone(), fd.clone(), |fd: &FlightData| {
                fd.d()
            });

        // Setpoint FFT (Roll, Pitch, Yaw only - first 3 elements)
        let setpoint_ffts =
            FftVectorSeries::new(ctx, fft_settings.clone(), fd.clone(), |fd: &FlightData| {
                let sp = fd.setpoint();
                match sp {
                    Some(s) => [Some(s[0]), Some(s[1]), Some(s[2])],
                    None => [None, None, None],
                }
            });

        let pid_error_ffts =
            FftVectorSeries::new(ctx, fft_settings.clone(), fd.clone(), |fd: &FlightData| {
                match fd.pid_error() {
                    Some(e) => [Some(e[0]), Some(e[1]), Some(e[2])],
                    None => [None, None, None],
                }
            });

        let pid_sum_ffts =
            FftVectorSeries::new(ctx, fft_settings.clone(), fd.clone(), |fd: &FlightData| {
                match fd.pid_sum() {
                    Some(s) => [Some(s[0]), Some(s[1]), Some(s[2])],
                    None => [None, None, None],
                }
            });

        Self {
            domain: VibeDomain::Throttle,

            gyro_raw_enabled: fd
                .gyro_unfiltered()
                .map(|v| !v[0].is_empty())
                .unwrap_or(false),
            gyro_filtered_enabled: true,
            dterm_raw_enabled: has_direct_d_raw || has_calculated_d_raw,
            dterm_filtered_enabled: true,
            setpoint_enabled: false,
            pid_error_enabled: false,
            pid_sum_enabled: false,

            fft_settings,
            max_freq_hz: fd.sample_rate() as f32 / 2.0, // Nyquist
            min_freq_hz: 0.0,                           // Default to showing all frequencies
            current_preset: AnalysisPreset::Custom,
            show_peak_frequencies: false,

            show_axis_labels: true,
            show_rpm_harmonics: false, // Default off

            // Initialize frequency bands with common ranges
            frequency_bands: vec![
                FrequencyBand::new(
                    "Prop Wash",
                    20.0,
                    80.0,
                    Color32::from_rgba_unmultiplied(255, 165, 0, 80),
                ),
                FrequencyBand::new(
                    "Frame Resonance",
                    80.0,
                    150.0,
                    Color32::from_rgba_unmultiplied(255, 0, 0, 80),
                ),
                FrequencyBand::new(
                    "Motor Noise",
                    150.0,
                    300.0,
                    Color32::from_rgba_unmultiplied(0, 255, 0, 80),
                ),
            ],

            gyro_raw_ffts,
            gyro_filtered_ffts,
            dterm_raw_ffts,
            dterm_filtered_ffts,
            setpoint_ffts,
            pid_error_ffts,
            pid_sum_ffts,

            fd,
        }
    }

    pub fn update_fft_settings(&mut self) {
        self.gyro_raw_ffts
            .set_fft_settings(self.fft_settings.clone());
        self.gyro_filtered_ffts
            .set_fft_settings(self.fft_settings.clone());
        self.dterm_raw_ffts
            .set_fft_settings(self.fft_settings.clone());
        self.dterm_filtered_ffts
            .set_fft_settings(self.fft_settings.clone());
        self.setpoint_ffts
            .set_fft_settings(self.fft_settings.clone());
        self.pid_error_ffts
            .set_fft_settings(self.fft_settings.clone());
        self.pid_sum_ffts
            .set_fft_settings(self.fft_settings.clone());
    }

    /// Get debug mode information from log headers
    fn get_debug_mode_info(&self) -> Option<(String, String, egui::Color32)> {
        let debug_mode = self.fd.unknown_headers.get("debug_mode")?;

        let (description, color) = match debug_mode.to_uppercase().as_str() {
            "GYRO_SCALED" | "GYRO" => (
                "Shows pre vs post-filter gyro data. Compare raw gyro noise to filtered output to evaluate filter effectiveness.",
                egui::Color32::from_rgb(100, 200, 255)
            ),
            "D_LPF" | "DTERM_FILTER" => (
                "D-term filter debugging. Shows D-term before and after filtering - useful for optimizing D-term LPF settings.",
                egui::Color32::from_rgb(255, 180, 100)
            ),
            "RPM_FILTER" => (
                "RPM filter debugging. Shows motor harmonics and filter effectiveness at removing motor noise.",
                egui::Color32::from_rgb(100, 255, 180)
            ),
            "DYNAMIC_FILTER" | "DYN_NOTCH" => (
                "Dynamic notch filter tracking. Shows how the notch filters are following frame resonances.",
                egui::Color32::from_rgb(180, 100, 255)
            ),
            "GYRO_RAW" => (
                "Raw gyro output before any filtering. Useful for seeing all noise sources.",
                egui::Color32::from_rgb(255, 100, 100)
            ),
            "FEEDFORWARD" | "FF" => (
                "Feedforward debugging. Shows feedforward contribution to overall PID output.",
                egui::Color32::from_rgb(255, 255, 100)
            ),
            "PIDLOOP" | "PID_LOOP" => (
                "Full PID loop timing. Shows loop execution times for performance analysis.",
                egui::Color32::from_rgb(200, 200, 200)
            ),
            _ => (
                "Debug data available. Check debug channels in log for specialized analysis.",
                egui::Color32::from_rgb(150, 150, 150)
            ),
        };

        Some((debug_mode.clone(), description.to_string(), color))
    }

    /// Returns true if any FFT series is currently processing
    pub fn is_any_processing(&self) -> bool {
        self.gyro_raw_ffts.is_processing()
            || self.gyro_filtered_ffts.is_processing()
            || self.dterm_raw_ffts.is_processing()
            || self.dterm_filtered_ffts.is_processing()
            || self.setpoint_ffts.is_processing()
            || self.pid_error_ffts.is_processing()
            || self.pid_sum_ffts.is_processing()
    }

    pub fn show(&mut self, ui: &mut egui::Ui) {
        let old_fft_settings = self.fft_settings.clone();
        let old_preset = self.current_preset;
        let fft_size = self.fft_settings.size;
        let sample_rate = self.fd.sample_rate() as f32;
        let nyquist = sample_rate / 2.0;

        // === DEBUG MODE INFO PANEL ===
        if let Some((mode_name, description, color)) = self.get_debug_mode_info() {
            egui::Frame::none()
                .fill(egui::Color32::from_black_alpha(60))
                .rounding(4.0)
                .inner_margin(8.0)
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("[DBG] Debug Mode:").strong());
                        ui.label(egui::RichText::new(&mode_name).color(color).strong());
                    });
                    ui.label(egui::RichText::new(&description).weak().small());
                });
            ui.add_space(8.0);
        }

        // === TOOLBAR WITH HORIZONTAL SCROLL ===
        egui::ScrollArea::horizontal()
            .id_source("vibe_toolbar")
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    // Domain & Presets
                    ui.label("Domain:");
                    ui.selectable_value(&mut self.domain, VibeDomain::Time, "Time").on_hover_text("Show frequency vs time");
                    ui.selectable_value(&mut self.domain, VibeDomain::Throttle, "Throttle").on_hover_text("Show frequency vs throttle");
                    ui.separator();
                    ui.label("Presets:");
                    ui.selectable_value(&mut self.current_preset, AnalysisPreset::MotorTuning, "Motor");
                    ui.selectable_value(&mut self.current_preset, AnalysisPreset::PropWash, "Prop");
                    ui.selectable_value(&mut self.current_preset, AnalysisPreset::FullSpectrum, "Full");
                    ui.separator();
                    
                    // Series toggles
                    ui.label("Series:");
                    ui.toggle_value(&mut self.gyro_raw_enabled, "Gyro Raw");
                    ui.toggle_value(&mut self.gyro_filtered_enabled, "Gyro Filt");
                    ui.toggle_value(&mut self.dterm_raw_enabled, "D Raw");
                    ui.toggle_value(&mut self.dterm_filtered_enabled, "D Filt");
                    ui.toggle_value(&mut self.setpoint_enabled, "Setpoint");
                    ui.toggle_value(&mut self.pid_error_enabled, "PID Err");
                    ui.toggle_value(&mut self.pid_sum_enabled, "PID Sum");
                    ui.separator();
                    
                    // FFT settings
                    ui.label("FFT:");
                    for size in &FFT_SIZE_OPTIONS {
                        ui.selectable_value(&mut self.fft_settings.size, *size, format!("{}", size));
                    }
                    ui.separator();
                    ui.label("Step:");
                    for value in &[1usize, 8, 32, 128] {
                        ui.add_enabled_ui(*value <= fft_size, |ui| {
                            ui.selectable_value(&mut self.fft_settings.step_size, *value, format!("{}", value));
                        });
                    }
                    ui.separator();
                    
                    // Frequency range
                    ui.label("Freq:");
                    for freq in &[100.0_f32, 250.0, 500.0] {
                        let label = if *freq == 100.0 { "<100".to_string() } else { format!("{:.0}", freq) };
                        ui.selectable_value(&mut self.max_freq_hz, *freq, &label);
                    }
                    ui.selectable_value(&mut self.max_freq_hz, nyquist, format!("Full ({:.0})", nyquist));
                    ui.separator();
                    ui.label("Min:");
                    ui.add(egui::DragValue::new(&mut self.min_freq_hz).clamp_range(0.0..=self.max_freq_hz - 1.0).speed(1.0).suffix(" Hz"));
                    ui.separator();
                    
                    // Color scheme
                    ui.label("Color:");
                    egui::ComboBox::from_id_source("colorscheme")
                        .selected_text(format!("{:?}", self.fft_settings.plot_colorscheme))
                        .width(70.0)
                        .show_ui(ui, |ui| {
                            for value in Colorscheme::ALL {
                                ui.selectable_value(&mut self.fft_settings.plot_colorscheme, value, format!("{:?}", value));
                            }
                        });
                    ui.separator();
                    
                    // Scale settings
                    ui.checkbox(&mut self.fft_settings.auto_scale, "Auto").on_hover_text("Auto normalize color scale");
                    if !self.fft_settings.auto_scale {
                        ui.add(egui::DragValue::new(&mut self.fft_settings.plot_max).clamp_range(0.01..=2.0).speed(0.01).fixed_decimals(2)).on_hover_text("Color scale max");
                    }
                    ui.separator();
                    ui.label("Smooth:");
                    ui.add(egui::DragValue::new(&mut self.fft_settings.smooth_factor).clamp_range(1u8..=5).speed(0.1));
                    ui.separator();
                    ui.label("Poles:");
                    ui.add(egui::DragValue::new(&mut self.fft_settings.motor_poles).clamp_range(2u8..=36).speed(0.5));
                    ui.separator();
                    
                    // Amplitude scale
                    ui.selectable_value(&mut self.fft_settings.amplitude_scale, AmplitudeScale::Linear, "Lin").on_hover_text("Linear amplitude");
                    ui.selectable_value(&mut self.fft_settings.amplitude_scale, AmplitudeScale::DeciBel, "dB").on_hover_text("Decibel (PSD)");
                    ui.separator();
                    
                    // Overlays
                    ui.checkbox(&mut self.show_rpm_harmonics, "RPM").on_hover_text("Show motor harmonics");
                    ui.checkbox(&mut self.show_peak_frequencies, "Peaks").on_hover_text("Show detected peaks");
                    ui.menu_button(icons::CARET_DOWN, |ui| {
                        ui.label("Frequency Bands:");
                        for band in &mut self.frequency_bands {
                            ui.horizontal(|ui| {
                                ui.checkbox(&mut band.enabled, &band.name);
                                ui.color_edit_button_srgba(&mut band.color);
                            });
                        }
                    }).response.on_hover_text("Frequency bands");
                });
            });


        // Apply preset settings only when preset changes
        if old_preset != self.current_preset {
            match self.current_preset {
                AnalysisPreset::MotorTuning => {
                    self.max_freq_hz = 500.0;
                    self.fft_settings.size = 512;
                    self.fft_settings.step_size = 8;
                }
                AnalysisPreset::PropWash => {
                    self.max_freq_hz = 100.0;
                    self.fft_settings.size = 1024;
                    self.fft_settings.step_size = 32;
                }
                AnalysisPreset::FullSpectrum => {
                    self.max_freq_hz = nyquist;
                    self.fft_settings.size = 256;
                    self.fft_settings.step_size = 8;
                }
                AnalysisPreset::Custom => {}
            }
        } else if self.fft_settings != old_fft_settings {
            // If user manually changed settings while on a preset, switch to Custom
            self.current_preset = AnalysisPreset::Custom;
        }

        // Sync frequency bounds to fft_settings
        self.fft_settings.min_freq_hz = self.min_freq_hz;
        self.fft_settings.max_freq_hz = self.max_freq_hz;

        if self.fft_settings != old_fft_settings {
            self.fft_settings.step_size =
                usize::min(self.fft_settings.step_size, self.fft_settings.size);
            self.update_fft_settings();
        }

        ui.separator();
        
        // === PEAK FREQUENCIES PANEL ===
        if self.show_peak_frequencies {
            egui::CollapsingHeader::new("Detected Peak Frequencies")
                .default_open(true)
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        // Collect all series with their peaks
                        let series_list: Vec<(&str, &FftVectorSeries)> = vec![
                            ("Gyro Raw", &self.gyro_raw_ffts),
                            ("Gyro Filt", &self.gyro_filtered_ffts),
                            ("D Raw", &self.dterm_raw_ffts),
                            ("D Filt", &self.dterm_filtered_ffts),
                        ];
                        
                        for (name, series) in series_list {
                            let peaks = series.get_all_peaks();
                            let has_peaks = peaks.iter().any(|p| !p.is_empty());
                            
                            if has_peaks {
                                egui::Frame::none()
                                    .fill(egui::Color32::from_black_alpha(40))
                                    .rounding(4.0)
                                    .inner_margin(4.0)
                                    .show(ui, |ui| {
                                        ui.vertical(|ui| {
                                            ui.label(egui::RichText::new(name).strong().size(11.0));
                                            for (axis_idx, axis_peaks) in peaks.iter().enumerate() {
                                                if !axis_peaks.is_empty() {
                                                    let axis_name = AXIS_NAMES[axis_idx];
                                                    let peak_str: String = axis_peaks.iter()
                                                        .take(3)
                                                        .map(|p| format!("{:.0}Hz", p.frequency_hz))
                                                        .collect::<Vec<_>>()
                                                        .join(", ");
                                                    ui.label(
                                                        egui::RichText::new(format!("{}: {}", axis_name, peak_str))
                                                            .size(10.0)
                                                            .color(egui::Color32::from_rgb(0, 255, 255))
                                                    );
                                                }
                                            }
                                        });
                                    });
                            }
                        }
                    });
                });
            ui.separator();
        }

        // Determine the first enabled column to show axis labels only there
        let _first_enabled = if self.gyro_raw_enabled {
            0
        } else if self.gyro_filtered_enabled {
            1
        } else if self.dterm_raw_enabled {
            2
        } else if self.dterm_filtered_enabled {
            3
        } else if self.setpoint_enabled {
            4
        } else if self.pid_error_enabled {
            5
        } else if self.pid_sum_enabled {
            6
        } else {
            usize::MAX // No columns enabled
        };

        // Display columns with equal widths like PIDToolbox
        let enabled_columns: Vec<(bool, &str, &mut FftVectorSeries, usize)> = vec![
            (
                self.gyro_raw_enabled,
                "Gyro (raw)",
                &mut self.gyro_raw_ffts,
                0,
            ),
            (
                self.gyro_filtered_enabled,
                "Gyro (filtered)",
                &mut self.gyro_filtered_ffts,
                1,
            ),
            (
                self.dterm_raw_enabled,
                "D Term (raw)",
                &mut self.dterm_raw_ffts,
                2,
            ),
            (
                self.dterm_filtered_enabled,
                "D Term (filtered)",
                &mut self.dterm_filtered_ffts,
                3,
            ),
            (
                self.setpoint_enabled,
                "Setpoint",
                &mut self.setpoint_ffts,
                4,
            ),
            (
                self.pid_error_enabled,
                "PID Error",
                &mut self.pid_error_ffts,
                5,
            ),
            (self.pid_sum_enabled, "PID Sum", &mut self.pid_sum_ffts, 6),
        ];

        let active_columns: Vec<_> = enabled_columns
            .into_iter()
            .filter(|(enabled, _, _, _)| *enabled)
            .collect();

        if active_columns.is_empty() {
            return;
        }

        let num_cols = active_columns.len();
        let domain = self.domain;

        // Fixed width per column to ensure proper display and scrolling
        let col_width = 300.0;
        let total_width = num_cols as f32 * col_width;

        egui::ScrollArea::horizontal()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    for (i, (_, title, fft_series, _)) in active_columns.into_iter().enumerate() {
                        ui.allocate_ui_with_layout(
                            egui::vec2(col_width, ui.available_height()),
                            egui::Layout::top_down(egui::Align::LEFT),
                            |ui| {
                                // Column header with title and <100Hz checkbox
                                ui.horizontal(|ui| {
                                    ui.heading(title);
                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            let mut sub_100hz = fft_series.sub_100hz;
                                            if ui.checkbox(&mut sub_100hz, "<100Hz").changed() {
                                                fft_series.set_sub_100hz(sub_100hz);
                                            }
                                        },
                                    );
                                });
                                // Draw colorbar legend
                                self.fft_settings.draw_colorbar(ui, col_width - 10.0);
                                fft_series.show(ui, domain, i == 0, &self.frequency_bands, self.show_rpm_harmonics, self.show_peak_frequencies);
                            },
                        );
                    }

                });
            });
    }
}

use std::sync::Arc;

use egui::{Color32, RichText, Stroke};
use egui_plot::{Corner, Legend, Line, Plot, PlotPoints, VLine};

use crate::flight_data::FlightData;
use crate::gui::colors::Colors;
use crate::gui::flex::FlexColumns;

use super::{MIN_WIDE_WIDTH, PLOT_HEIGHT};

/// Filter settings parsed from log headers
#[derive(Clone, Default)]
#[allow(dead_code)]
pub struct FilterSettings {
    // Gyro lowpass filters
    pub gyro_lpf1_type: String,
    pub gyro_lpf1_hz: f32,
    pub gyro_lpf1_dyn_min: f32,
    pub gyro_lpf1_dyn_max: f32,
    pub gyro_lpf2_type: String,
    pub gyro_lpf2_hz: f32,
    
    // D-term lowpass filters
    pub dterm_lpf1_type: String,
    pub dterm_lpf1_hz: f32,
    pub dterm_lpf1_dyn_min: f32,
    pub dterm_lpf1_dyn_max: f32,
    pub dterm_lpf2_type: String,
    pub dterm_lpf2_hz: f32,
    
    // Notch filters
    pub gyro_notch1_hz: f32,
    pub gyro_notch1_cutoff: f32,
    pub gyro_notch2_hz: f32,
    pub gyro_notch2_cutoff: f32,
    pub dterm_notch_hz: f32,
    pub dterm_notch_cutoff: f32,
    
    // RPM filter settings
    pub rpm_filter_harmonics: u8,
    pub rpm_filter_q: f32,
    pub rpm_filter_min_hz: f32,
    pub rpm_filter_fade_range_hz: f32,
    
    // Sample rate
    pub gyro_sample_rate: f32,
    pub pid_loop_rate: f32,
}

impl FilterSettings {
    /// Parse filter settings from FlightData headers
    pub fn parse_from_headers(fd: &FlightData) -> Self {
        let headers = &fd.unknown_headers;
        
        let get_f32 = |key: &str| -> f32 {
            headers.get(key)
                .and_then(|v| v.parse::<f32>().ok())
                .unwrap_or(0.0)
        };
        
        let get_u8 = |key: &str| -> u8 {
            headers.get(key)
                .and_then(|v| v.parse::<u8>().ok())
                .unwrap_or(0)
        };
        
        let get_str = |key: &str| -> String {
            headers.get(key).cloned().unwrap_or_default()
        };
        
        Self {
            // Gyro LPF
            gyro_lpf1_type: get_str("gyro_lpf1_type").to_uppercase(),
            gyro_lpf1_hz: get_f32("gyro_lpf1_static_hz").max(get_f32("gyro_lowpass_hz")),
            gyro_lpf1_dyn_min: get_f32("gyro_lpf1_dyn_min_hz").max(get_f32("dyn_lpf_gyro_min_hz")),
            gyro_lpf1_dyn_max: get_f32("gyro_lpf1_dyn_max_hz").max(get_f32("dyn_lpf_gyro_max_hz")),
            gyro_lpf2_type: get_str("gyro_lpf2_type").to_uppercase(),
            gyro_lpf2_hz: get_f32("gyro_lpf2_static_hz").max(get_f32("gyro_lowpass2_hz")),
            
            // D-term LPF
            dterm_lpf1_type: get_str("dterm_lpf1_type").to_uppercase(),
            dterm_lpf1_hz: get_f32("dterm_lpf1_static_hz").max(get_f32("dterm_lowpass_hz")),
            dterm_lpf1_dyn_min: get_f32("dterm_lpf1_dyn_min_hz").max(get_f32("dyn_lpf_dterm_min_hz")),
            dterm_lpf1_dyn_max: get_f32("dterm_lpf1_dyn_max_hz").max(get_f32("dyn_lpf_dterm_max_hz")),
            dterm_lpf2_type: get_str("dterm_lpf2_type").to_uppercase(),
            dterm_lpf2_hz: get_f32("dterm_lpf2_static_hz").max(get_f32("dterm_lowpass2_hz")),
            
            // Notch filters
            gyro_notch1_hz: get_f32("gyro_notch1_hz"),
            gyro_notch1_cutoff: get_f32("gyro_notch1_cutoff"),
            gyro_notch2_hz: get_f32("gyro_notch2_hz"),
            gyro_notch2_cutoff: get_f32("gyro_notch2_cutoff"),
            dterm_notch_hz: get_f32("dterm_notch_hz"),
            dterm_notch_cutoff: get_f32("dterm_notch_cutoff"),
            
            // RPM filter
            rpm_filter_harmonics: get_u8("rpm_filter_harmonics").max(get_u8("gyro_rpm_notch_harmonics")),
            rpm_filter_q: get_f32("rpm_filter_q").max(get_f32("gyro_rpm_notch_q")).max(500.0) / 100.0,
            rpm_filter_min_hz: get_f32("rpm_filter_min_hz").max(get_f32("gyro_rpm_notch_min")),
            rpm_filter_fade_range_hz: get_f32("rpm_filter_fade_range_hz"),
            
            // Sample rates
            gyro_sample_rate: fd.sample_rate() as f32,
            pid_loop_rate: get_f32("pid_process_denom").max(1.0) * fd.sample_rate() as f32,
        }
    }
    
    /// Check if RPM filter is enabled
    pub fn rpm_filter_enabled(&self) -> bool {
        self.rpm_filter_harmonics > 0
    }
    
    /// Check if dynamic gyro LPF is enabled
    pub fn gyro_dyn_lpf_enabled(&self) -> bool {
        self.gyro_lpf1_dyn_min > 0.0 && self.gyro_lpf1_dyn_max > 0.0
    }
    
    /// Check if dynamic D-term LPF is enabled
    pub fn dterm_dyn_lpf_enabled(&self) -> bool {
        self.dterm_lpf1_dyn_min > 0.0 && self.dterm_lpf1_dyn_max > 0.0
    }
}

/// Noise analysis results
#[derive(Clone, Default)]
#[allow(dead_code)]
struct NoiseAnalysis {
    /// RMS noise levels for raw gyro (per axis)
    raw_gyro_rms: [f32; 3],
    /// RMS noise levels for filtered gyro (per axis)
    filtered_gyro_rms: [f32; 3],
    /// Noise reduction percentage (per axis)
    noise_reduction_pct: [f32; 3],
    /// Peak frequencies from FFT (top 5)
    peak_frequencies: Vec<f32>,
    /// D-term noise RMS (per axis)
    dterm_noise_rms: [f32; 3],
}

impl NoiseAnalysis {
    fn compute(fd: &FlightData) -> Self {
        let mut result = Self::default();
        
        // Calculate raw gyro noise
        if let Some(gyro_raw) = fd.gyro_unfiltered() {
            for (i, axis_data) in gyro_raw.iter().enumerate() {
                result.raw_gyro_rms[i] = Self::calculate_noise_rms(axis_data);
            }
        }
        
        // Calculate filtered gyro noise
        if let Some(gyro_filt) = fd.gyro_filtered() {
            for (i, axis_data) in gyro_filt.iter().enumerate() {
                result.filtered_gyro_rms[i] = Self::calculate_noise_rms(axis_data);
            }
        }
        
        // Calculate noise reduction percentage
        for i in 0..3 {
            if result.raw_gyro_rms[i] > 0.0 {
                result.noise_reduction_pct[i] = 
                    (1.0 - result.filtered_gyro_rms[i] / result.raw_gyro_rms[i]) * 100.0;
            }
        }
        
        // Calculate D-term noise
        let d_terms = fd.d();
        for (i, d_opt) in d_terms.iter().enumerate() {
            if let Some(d_data) = d_opt {
                result.dterm_noise_rms[i] = Self::calculate_noise_rms(d_data);
            }
        }
        
        result
    }
    
    /// Calculate noise RMS using high-pass filter approximation
    fn calculate_noise_rms(data: &[f32]) -> f32 {
        if data.len() < 10 {
            return 0.0;
        }
        
        // Simple high-pass: difference from moving average
        let window = 5;
        let mut noise_sum = 0.0;
        let mut count = 0;
        
        for i in window..data.len() - window {
            let avg: f32 = data[i - window..=i + window].iter().sum::<f32>() / (2 * window + 1) as f32;
            let noise = data[i] - avg;
            noise_sum += noise * noise;
            count += 1;
        }
        
        if count > 0 {
            (noise_sum / count as f32).sqrt()
        } else {
            0.0
        }
    }
}

/// Filter effectiveness scoring with letter grades
#[derive(Clone, Default)]
pub struct FilterEffectivenessScore {
    /// Overall score (0-100)
    pub overall_score: f32,
    /// Letter grade (A+, A, B, C, D, F)
    pub grade: String,
    /// Color for the grade
    pub grade_color: Color32,
    /// Individual component scores
    pub gyro_filtering_score: f32,
    pub dterm_noise_score: f32,
    pub config_quality_score: f32,
    /// Estimated filter latency in milliseconds
    pub estimated_latency_ms: f32,
    /// Latency assessment (Low/Moderate/High)
    pub latency_assessment: String,
    /// Latency color
    pub latency_color: Color32,
    /// Detailed breakdown
    pub summary: String,
}


impl FilterEffectivenessScore {
    /// Calculate comprehensive filter effectiveness score
    pub fn compute(noise: &NoiseAnalysis, settings: &FilterSettings) -> Self {
        let mut score = Self::default();
        
        // 1. Gyro filtering score (40% weight) - based on noise reduction
        let avg_reduction = (noise.noise_reduction_pct[0] + noise.noise_reduction_pct[1] + noise.noise_reduction_pct[2]) / 3.0;
        score.gyro_filtering_score = (avg_reduction).clamp(0.0, 100.0);
        
        // 2. D-term noise score (30% weight) - lower is better
        let avg_dterm_noise = (noise.dterm_noise_rms[0] + noise.dterm_noise_rms[1]) / 2.0;
        // Good D-term noise is < 10, bad is > 50
        score.dterm_noise_score = (100.0 - (avg_dterm_noise - 10.0).max(0.0) * 2.0).clamp(0.0, 100.0);
        
        // 3. Config quality score (30% weight) - based on settings
        let mut config_points = 0.0;
        let mut config_max = 0.0;
        
        // RPM filter enabled (+25 points)
        config_max += 25.0;
        if settings.rpm_filter_enabled() {
            config_points += 25.0;
        }
        
        // Dynamic gyro LPF (+15 points)
        config_max += 15.0;
        if settings.gyro_dyn_lpf_enabled() {
            config_points += 15.0;
        }
        
        // Dynamic D-term LPF (+15 points)
        config_max += 15.0;
        if settings.dterm_dyn_lpf_enabled() {
            config_points += 15.0;
        }
        
        // Gyro LPF2 for extra filtering (+10 points)
        config_max += 10.0;
        if settings.gyro_lpf2_hz > 0.0 {
            config_points += 10.0;
        }
        
        // D-term LPF2 (+10 points)
        config_max += 10.0;
        if settings.dterm_lpf2_hz > 0.0 {
            config_points += 10.0;
        }
        
        // Reasonable LPF1 frequency (not too aggressive, 150-400Hz is good) (+15 points)
        config_max += 15.0;
        if settings.gyro_lpf1_hz >= 100.0 && settings.gyro_lpf1_hz <= 500.0 {
            config_points += 15.0;
        } else if settings.gyro_lpf1_hz > 0.0 {
            config_points += 5.0; // Some filter is better than none
        }
        
        // Notch filters configured (+10 points)
        config_max += 10.0;
        if settings.gyro_notch1_hz > 0.0 || settings.gyro_notch2_hz > 0.0 {
            config_points += 10.0;
        }
        
        score.config_quality_score = if config_max > 0.0 {
            (config_points / config_max) * 100.0
        } else {
            50.0
        };
        
        // Calculate overall score (weighted average)
        score.overall_score = 
            score.gyro_filtering_score * 0.40 +
            score.dterm_noise_score * 0.30 +
            score.config_quality_score * 0.30;
        
        // Assign letter grade
        (score.grade, score.grade_color) = match score.overall_score as i32 {
            95..=100 => ("A+".to_string(), Color32::from_rgb(0, 255, 100)),
            85..=94 => ("A".to_string(), Color32::from_rgb(100, 255, 100)),
            75..=84 => ("B".to_string(), Color32::from_rgb(180, 255, 100)),
            65..=74 => ("C".to_string(), Color32::from_rgb(255, 255, 100)),
            50..=64 => ("D".to_string(), Color32::from_rgb(255, 180, 100)),
            _ => ("F".to_string(), Color32::from_rgb(255, 100, 100)),
        };
        
        // === LATENCY ESTIMATION ===
        // Estimate filter delay based on cutoff frequencies
        // For a 2nd order Butterworth LPF, group delay ≈ 0.32 / f_cutoff (at low frequencies)
        // We accumulate delay from multiple filter stages
        
        let mut total_delay_ms = 0.0f32;
        
        // Gyro LPF1 delay
        if settings.gyro_lpf1_hz > 0.0 {
            // PT1: delay ≈ 1 / (2 * pi * fc) * 1000ms
            // Simplified: delay_ms ≈ 159 / fc
            total_delay_ms += 159.0 / settings.gyro_lpf1_hz;
        } else if settings.gyro_dyn_lpf_enabled() {
            // Use average of dynamic range
            let avg_hz = (settings.gyro_lpf1_dyn_min + settings.gyro_lpf1_dyn_max) / 2.0;
            if avg_hz > 0.0 {
                total_delay_ms += 159.0 / avg_hz;
            }
        }
        
        // Gyro LPF2 delay
        if settings.gyro_lpf2_hz > 0.0 {
            total_delay_ms += 159.0 / settings.gyro_lpf2_hz;
        }
        
        // D-term LPF1 delay (only affects D-term, but still adds to overall feel)
        if settings.dterm_lpf1_hz > 0.0 {
            total_delay_ms += 159.0 / settings.dterm_lpf1_hz * 0.5; // Weight D-term less
        }
        
        // D-term LPF2 delay
        if settings.dterm_lpf2_hz > 0.0 {
            total_delay_ms += 159.0 / settings.dterm_lpf2_hz * 0.5;
        }
        
        // Notch filters add some delay (typically ~0.5-1ms each)
        if settings.gyro_notch1_hz > 0.0 {
            total_delay_ms += 0.5;
        }
        if settings.gyro_notch2_hz > 0.0 {
            total_delay_ms += 0.5;
        }
        
        // RPM filter adds minimal delay when properly configured
        if settings.rpm_filter_enabled() {
            total_delay_ms += 0.3 * settings.rpm_filter_harmonics as f32;
        }
        
        score.estimated_latency_ms = total_delay_ms;
        
        // Assess latency level
        (score.latency_assessment, score.latency_color) = if total_delay_ms < 3.0 {
            ("Very Low".to_string(), Color32::from_rgb(0, 255, 100))
        } else if total_delay_ms < 6.0 {
            ("Low".to_string(), Color32::from_rgb(100, 255, 100))
        } else if total_delay_ms < 10.0 {
            ("Moderate".to_string(), Color32::from_rgb(255, 255, 100))
        } else if total_delay_ms < 15.0 {
            ("High".to_string(), Color32::from_rgb(255, 180, 100))
        } else {
            ("Very High".to_string(), Color32::from_rgb(255, 100, 100))
        };
        
        // Generate summary
        score.summary = if score.overall_score >= 85.0 {
            "Excellent filter configuration. Noise is well controlled.".to_string()
        } else if score.overall_score >= 70.0 {
            "Good filtering. Minor optimizations possible.".to_string()
        } else if score.overall_score >= 55.0 {
            "Moderate filtering. Consider enabling RPM filter or dynamic LPF.".to_string()
        } else {
            "Poor filtering. Significant noise getting through. Review filter setup.".to_string()
        };
        
        score
    }
}


/// Motor RPM data for harmonic visualization
#[allow(dead_code)]
struct MotorRpmData {
    /// Average RPM per motor
    avg_rpm: Vec<f32>,
    /// Average RPM across all motors
    overall_avg_rpm: f32,
    /// eRPM to mechanical RPM divisor (motor poles / 2)
    erpm_divisor: f32,
}

impl MotorRpmData {
    fn compute(fd: &FlightData) -> Option<Self> {
        let erpm_data = fd.electrical_rpm()?;
        
        if erpm_data.is_empty() || erpm_data[0].is_empty() {
            return None;
        }
        
        // Assume 14-pole motor (7 pole pairs) - common for 5" quads
        let erpm_divisor = 7.0;
        
        let avg_rpm: Vec<f32> = erpm_data.iter()
            .map(|motor| {
                let sum: f32 = motor.iter().sum();
                (sum / motor.len() as f32) / erpm_divisor
            })
            .collect();
        
        let overall_avg_rpm = avg_rpm.iter().sum::<f32>() / avg_rpm.len() as f32;
        
        Some(Self {
            avg_rpm,
            overall_avg_rpm,
            erpm_divisor,
        })
    }
    
    /// Calculate fundamental motor frequency in Hz
    fn fundamental_hz(&self) -> f32 {
        self.overall_avg_rpm / 60.0
    }
    
    /// Get harmonic frequencies
    fn harmonics(&self, num_harmonics: u8) -> Vec<f32> {
        let fundamental = self.fundamental_hz();
        (1..=num_harmonics)
            .map(|h| fundamental * h as f32)
            .collect()
    }
}

pub struct FilterTab {
    fd: Arc<FlightData>,
    filter_settings: FilterSettings,
    noise_analysis: NoiseAnalysis,
    motor_rpm: Option<MotorRpmData>,
    show_raw_gyro: bool,
    show_filtered_gyro: bool,
    show_rpm_harmonics: bool,
    selected_axis: usize, // 0=Roll, 1=Pitch, 2=Yaw
}

impl FilterTab {
    pub fn new(fd: Arc<FlightData>) -> Self {
        let filter_settings = FilterSettings::parse_from_headers(&fd);
        let noise_analysis = NoiseAnalysis::compute(&fd);
        let motor_rpm = MotorRpmData::compute(&fd);
        
        Self {
            fd,
            filter_settings,
            noise_analysis,
            motor_rpm,
            show_raw_gyro: true,
            show_filtered_gyro: true,
            show_rpm_harmonics: true,
            selected_axis: 0,
        }
    }

    pub fn show(&mut self, ui: &mut egui::Ui) {
        let colors = Colors::get(ui);
        
        egui::ScrollArea::vertical().show(ui, |ui| {
            // Filter Settings Section
            ui.heading("Filter Configuration");
            ui.add_space(8.0);
            
            FlexColumns::new(MIN_WIDE_WIDTH)
                .column(|ui| self.show_gyro_filter_settings(ui, &colors))
                .column(|ui| self.show_dterm_filter_settings(ui, &colors))
                .column(|ui| self.show_rpm_filter_settings(ui, &colors))
                .show(ui);
            
            ui.add_space(16.0);
            ui.separator();
            ui.add_space(8.0);
            
            // Noise Analysis Section
            ui.heading("Noise Analysis");
            ui.add_space(8.0);
            
            FlexColumns::new(MIN_WIDE_WIDTH)
                .column(|ui| self.show_noise_metrics(ui, &colors))
                .column(|ui| self.show_filter_effectiveness(ui, &colors))
                .show(ui);
            
            ui.add_space(16.0);
            ui.separator();
            ui.add_space(8.0);
            
            // FFT Comparison Section
            ui.heading("Gyro FFT Comparison (Raw vs Filtered)");
            ui.add_space(8.0);
            
            // Controls
            ui.horizontal(|ui| {
                ui.label("Axis:");
                ui.selectable_value(&mut self.selected_axis, 0, "Roll");
                ui.selectable_value(&mut self.selected_axis, 1, "Pitch");
                ui.selectable_value(&mut self.selected_axis, 2, "Yaw");
                
                ui.separator();
                
                ui.checkbox(&mut self.show_raw_gyro, "Show Raw");
                ui.checkbox(&mut self.show_filtered_gyro, "Show Filtered");
                
                if self.motor_rpm.is_some() {
                    ui.checkbox(&mut self.show_rpm_harmonics, "Show RPM Harmonics");
                }
            });
            
            ui.add_space(8.0);
            self.show_fft_comparison(ui, &colors);
            
            ui.add_space(16.0);
            ui.separator();
            ui.add_space(8.0);
            
            // Recommendations Section
            ui.heading("Filter Recommendations");
            ui.add_space(8.0);
            self.show_recommendations(ui, &colors);
        });
    }
    
    fn show_gyro_filter_settings(&self, ui: &mut egui::Ui, _colors: &Colors) -> egui::Response {
        ui.vertical(|ui| {
            ui.label(RichText::new("Gyro Lowpass Filters").strong());
            ui.add_space(4.0);
            
            // LPF1
            ui.horizontal(|ui| {
                ui.label("LPF1:");
                if self.filter_settings.gyro_dyn_lpf_enabled() {
                    ui.label(format!(
                        "Dynamic {} ({:.0}-{:.0} Hz)",
                        self.filter_settings.gyro_lpf1_type,
                        self.filter_settings.gyro_lpf1_dyn_min,
                        self.filter_settings.gyro_lpf1_dyn_max
                    ));
                } else if self.filter_settings.gyro_lpf1_hz > 0.0 {
                    ui.label(format!(
                        "{} @ {:.0} Hz",
                        self.filter_settings.gyro_lpf1_type,
                        self.filter_settings.gyro_lpf1_hz
                    ));
                } else {
                    ui.label("OFF");
                }
            });
            
            // LPF2
            ui.horizontal(|ui| {
                ui.label("LPF2:");
                if self.filter_settings.gyro_lpf2_hz > 0.0 {
                    ui.label(format!(
                        "{} @ {:.0} Hz",
                        self.filter_settings.gyro_lpf2_type,
                        self.filter_settings.gyro_lpf2_hz
                    ));
                } else {
                    ui.label("OFF");
                }
            });
            
            ui.add_space(8.0);
            ui.label(RichText::new("Gyro Notch Filters").strong());
            ui.add_space(4.0);
            
            // Notch 1
            ui.horizontal(|ui| {
                ui.label("Notch 1:");
                if self.filter_settings.gyro_notch1_hz > 0.0 {
                    ui.label(format!(
                        "{:.0} Hz (cutoff {:.0})",
                        self.filter_settings.gyro_notch1_hz,
                        self.filter_settings.gyro_notch1_cutoff
                    ));
                } else {
                    ui.label("OFF");
                }
            });
            
            // Notch 2
            ui.horizontal(|ui| {
                ui.label("Notch 2:");
                if self.filter_settings.gyro_notch2_hz > 0.0 {
                    ui.label(format!(
                        "{:.0} Hz (cutoff {:.0})",
                        self.filter_settings.gyro_notch2_hz,
                        self.filter_settings.gyro_notch2_cutoff
                    ));
                } else {
                    ui.label("OFF");
                }
            });
        })
        .response
    }
    
    fn show_dterm_filter_settings(&self, ui: &mut egui::Ui, _colors: &Colors) -> egui::Response {
        ui.vertical(|ui| {
            ui.label(RichText::new("D-Term Lowpass Filters").strong());
            ui.add_space(4.0);
            
            // LPF1
            ui.horizontal(|ui| {
                ui.label("LPF1:");
                if self.filter_settings.dterm_dyn_lpf_enabled() {
                    ui.label(format!(
                        "Dynamic {} ({:.0}-{:.0} Hz)",
                        self.filter_settings.dterm_lpf1_type,
                        self.filter_settings.dterm_lpf1_dyn_min,
                        self.filter_settings.dterm_lpf1_dyn_max
                    ));
                } else if self.filter_settings.dterm_lpf1_hz > 0.0 {
                    ui.label(format!(
                        "{} @ {:.0} Hz",
                        self.filter_settings.dterm_lpf1_type,
                        self.filter_settings.dterm_lpf1_hz
                    ));
                } else {
                    ui.label("OFF");
                }
            });
            
            // LPF2
            ui.horizontal(|ui| {
                ui.label("LPF2:");
                if self.filter_settings.dterm_lpf2_hz > 0.0 {
                    ui.label(format!(
                        "{} @ {:.0} Hz",
                        self.filter_settings.dterm_lpf2_type,
                        self.filter_settings.dterm_lpf2_hz
                    ));
                } else {
                    ui.label("OFF");
                }
            });
            
            ui.add_space(8.0);
            ui.label(RichText::new("D-Term Notch Filter").strong());
            ui.add_space(4.0);
            
            ui.horizontal(|ui| {
                ui.label("Notch:");
                if self.filter_settings.dterm_notch_hz > 0.0 {
                    ui.label(format!(
                        "{:.0} Hz (cutoff {:.0})",
                        self.filter_settings.dterm_notch_hz,
                        self.filter_settings.dterm_notch_cutoff
                    ));
                } else {
                    ui.label("OFF");
                }
            });
        })
        .response
    }
    
    fn show_rpm_filter_settings(&self, ui: &mut egui::Ui, colors: &Colors) -> egui::Response {
        ui.vertical(|ui| {
            ui.label(RichText::new("RPM Filter").strong());
            ui.add_space(4.0);
            
            if self.filter_settings.rpm_filter_enabled() {
                ui.horizontal(|ui| {
                    ui.label("Status:");
                    ui.label(RichText::new("ENABLED").color(Color32::GREEN));
                });
                
                ui.horizontal(|ui| {
                    ui.label("Harmonics:");
                    ui.label(format!("{}", self.filter_settings.rpm_filter_harmonics));
                });
                
                ui.horizontal(|ui| {
                    ui.label("Q Factor:");
                    ui.label(format!("{:.1}", self.filter_settings.rpm_filter_q));
                });
                
                ui.horizontal(|ui| {
                    ui.label("Min Hz:");
                    ui.label(format!("{:.0}", self.filter_settings.rpm_filter_min_hz));
                });
                
                // Show motor RPM info if available
                if let Some(rpm) = &self.motor_rpm {
                    ui.add_space(8.0);
                    ui.label(RichText::new("Motor Analysis").strong());
                    
                    ui.horizontal(|ui| {
                        ui.label("Avg RPM:");
                        ui.label(format!("{:.0}", rpm.overall_avg_rpm));
                    });
                    
                    ui.horizontal(|ui| {
                        ui.label("Fundamental:");
                        ui.label(format!("{:.1} Hz", rpm.fundamental_hz()));
                    });
                    
                    // Show harmonics
                    let harmonics = rpm.harmonics(self.filter_settings.rpm_filter_harmonics);
                    for (i, hz) in harmonics.iter().enumerate() {
                        ui.horizontal(|ui| {
                            ui.label(format!("H{}:", i + 1));
                            ui.colored_label(colors.motors[i % 4], format!("{:.1} Hz", hz));
                        });
                    }
                }
            } else {
                ui.horizontal(|ui| {
                    ui.label("Status:");
                    ui.label(RichText::new("DISABLED").color(Color32::GRAY));
                });
                ui.label("Enable RPM filter for better noise rejection");
            }
        })
        .response
    }
    
    fn show_noise_metrics(&self, ui: &mut egui::Ui, colors: &Colors) -> egui::Response {
        ui.vertical(|ui| {
            ui.label(RichText::new("Noise RMS (deg/s)").strong());
            ui.add_space(4.0);
            
            let axis_names = ["Roll", "Pitch", "Yaw"];
            let axis_colors = colors.triple_primary;
            
            // Header
            ui.horizontal(|ui| {
                ui.label(RichText::new("Axis").strong());
                ui.add_space(20.0);
                ui.label(RichText::new("Raw").strong());
                ui.add_space(20.0);
                ui.label(RichText::new("Filtered").strong());
                ui.add_space(20.0);
                ui.label(RichText::new("Reduction").strong());
            });
            
            for i in 0..3 {
                ui.horizontal(|ui| {
                    ui.colored_label(axis_colors[i], format!("{:5}", axis_names[i]));
                    ui.add_space(10.0);
                    ui.label(format!("{:6.1}", self.noise_analysis.raw_gyro_rms[i]));
                    ui.add_space(20.0);
                    ui.label(format!("{:6.1}", self.noise_analysis.filtered_gyro_rms[i]));
                    ui.add_space(20.0);
                    
                    let reduction = self.noise_analysis.noise_reduction_pct[i];
                    let color = if reduction > 70.0 {
                        Color32::GREEN
                    } else if reduction > 50.0 {
                        Color32::YELLOW
                    } else {
                        Color32::RED
                    };
                    ui.colored_label(color, format!("{:.1}%", reduction));
                });
            }
            
            ui.add_space(8.0);
            ui.label(RichText::new("D-Term Noise RMS").strong());
            
            for i in 0..2 {
                ui.horizontal(|ui| {
                    ui.colored_label(axis_colors[i], format!("{:5}", axis_names[i]));
                    ui.add_space(10.0);
                    ui.label(format!("{:.1}", self.noise_analysis.dterm_noise_rms[i]));
                });
            }
        })
        .response
    }
    
    fn show_filter_effectiveness(&self, ui: &mut egui::Ui, colors: &Colors) -> egui::Response {
        // Compute the comprehensive effectiveness score
        let score = FilterEffectivenessScore::compute(&self.noise_analysis, &self.filter_settings);
        
        ui.vertical(|ui| {
            // Header with large letter grade
            ui.horizontal(|ui| {
                ui.label(RichText::new("Filter Effectiveness").strong().size(16.0));
                ui.add_space(16.0);
                ui.label(
                    RichText::new(&score.grade)
                        .color(score.grade_color)
                        .strong()
                        .size(28.0)
                );
                ui.label(
                    RichText::new(format!(" ({:.0}/100)", score.overall_score))
                        .weak()
                        .size(14.0)
                );
            });
            
            ui.add_space(4.0);
            ui.label(RichText::new(&score.summary).weak().italics());
            ui.add_space(8.0);
            
            // Score breakdown
            ui.label(RichText::new("Score Breakdown").strong());
            ui.add_space(4.0);
            
            // Helper to draw a mini progress bar
            let draw_score_bar = |ui: &mut egui::Ui, label: &str, value: f32, tooltip: &str| {
                ui.horizontal(|ui| {
                    ui.label(format!("{:18}", label));
                    
                    let (rect, response) = ui.allocate_exact_size(
                        egui::Vec2::new(100.0, 12.0),
                        egui::Sense::hover(),
                    );
                    
                    // Background
                    ui.painter().rect_filled(rect, 2.0, Color32::from_gray(40));
                    
                    // Bar
                    let bar_width = (value / 100.0).clamp(0.0, 1.0) * 100.0;
                    let bar_rect = egui::Rect::from_min_size(rect.min, egui::Vec2::new(bar_width, 12.0));
                    
                    let bar_color = if value >= 75.0 {
                        Color32::from_rgb(100, 255, 100)
                    } else if value >= 50.0 {
                        Color32::from_rgb(255, 255, 100)
                    } else {
                        Color32::from_rgb(255, 100, 100)
                    };
                    
                    ui.painter().rect_filled(bar_rect, 2.0, bar_color);
                    
                    ui.add_space(8.0);
                    ui.label(format!("{:.0}%", value));
                    
                    response.on_hover_text(tooltip);
                });
            };
            
            draw_score_bar(ui, "Gyro Filtering:", score.gyro_filtering_score, "Noise reduction from raw to filtered gyro");
            draw_score_bar(ui, "D-Term Noise:", score.dterm_noise_score, "D-term noise level (lower noise = higher score)");
            draw_score_bar(ui, "Config Quality:", score.config_quality_score, "Filter configuration (RPM filter, dynamic LPF, etc.)");
            
            ui.add_space(8.0);
            
            // === LATENCY DISPLAY ===
            ui.horizontal(|ui| {
                ui.label(RichText::new("Estimated Latency:").strong());
                ui.label(
                    RichText::new(format!("{:.1}ms", score.estimated_latency_ms))
                        .color(score.latency_color)
                        .strong()
                );
                ui.label(
                    RichText::new(format!("({})", score.latency_assessment))
                        .color(score.latency_color)
                );
            });
            ui.label(
                RichText::new("Lower latency = snappier response. Higher latency = more noise reduction.")
                    .weak()
                    .small()
            );
            
            ui.add_space(8.0);
            ui.separator();
            ui.add_space(4.0);

            
            // Noise reduction per axis
            ui.label(RichText::new("Per-Axis Noise Reduction").strong());
            let axis_names = ["Roll", "Pitch", "Yaw"];
            
            for (i, name) in axis_names.iter().enumerate() {
                let reduction = self.noise_analysis.noise_reduction_pct[i].clamp(0.0, 100.0);
                let bar_width = (reduction / 100.0) * 120.0;
                
                ui.horizontal(|ui| {
                    ui.label(format!("{:5}", name));
                    
                    let (rect, _response) = ui.allocate_exact_size(
                        egui::Vec2::new(130.0, 14.0),
                        egui::Sense::hover(),
                    );
                    
                    ui.painter().rect_filled(rect, 2.0, Color32::from_gray(40));
                    
                    let bar_rect = egui::Rect::from_min_size(rect.min, egui::Vec2::new(bar_width, 14.0));
                    
                    let color = if reduction > 70.0 {
                        colors.triple_primary[i]
                    } else if reduction > 50.0 {
                        Color32::YELLOW
                    } else {
                        Color32::RED
                    };
                    
                    ui.painter().rect_filled(bar_rect, 2.0, color);
                    ui.painter().text(
                        rect.center(),
                        egui::Align2::CENTER_CENTER,
                        format!("{:.0}%", reduction),
                        egui::FontId::default(),
                        Color32::WHITE,
                    );
                });
            }
        })
        .response
    }
    
    fn show_fft_comparison(&mut self, ui: &mut egui::Ui, colors: &Colors) {
        let axis_colors = colors.triple_primary;
        let selected_color = axis_colors[self.selected_axis];
        
        // Calculate simple FFT for visualization
        let raw_fft = self.calculate_simple_fft(true, self.selected_axis);
        let filtered_fft = self.calculate_simple_fft(false, self.selected_axis);
        
        Plot::new("filter_fft_comparison")
            .height(PLOT_HEIGHT * 1.5)
            .x_axis_label("Frequency (Hz)")
            .y_axis_label("Amplitude")
            .include_x(0.0)
            .include_x(500.0)
            .include_y(0.0)
            .legend(Legend::default().position(Corner::RightTop))
            .show(ui, |plot_ui| {
                // Draw raw gyro FFT
                if self.show_raw_gyro && !raw_fft.is_empty() {
                    let raw_line = Line::new(PlotPoints::new(raw_fft))
                        .stroke(Stroke::new(1.5, selected_color.gamma_multiply(0.5)))
                        .name("Raw Gyro");
                    plot_ui.line(raw_line);
                }
                
                // Draw filtered gyro FFT
                if self.show_filtered_gyro && !filtered_fft.is_empty() {
                    let filtered_line = Line::new(PlotPoints::new(filtered_fft))
                        .stroke(Stroke::new(2.0, selected_color))
                        .name("Filtered Gyro");
                    plot_ui.line(filtered_line);
                }
                
                // Draw filter cutoff frequencies
                if self.filter_settings.gyro_lpf1_hz > 0.0 {
                    plot_ui.vline(
                        VLine::new(self.filter_settings.gyro_lpf1_hz as f64)
                            .color(Color32::YELLOW)
                            .style(egui_plot::LineStyle::Dashed { length: 5.0 })
                            .name(format!("LPF1 @ {:.0}Hz", self.filter_settings.gyro_lpf1_hz))
                    );
                }
                
                if self.filter_settings.gyro_lpf2_hz > 0.0 {
                    plot_ui.vline(
                        VLine::new(self.filter_settings.gyro_lpf2_hz as f64)
                            .color(Color32::LIGHT_YELLOW)
                            .style(egui_plot::LineStyle::Dashed { length: 5.0 })
                            .name(format!("LPF2 @ {:.0}Hz", self.filter_settings.gyro_lpf2_hz))
                    );
                }
                
                // Draw RPM harmonics
                if self.show_rpm_harmonics {
                    if let Some(rpm) = &self.motor_rpm {
                        let harmonics = rpm.harmonics(self.filter_settings.rpm_filter_harmonics.max(3));
                        for (i, hz) in harmonics.iter().enumerate() {
                            if *hz > 0.0 && *hz < 500.0 {
                                plot_ui.vline(
                                    VLine::new(*hz as f64)
                                        .color(colors.motors[i % 4])
                                        .style(egui_plot::LineStyle::Dotted { spacing: 3.0 })
                                        .name(format!("H{} @ {:.0}Hz", i + 1, hz))
                                );
                            }
                        }
                    }
                }
                
                // Draw notch filter positions
                if self.filter_settings.gyro_notch1_hz > 0.0 {
                    plot_ui.vline(
                        VLine::new(self.filter_settings.gyro_notch1_hz as f64)
                            .color(Color32::RED)
                            .style(egui_plot::LineStyle::Dashed { length: 3.0 })
                            .name(format!("Notch1 @ {:.0}Hz", self.filter_settings.gyro_notch1_hz))
                    );
                }
                
                if self.filter_settings.gyro_notch2_hz > 0.0 {
                    plot_ui.vline(
                        VLine::new(self.filter_settings.gyro_notch2_hz as f64)
                            .color(Color32::from_rgb(255, 100, 100))
                            .style(egui_plot::LineStyle::Dashed { length: 3.0 })
                            .name(format!("Notch2 @ {:.0}Hz", self.filter_settings.gyro_notch2_hz))
                    );
                }
            });
    }
    
    /// Calculate a simplified FFT for visualization
    fn calculate_simple_fft(&self, raw: bool, axis: usize) -> Vec<[f64; 2]> {
        let data = if raw {
            self.fd.gyro_unfiltered().map(|g| g[axis])
        } else {
            self.fd.gyro_filtered().map(|g| g[axis])
        };
        
        let Some(data) = data else {
            return vec![];
        };
        
        if data.len() < 512 {
            return vec![];
        }
        
        // Use a simple periodogram approach
        let fft_size = 512;
        let sample_rate = self.fd.sample_rate();
        let freq_resolution = sample_rate / fft_size as f64;
        
        // Take middle portion of data
        let start = (data.len() - fft_size) / 2;
        let window: Vec<f32> = data[start..start + fft_size].to_vec();
        
        // Apply Hanning window
        let windowed: Vec<f32> = window.iter().enumerate()
            .map(|(i, &v)| {
                let w = 0.5 * (1.0 - (2.0 * std::f32::consts::PI * i as f32 / fft_size as f32).cos());
                v * w
            })
            .collect();
        
        // Simple DFT for visualization (not optimized, but works for demo)
        let mut spectrum = Vec::with_capacity(fft_size / 2);
        for k in 0..fft_size / 2 {
            let freq = k as f64 * freq_resolution;
            if freq > 500.0 {
                break;
            }
            
            let mut real = 0.0f32;
            let mut imag = 0.0f32;
            
            for (n, &sample) in windowed.iter().enumerate().take(fft_size) {
                let angle = 2.0 * std::f32::consts::PI * k as f32 * n as f32 / fft_size as f32;
                real += sample * angle.cos();
                imag -= sample * angle.sin();
            }
            
            let magnitude = (real * real + imag * imag).sqrt() / fft_size as f32;
            spectrum.push([freq, magnitude as f64]);
        }
        
        spectrum
    }
    
    fn show_recommendations(&self, ui: &mut egui::Ui, _colors: &Colors) {
        ui.vertical(|ui| {
            let mut recommendations = Vec::new();
            
            // Check noise reduction
            let avg_reduction: f32 = self.noise_analysis.noise_reduction_pct.iter().sum::<f32>() / 3.0;
            
            if avg_reduction < 50.0 {
                recommendations.push((
                    "⚠",
                    "Low noise reduction detected",
                    "Consider lowering gyro LPF cutoff frequencies or enabling dynamic filtering",
                ));
            }
            
            // Check D-term noise
            let avg_dterm_noise: f32 = self.noise_analysis.dterm_noise_rms.iter().take(2).sum::<f32>() / 2.0;
            if avg_dterm_noise > 50.0 {
                recommendations.push((
                    "⚠", 
                    "High D-term noise",
                    "Consider lowering D-term LPF cutoff or reducing D gains",
                ));
            }
            
            // Check RPM filter
            if !self.filter_settings.rpm_filter_enabled() && self.motor_rpm.is_some() {
                recommendations.push((
                    "★",
                    "RPM filter not enabled",
                    "Enable RPM filter for better motor noise rejection (requires bidirectional DShot)",
                ));
            }
            
            // Check dynamic filtering
            if !self.filter_settings.gyro_dyn_lpf_enabled() {
                recommendations.push((
                    "★",
                    "Static gyro LPF in use",
                    "Consider enabling dynamic gyro LPF for better performance across throttle range",
                ));
            }
            
            // Good configuration check
            if recommendations.is_empty() {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("✓").size(16.0));
                    ui.label("Filter configuration looks good! No major issues detected.");
                });
            } else {
                for (icon, title, description) in recommendations {
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(icon).size(16.0));
                        ui.vertical(|ui| {
                            ui.label(RichText::new(title).strong());
                            ui.label(description);
                        });
                    });
                    ui.add_space(4.0);
                }
            }
            
            ui.add_space(8.0);
            
            // Show motor harmonic suggestions if RPM data available
            if let Some(rpm) = &self.motor_rpm {
                ui.label(RichText::new("Motor Harmonic Analysis").strong());
                
                let fundamental = rpm.fundamental_hz();
                if fundamental > 50.0 {
                    ui.horizontal(|ui| {
                        ui.label("Suggested notch frequencies:");
                    });
                    
                    for h in 1..=3 {
                        let freq = fundamental * h as f32;
                        ui.horizontal(|ui| {
                            ui.label(format!("  H{}: {:.0} Hz", h, freq));
                        });
                    }
                }
            }
        });
    }
}

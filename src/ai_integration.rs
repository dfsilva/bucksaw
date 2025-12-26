use serde::{Deserialize, Serialize};
use std::sync::mpsc;
#[cfg(not(target_arch = "wasm32"))]
use std::thread;

#[cfg(target_arch = "wasm32")]
use wasm_bindgen_futures::spawn_local;

/// AI model from OpenRouter
#[derive(Clone, Debug)]
pub struct AIModel {
    pub id: String,
    pub name: String,
}

/// Default/fallback models if API fetch fails
pub const DEFAULT_MODELS: &[(&str, &str)] = &[
    ("anthropic/claude-3.5-sonnet", "Claude 3.5 Sonnet"),
    ("anthropic/claude-3-haiku", "Claude 3 Haiku (Fast)"),
    ("openai/gpt-4o", "GPT-4o"),
    ("openai/gpt-4o-mini", "GPT-4o Mini (Fast)"),
    ("google/gemini-pro-1.5", "Gemini Pro 1.5"),
    ("meta-llama/llama-3.1-70b-instruct", "Llama 3.1 70B"),
];

/// Response from OpenRouter /models endpoint
#[derive(Deserialize)]
struct ModelsResponse {
    data: Vec<ModelInfo>,
}

#[derive(Deserialize)]
struct ModelInfo {
    id: String,
    name: Option<String>,
}

/// Result of fetching models
#[allow(dead_code)]
pub enum ModelFetchResult {
    Success(Vec<AIModel>),
    Error(String),
}

impl OpenRouterClient {
    /// Fetch available models from OpenRouter API (blocking, native only)
    #[cfg(not(target_arch = "wasm32"))]
    pub fn fetch_models_blocking() -> ModelFetchResult {
        let client = match reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_secs(15))
            .build()
        {
            Ok(c) => c,
            Err(e) => {
                return ModelFetchResult::Error(format!("Failed to create HTTP client: {}", e))
            }
        };

        let response = client
            .get("https://openrouter.ai/api/v1/models")
            .header("Content-Type", "application/json")
            .send();

        Self::handle_models_response(response)
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn handle_models_response(
        response: Result<reqwest::blocking::Response, reqwest::Error>,
    ) -> ModelFetchResult {
        match response {
            Ok(resp) => {
                if resp.status().is_success() {
                    match resp.json::<ModelsResponse>() {
                        Ok(models_resp) => Self::process_models(models_resp),
                        Err(e) => ModelFetchResult::Error(format!("Failed to parse models: {}", e)),
                    }
                } else {
                    ModelFetchResult::Error(format!("API returned status: {}", resp.status()))
                }
            }
            Err(e) => ModelFetchResult::Error(format!("Request failed: {}", e)),
        }
    }

    #[cfg(target_arch = "wasm32")]
    async fn fetch_models_async_internal() -> ModelFetchResult {
        let client = reqwest::Client::new();

        let response = client
            .get("https://openrouter.ai/api/v1/models")
            .header("Content-Type", "application/json")
            .send()
            .await;

        match response {
            Ok(resp) => {
                if resp.status().is_success() {
                    match resp.json::<ModelsResponse>().await {
                        Ok(models_resp) => Self::process_models(models_resp),
                        Err(e) => ModelFetchResult::Error(format!("Failed to parse models: {}", e)),
                    }
                } else {
                    ModelFetchResult::Error(format!("API returned status: {}", resp.status()))
                }
            }
            Err(e) => ModelFetchResult::Error(format!("Request failed: {}", e)),
        }
    }

    fn process_models(models_resp: ModelsResponse) -> ModelFetchResult {
        let models: Vec<AIModel> = models_resp
            .data
            .into_iter()
            .map(|m| AIModel {
                name: m.name.unwrap_or_else(|| m.id.clone()),
                id: m.id,
            })
            .collect();
        ModelFetchResult::Success(models)
    }

    /// Fetch models in background (thread on native, spawn_local on Wasm)
    pub fn fetch_models_async() -> mpsc::Receiver<ModelFetchResult> {
        let (tx, rx) = mpsc::channel();

        #[cfg(not(target_arch = "wasm32"))]
        thread::spawn(move || {
            let result = Self::fetch_models_blocking();
            let _ = tx.send(result);
        });

        #[cfg(target_arch = "wasm32")]
        spawn_local(async move {
            let result = Self::fetch_models_async_internal().await;
            let _ = tx.send(result);
        });

        rx
    }
}

/// Request body for OpenRouter API
#[derive(Serialize)]
struct ChatRequest {
    model: String,
    messages: Vec<ChatMessage>,
    max_tokens: u32,
}

#[derive(Serialize)]
struct ChatMessage {
    role: String,
    content: String,
}

/// Response from OpenRouter API
#[derive(Deserialize)]
struct ChatResponse {
    choices: Vec<Choice>,
}

#[derive(Deserialize)]
struct Choice {
    message: MessageContent,
}

#[derive(Deserialize)]
struct MessageContent {
    content: String,
}

/// Error response from OpenRouter
#[derive(Deserialize)]
struct ErrorResponse {
    error: Option<ErrorDetail>,
}

#[derive(Deserialize)]
struct ErrorDetail {
    message: String,
}

/// Analysis focus option for AI tuning recommendations
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AnalysisFocus {
    #[default]
    /// General tuning - balanced approach
    General,
    /// Focus on reducing latency for snappier response
    ReduceLatency,
    /// Focus on reducing noise and motor heat
    ReduceNoise,
    /// Focus on maximum performance for racing
    MaxPerformance,
}

impl AnalysisFocus {
    pub const ALL: [AnalysisFocus; 4] = [
        AnalysisFocus::General,
        AnalysisFocus::ReduceLatency,
        AnalysisFocus::ReduceNoise,
        AnalysisFocus::MaxPerformance,
    ];
}

impl std::fmt::Display for AnalysisFocus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AnalysisFocus::General => write!(f, "🎯 General"),
            AnalysisFocus::ReduceLatency => write!(f, "⚡ Reduce Latency"),
            AnalysisFocus::ReduceNoise => write!(f, "🔇 Reduce Noise"),
            AnalysisFocus::MaxPerformance => write!(f, "🏁 Max Performance"),
        }
    }
}

/// Flight metrics to send to AI
#[derive(Clone)]
pub struct FlightMetrics {
    pub firmware: String,
    pub craft_name: String,
    pub duration_sec: f64,

    // PID values (P, I, D-Max, Feedforward)
    pub roll_pid: [f32; 4],
    pub pitch_pid: [f32; 4],
    pub yaw_pid: [f32; 4],

    // D-min values (actual D at low stick input)
    pub d_min: [f32; 3],

    // Analysis results
    pub step_overshoot: [f32; 3],
    pub step_undershoot: [f32; 3],
    pub gyro_noise_rms: [f32; 3],
    pub dterm_noise_rms: [f32; 3],
    pub tracking_error_rms: [f32; 3],
    pub motor_saturation_pct: f32,

    // Filter settings (comprehensive)
    pub gyro_lpf_hz: Option<f32>,
    pub gyro_lpf2_hz: Option<f32>,
    pub dterm_lpf_hz: Option<f32>,
    pub dterm_lpf2_hz: Option<f32>,
    pub yaw_lpf_hz: Option<f32>,
    pub dyn_notch_count: Option<f32>,
    pub dyn_notch_min: Option<f32>,
    pub dyn_notch_max: Option<f32>,
    pub rpm_filter_harmonics: Option<f32>,

    // Rate settings (roll, pitch, yaw)
    pub rc_rate: Option<[f32; 3]>,
    pub rc_expo: Option<[f32; 3]>,
    pub super_rate: Option<[f32; 3]>,

    // Motor statistics
    pub motor_min_pct: f32,
    pub motor_max_pct: f32,
    pub motor_avg_pct: f32,
    pub motor_differential: f32,

    // Anomaly summary
    pub anomaly_count: usize,
    pub motor_saturation_events: usize,
    pub high_vibration_events: usize,

    // Step response timing
    pub step_rise_time_ms: Option<[f32; 3]>,
    pub step_settling_time_ms: Option<[f32; 3]>,

    // ===== PID Controller Settings =====

    // Feedforward settings
    pub feedforward_jitter_reduction: Option<u32>,
    pub feedforward_smoothness: Option<u32>,
    pub feedforward_averaging: Option<String>, // "OFF", "2 Point", etc.
    pub feedforward_boost: Option<u32>,
    pub feedforward_max_rate_limit: Option<u32>,
    pub feedforward_transition: Option<u32>,

    // I-Term settings
    pub iterm_relax_enabled: bool,
    pub iterm_relax_type: Option<String>, // "Setpoint", "Gyro"
    pub iterm_relax_cutoff: Option<u32>,
    pub iterm_rotation: bool,

    // Anti-Gravity
    pub anti_gravity_gain: Option<u32>,

    // Dynamic Damping (D-Max)
    pub d_max_gain: Option<u32>,
    pub d_max_advance: Option<u32>,

    // Throttle and Motor Settings
    pub throttle_boost: Option<u32>,
    pub motor_output_limit: Option<u32>,
    pub dyn_idle_min_rpm: Option<u32>,
    pub vbat_sag_compensation: Option<u32>,
    pub thrust_linear: Option<u32>,

    // TPA
    pub tpa_mode: Option<String>, // "OFF", "D", "PD"
    pub tpa_rate: Option<u32>,
    pub tpa_breakpoint: Option<u32>,

    // Misc
    pub integrated_yaw: bool,
    pub abs_control_gain: Option<u32>,

    /// Analysis focus for AI recommendations
    pub analysis_focus: AnalysisFocus,

    // === NEW ANALYSIS METRICS ===

    // Propwash analysis
    pub propwash_severity: f32,      // 0-100 score
    pub propwash_events: usize,      // Number of detected events
    pub propwash_worst_axis: String, // "Roll", "Pitch", or "Yaw"

    // System latency
    pub system_latency_ms: [f32; 3], // Per-axis latency
    pub avg_latency_ms: f32,         // Average latency

    // I-term analysis
    pub iterm_windup_events: usize, // Count of saturation events
    pub iterm_drift: [f32; 3],      // Drift per axis
}

impl FlightMetrics {
    pub fn to_prompt(&self) -> String {
        // Build filter settings string
        let mut filter_info = String::new();
        if let Some(hz) = self.gyro_lpf_hz {
            filter_info.push_str(&format!("- Gyro LPF1: {}Hz\n", hz as i32));
        }
        if let Some(hz) = self.gyro_lpf2_hz {
            filter_info.push_str(&format!("- Gyro LPF2: {}Hz\n", hz as i32));
        }
        if let Some(hz) = self.dterm_lpf_hz {
            filter_info.push_str(&format!("- D-Term LPF1: {}Hz\n", hz as i32));
        }
        if let Some(hz) = self.dterm_lpf2_hz {
            filter_info.push_str(&format!("- D-Term LPF2: {}Hz\n", hz as i32));
        }
        if let Some(hz) = self.yaw_lpf_hz {
            filter_info.push_str(&format!("- Yaw LPF: {}Hz\n", hz as i32));
        }
        if let Some(count) = self.dyn_notch_count {
            filter_info.push_str(&format!("- Dynamic Notch: {} notches", count as i32));
            if let (Some(min), Some(max)) = (self.dyn_notch_min, self.dyn_notch_max) {
                filter_info.push_str(&format!(" ({}Hz-{}Hz)\n", min as i32, max as i32));
            } else {
                filter_info.push('\n');
            }
        }
        if let Some(harmonics) = self.rpm_filter_harmonics {
            filter_info.push_str(&format!(
                "- RPM Filter: {} harmonics (enabled)\n",
                harmonics as i32
            ));
        }
        let filter_section = if filter_info.is_empty() {
            String::new()
        } else {
            format!("## Filter Settings\n{}", filter_info)
        };

        // Build rate settings section
        let rate_section = {
            let mut rate_info = String::new();
            if let Some(rates) = &self.rc_rate {
                rate_info.push_str(&format!(
                    "- RC Rate: Roll={:.0}, Pitch={:.0}, Yaw={:.0}\n",
                    rates[0], rates[1], rates[2]
                ));
            }
            if let Some(expo) = &self.rc_expo {
                rate_info.push_str(&format!(
                    "- Expo: Roll={:.0}, Pitch={:.0}, Yaw={:.0}\n",
                    expo[0], expo[1], expo[2]
                ));
            }
            if let Some(srate) = &self.super_rate {
                rate_info.push_str(&format!(
                    "- Super Rate: Roll={:.0}, Pitch={:.0}, Yaw={:.0}\n",
                    srate[0], srate[1], srate[2]
                ));
            }
            if rate_info.is_empty() {
                String::new()
            } else {
                format!("## Rate Settings\n{}", rate_info)
            }
        };

        // Build PID Controller Settings section
        let pid_controller_section = {
            let mut info = String::new();

            // Feedforward
            info.push_str("### Feedforward\n");
            if let Some(v) = self.feedforward_jitter_reduction {
                info.push_str(&format!("- Jitter Reduction: {}\n", v));
            }
            if let Some(v) = self.feedforward_smoothness {
                info.push_str(&format!("- Smoothness: {}\n", v));
            }
            if let Some(ref v) = self.feedforward_averaging {
                info.push_str(&format!("- Averaging: {}\n", v));
            }
            if let Some(v) = self.feedforward_boost {
                info.push_str(&format!("- Boost: {}\n", v));
            }
            if let Some(v) = self.feedforward_max_rate_limit {
                info.push_str(&format!("- Max Rate Limit: {}\n", v));
            }
            if let Some(v) = self.feedforward_transition {
                info.push_str(&format!("- Transition: {}\n", v));
            }

            // I-Term settings
            info.push_str("\n### I-Term Settings\n");
            info.push_str(&format!(
                "- I-Term Relax: {}\n",
                if self.iterm_relax_enabled {
                    "Enabled"
                } else {
                    "Disabled"
                }
            ));
            if let Some(ref v) = self.iterm_relax_type {
                info.push_str(&format!("- Relax Type: {}\n", v));
            }
            if let Some(v) = self.iterm_relax_cutoff {
                info.push_str(&format!("- Relax Cutoff: {}\n", v));
            }
            info.push_str(&format!(
                "- I-Term Rotation: {}\n",
                if self.iterm_rotation {
                    "Enabled"
                } else {
                    "Disabled"
                }
            ));

            // Anti-Gravity
            info.push_str("\n### Anti-Gravity\n");
            if let Some(v) = self.anti_gravity_gain {
                info.push_str(&format!(
                    "- Gain: {} ({})\n",
                    v,
                    if v > 0 { "Enabled" } else { "Disabled" }
                ));
            }

            // Dynamic Damping (D-Max)
            info.push_str("\n### Dynamic Damping (D-Max)\n");
            if let Some(v) = self.d_max_gain {
                info.push_str(&format!("- Gain: {}\n", v));
            }
            if let Some(v) = self.d_max_advance {
                info.push_str(&format!("- Advance: {}\n", v));
            }

            // Throttle & Motor
            info.push_str("\n### Throttle & Motor Settings\n");
            if let Some(v) = self.throttle_boost {
                info.push_str(&format!("- Throttle Boost: {}\n", v));
            }
            if let Some(v) = self.motor_output_limit {
                info.push_str(&format!("- Motor Output Limit: {}%\n", v));
            }
            if let Some(v) = self.dyn_idle_min_rpm {
                info.push_str(&format!("- Dynamic Idle: {} (x100 RPM)\n", v));
            }
            if let Some(v) = self.vbat_sag_compensation {
                info.push_str(&format!("- Vbat Sag Compensation: {}%\n", v));
            }
            if let Some(v) = self.thrust_linear {
                info.push_str(&format!("- Thrust Linearization: {}%\n", v));
            }

            // TPA
            info.push_str("\n### TPA (Throttle PID Attenuation)\n");
            if let Some(ref v) = self.tpa_mode {
                info.push_str(&format!("- Mode: {}\n", v));
            }
            if let Some(v) = self.tpa_rate {
                info.push_str(&format!("- Rate: {}%\n", v));
            }
            if let Some(v) = self.tpa_breakpoint {
                info.push_str(&format!("- Breakpoint: {}µs\n", v));
            }

            // Misc
            info.push_str("\n### Miscellaneous\n");
            info.push_str(&format!(
                "- Integrated Yaw: {}\n",
                if self.integrated_yaw {
                    "Enabled"
                } else {
                    "Disabled"
                }
            ));
            if let Some(v) = self.abs_control_gain {
                info.push_str(&format!(
                    "- Absolute Control: {} ({})\n",
                    v,
                    if v > 0 { "Enabled" } else { "Disabled" }
                ));
            }

            format!("## PID Controller Settings\n{}", info)
        };

        // Build focus-specific instructions
        let focus_instruction = match self.analysis_focus {
            AnalysisFocus::General => "Provide balanced tuning recommendations considering both performance and noise control.".to_string(),
            AnalysisFocus::ReduceLatency => r#"**USER PRIORITY: REDUCE LATENCY**
Focus primarily on reducing input-to-response delay. Recommend:
- Raising filter cutoff frequencies where noise permits
- Reducing filter delay by using higher cutoffs for gyro_lpf, dterm_lpf
- Increasing Feedforward for faster stick response
- Optimizing D-term filtering for lower delay
- Reducing smoothing factors
- Consider suggests for iterm_relax and feedforward_transition
WARNING: Be mindful that reducing latency too aggressively can increase noise and motor heat."#.to_string(),
            AnalysisFocus::ReduceNoise => r#"**USER PRIORITY: REDUCE NOISE**
Focus primarily on reducing vibrations and motor heat. Recommend:
- Lower filter cutoff frequencies
- Increase dynamic notch count if needed
- Lower D-gain if D-term noise is high
- Enable or optimize RPM filtering if available
- Suggest safer, more conservative filter settings
- Prioritize motor longevity over responsiveness"#.to_string(),
            AnalysisFocus::MaxPerformance => r#"**USER PRIORITY: MAXIMUM PERFORMANCE**
Focus on achieving the fastest, most responsive tune for racing. Recommend:
- Aggressive filter settings (high cutoffs) where noise permits
- Higher P and D gains for sharper response
- Higher Feedforward for instant reaction
- Lower filtering delay at the expense of some noise tolerance
- Optimize for competition use, accepting trade-offs in motor heat
WARNING: These recommendations may increase motor wear and temperature."#.to_string(),
        };

        format!(
            r#"Analyze this FPV drone flight log and provide specific tuning recommendations.

{focus}

IMPORTANT: The user is running firmware version "{firmware}". 
All CLI commands you provide MUST be compatible with this exact firmware version.
Do NOT suggest commands or parameters that don't exist in this version.

## Craft Info
- Firmware Version: {firmware}
- Craft Name: {craft}
- Flight Duration: {duration:.1}s"#,
            focus = focus_instruction,
            firmware = self.firmware,
            craft = self.craft_name,
            duration = self.duration_sec,
        ) + &format!(
            r#"

## Current PID Settings
| Axis | P | I | D-Max | D-Min | FF |
|------|---|---|-------|-------|-----|
| Roll | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} |
| Pitch | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} |
| Yaw | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} |

Note: D-Max is the max D gain at high stick movement. D-Min is D at low stick input.
Feedforward (FF) provides instantaneous stick response.

{}

## Step Response Analysis (from gyro data)
- Roll Overshoot: {:.1}% | Undershoot: {:.1}%
- Pitch Overshoot: {:.1}% | Undershoot: {:.1}%
- Yaw Overshoot: {:.1}% | Undershoot: {:.1}%

## Noise Levels
- Gyro Noise RMS: Roll={:.1}, Pitch={:.1}, Yaw={:.1} deg/s
- D-Term Noise RMS: Roll={:.1}, Pitch={:.1} (high values cause hot motors)

## Tracking Error
- RMS Error: Roll={:.1}, Pitch={:.1}, Yaw={:.1} deg/s

## Motor Performance
- Saturation: {:.1}% of flight time (>10% indicates need for lower authority)

## Propwash Analysis
- Severity Score: {:.0}/100
- Detected Events: {}
- Worst Affected Axis: {}

## System Latency
- Average Latency: {:.1}ms
- Per-Axis: Roll={:.1}ms, Pitch={:.1}ms, Yaw={:.1}ms

## I-Term Health
- Windup Events: {}
- Drift: Roll={:.1}, Pitch={:.1}, Yaw={:.1}

{}
{}

Please provide:
1. Overall assessment (is the tune good, needs work, or problematic?)
2. Specific PID adjustments with CLI commands compatible with {}
3. D-gain, D-max, and Feedforward recommendations based on the controller settings
4. Filter recommendations if noise is high
5. TPA and Anti-Gravity recommendations based on current settings
6. Propwash-specific recommendations if severity is high
7. Latency optimization suggestions if system latency is high
8. Any other observations or warnings about the current tune

REMINDER: Only use CLI commands that exist in {}."#,
            // Roll
            self.roll_pid[0],
            self.roll_pid[1],
            self.roll_pid[2],
            self.d_min[0],
            self.roll_pid[3],
            // Pitch
            self.pitch_pid[0],
            self.pitch_pid[1],
            self.pitch_pid[2],
            self.d_min[1],
            self.pitch_pid[3],
            // Yaw
            self.yaw_pid[0],
            self.yaw_pid[1],
            self.yaw_pid[2],
            self.d_min[2],
            self.yaw_pid[3],
            // PID Controller Settings
            pid_controller_section,
            // Step response
            self.step_overshoot[0] * 100.0,
            self.step_undershoot[0] * 100.0,
            self.step_overshoot[1] * 100.0,
            self.step_undershoot[1] * 100.0,
            self.step_overshoot[2] * 100.0,
            self.step_undershoot[2] * 100.0,
            // Noise
            self.gyro_noise_rms[0],
            self.gyro_noise_rms[1],
            self.gyro_noise_rms[2],
            self.dterm_noise_rms[0],
            self.dterm_noise_rms[1],
            // Tracking
            self.tracking_error_rms[0],
            self.tracking_error_rms[1],
            self.tracking_error_rms[2],
            // Motors
            self.motor_saturation_pct,
            // Propwash
            self.propwash_severity,
            self.propwash_events,
            self.propwash_worst_axis,
            // Latency
            self.avg_latency_ms,
            self.system_latency_ms[0],
            self.system_latency_ms[1],
            self.system_latency_ms[2],
            // I-term
            self.iterm_windup_events,
            self.iterm_drift[0],
            self.iterm_drift[1],
            self.iterm_drift[2],
            // Filters & Rates
            filter_section,
            rate_section,
            // Firmware mentions
            self.firmware,
            self.firmware,
        )
    }
}

/// Result of AI analysis
pub enum AIAnalysisResult {
    Success(String),
    Error(String),
}

/// Client for OpenRouter API
pub struct OpenRouterClient;

impl OpenRouterClient {
    /// Analyze blocking (native only)
    #[cfg(not(target_arch = "wasm32"))]
    pub fn analyze_blocking(
        api_key: &str,
        model_id: &str,
        metrics: &FlightMetrics,
    ) -> AIAnalysisResult {
        let client = match reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_secs(60))
            .build()
        {
            Ok(c) => c,
            Err(e) => {
                return AIAnalysisResult::Error(format!("Failed to create HTTP client: {}", e))
            }
        };

        let request = Self::build_request(model_id, metrics);
        let response = client
            .post("https://openrouter.ai/api/v1/chat/completions")
            .header("Authorization", format!("Bearer {}", api_key))
            .header("Content-Type", "application/json")
            .header("HTTP-Referer", "https://github.com/bucksaw")
            .json(&request)
            .send();

        Self::handle_chat_response(response)
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn handle_chat_response(
        response: Result<reqwest::blocking::Response, reqwest::Error>,
    ) -> AIAnalysisResult {
        match response {
            Ok(resp) => {
                let status = resp.status();
                let body = resp.text().unwrap_or_default();
                Self::process_chat_body(status, &body)
            }
            Err(e) => AIAnalysisResult::Error(format!("Request failed: {}", e)),
        }
    }

    #[cfg(target_arch = "wasm32")]
    async fn analyze_async_internal(
        api_key: String,
        model_id: String,
        metrics: FlightMetrics,
    ) -> AIAnalysisResult {
        let client = reqwest::Client::new();
        let request = Self::build_request(&model_id, &metrics);

        // Note: Wasm fetches don't support custom timeouts easily in reqwest 0.11 without manual AbortController usage
        // but default browser timeout applies.

        let response = client
            .post("https://openrouter.ai/api/v1/chat/completions")
            .header("Authorization", format!("Bearer {}", api_key))
            .header("Content-Type", "application/json")
            .header("HTTP-Referer", "https://github.com/bucksaw")
            .json(&request)
            .send()
            .await;

        match response {
            Ok(resp) => {
                let status = resp.status();
                let body = resp.text().await.unwrap_or_default();
                Self::process_chat_body(status, &body)
            }
            Err(e) => AIAnalysisResult::Error(format!("Request failed: {}", e)),
        }
    }

    fn build_request(model_id: &str, metrics: &FlightMetrics) -> ChatRequest {
        let system_prompt = r#"You are an expert FPV drone tuning specialist with deep knowledge of Betaflight, PID tuning, and filter configuration. You analyze blackbox logs and provide actionable tuning advice.

Your recommendations should be:
- Specific and actionable (include exact CLI commands when suggesting changes)
- Conservative (small incremental changes are safer)
- Prioritized (address the most critical issues first)
- Educational (briefly explain why each change helps)

Format CLI commands in code blocks like: `set p_pitch = 50`"#;

        let user_prompt = metrics.to_prompt();

        // Log the AI prompt for debugging
        crate::app_logger::log_info(
            crate::app_logger::LogCategory::AI,
            format!("Using model: {}", model_id),
        );
        crate::app_logger::log_ai_prompt(&user_prompt);

        ChatRequest {
            model: model_id.to_string(),
            messages: vec![
                ChatMessage {
                    role: "system".to_string(),
                    content: system_prompt.to_string(),
                },
                ChatMessage {
                    role: "user".to_string(),
                    content: user_prompt,
                },
            ],
            max_tokens: 2000,
        }
    }

    fn process_chat_body(status: reqwest::StatusCode, body: &str) -> AIAnalysisResult {
        if status.is_success() {
            match serde_json::from_str::<ChatResponse>(body) {
                Ok(chat_resp) => {
                    if let Some(choice) = chat_resp.choices.first() {
                        AIAnalysisResult::Success(choice.message.content.clone())
                    } else {
                        AIAnalysisResult::Error("Empty response from AI".to_string())
                    }
                }
                Err(e) => AIAnalysisResult::Error(format!("Failed to parse response: {}", e)),
            }
        } else {
            if let Ok(err_resp) = serde_json::from_str::<ErrorResponse>(body) {
                if let Some(detail) = err_resp.error {
                    return AIAnalysisResult::Error(format!("API Error: {}", detail.message));
                }
            }
            AIAnalysisResult::Error(format!("API Error ({}): {}", status, body))
        }
    }

    /// Start analysis in background, returns receiver for result
    pub fn analyze_async(
        api_key: String,
        model_id: String,
        metrics: FlightMetrics,
    ) -> mpsc::Receiver<AIAnalysisResult> {
        let (tx, rx) = mpsc::channel();

        #[cfg(not(target_arch = "wasm32"))]
        thread::spawn(move || {
            let result = Self::analyze_blocking(&api_key, &model_id, &metrics);
            let _ = tx.send(result);
        });

        #[cfg(target_arch = "wasm32")]
        spawn_local(async move {
            let result = Self::analyze_async_internal(api_key, model_id, metrics).await;
            let _ = tx.send(result);
        });

        rx
    }
}

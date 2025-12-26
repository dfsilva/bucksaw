//! Firebase/Google Analytics 4 integration using the Measurement Protocol
//!
//! This module provides a unified interface to log events to Google Analytics 4
//! (which powers Firebase Analytics) from both native and WASM builds using
//! the GA4 Measurement Protocol.
//!
//! Configuration:
//! Set your GA4 credentials via environment variables or hardcode them:
//! - GA4_MEASUREMENT_ID: Your Measurement ID (e.g., "G-XXXXXXXXXX")
//! - GA4_API_SECRET: Your API secret from GA4 Admin > Data Streams > Measurement Protocol

use std::sync::OnceLock;

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

// ============================================================================
// Configuration
// ============================================================================

/// GA4 Measurement Protocol configuration
///
/// Set these via environment variables at build time, or replace the defaults:
/// - GA4_MEASUREMENT_ID: Your Measurement ID (e.g., "G-XXXXXXXXXX")
///   This is the `measurementId` from your Firebase config
/// - GA4_API_SECRET: Your API secret from GA4 Admin > Data Streams > Measurement Protocol
///
/// To build with analytics enabled:
/// ```
/// GA4_MEASUREMENT_ID=G-XXXXXX GA4_API_SECRET=your_secret cargo build --release
/// ```
const GA4_MEASUREMENT_ID: &str = match option_env!("GA4_MEASUREMENT_ID") {
    Some(id) => id,
    None => "", // Replace with your Measurement ID or set env var
};

const GA4_API_SECRET: &str = match option_env!("GA4_API_SECRET") {
    Some(secret) => secret,
    None => "", // Replace with your API secret or set env var
};

/// Check if analytics is properly configured
fn is_configured() -> bool {
    !GA4_MEASUREMENT_ID.starts_with("G-XXXX") && !GA4_API_SECRET.is_empty()
}

// ============================================================================
// Client ID Management
// ============================================================================

/// Get or generate a persistent client ID for this user
fn get_client_id() -> String {
    static CLIENT_ID: OnceLock<String> = OnceLock::new();
    CLIENT_ID
        .get_or_init(|| {
            // Try to load from storage, or generate a new one
            load_or_generate_client_id()
        })
        .clone()
}

#[cfg(target_arch = "wasm32")]
fn load_or_generate_client_id() -> String {
    use web_sys::window;

    let storage_key = "ga4_client_id";

    if let Some(window) = window() {
        if let Ok(Some(storage)) = window.local_storage() {
            // Try to load existing client ID
            if let Ok(Some(id)) = storage.get_item(storage_key) {
                return id;
            }
            // Generate and save new client ID
            let new_id = generate_client_id();
            let _ = storage.set_item(storage_key, &new_id);
            return new_id;
        }
    }
    generate_client_id()
}

#[cfg(not(target_arch = "wasm32"))]
fn load_or_generate_client_id() -> String {
    use std::fs;
    use std::path::PathBuf;

    let config_path = dirs::config_dir()
        .unwrap_or_else(|| PathBuf::from("."))
        .join("pid-lab")
        .join("client_id");

    // Try to load existing client ID
    if let Ok(id) = fs::read_to_string(&config_path) {
        let id = id.trim().to_string();
        if !id.is_empty() {
            return id;
        }
    }

    // Generate and save new client ID
    let new_id = generate_client_id();
    if let Some(parent) = config_path.parent() {
        let _ = fs::create_dir_all(parent);
    }
    let _ = fs::write(&config_path, &new_id);
    new_id
}

#[cfg(not(target_arch = "wasm32"))]
fn generate_client_id() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};

    let timestamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis())
        .unwrap_or(0);

    // Simple pseudo-random component
    let random: u64 = (timestamp as u64)
        .wrapping_mul(1103515245)
        .wrapping_add(12345);

    format!("{}.{}", random % 1_000_000_000, timestamp)
}

#[cfg(target_arch = "wasm32")]
fn generate_client_id() -> String {
    let timestamp = js_sys::Date::now();
    let random = js_sys::Math::random();

    // Mimic the format used in native: random part . timestamp
    format!("{}.{}", (random * 1_000_000_000.0) as u64, timestamp as u64)
}

// ============================================================================
// Event Sending
// ============================================================================

/// Log an event to Google Analytics 4 using the Measurement Protocol
///
/// # Arguments
/// * `event_name` - The name of the event (e.g., "tab_selected", "file_opened")
/// * `params` - Optional parameters as key-value pairs
pub fn log_event(event_name: &str, params: Option<&[(&str, serde_json::Value)]>) {
    if !is_configured() {
        log::debug!(
            "Analytics not configured. Event: {} params: {:?}",
            event_name,
            params
        );
        return;
    }

    let client_id = get_client_id();
    let event_name = event_name.to_string();
    let params_owned: Vec<(String, serde_json::Value)> = params
        .unwrap_or(&[])
        .iter()
        .map(|(k, v)| (k.to_string(), v.clone()))
        .collect();

    // Send asynchronously to not block the UI
    send_event_async(client_id, event_name, params_owned);
}

#[cfg(not(target_arch = "wasm32"))]
fn send_event_async(
    client_id: String,
    event_name: String,
    params: Vec<(String, serde_json::Value)>,
) {
    std::thread::spawn(move || {
        if let Err(e) = send_event_blocking(&client_id, &event_name, &params) {
            log::warn!("Failed to send analytics event '{}': {}", event_name, e);
        }
    });
}

#[cfg(target_arch = "wasm32")]
fn send_event_async(
    client_id: String,
    event_name: String,
    params: Vec<(String, serde_json::Value)>,
) {
    wasm_bindgen_futures::spawn_local(async move {
        if let Err(e) = send_event_wasm(&client_id, &event_name, &params).await {
            log::warn!("Failed to send analytics event '{}': {:?}", event_name, e);
        }
    });
}

#[cfg(not(target_arch = "wasm32"))]
fn send_event_blocking(
    client_id: &str,
    event_name: &str,
    params: &[(String, serde_json::Value)],
) -> Result<(), String> {
    let url = format!(
        "https://www.google-analytics.com/mp/collect?measurement_id={}&api_secret={}",
        GA4_MEASUREMENT_ID, GA4_API_SECRET
    );

    let mut event_params = serde_json::Map::new();
    for (key, value) in params {
        event_params.insert(key.clone(), value.clone());
    }

    let payload = serde_json::json!({
        "client_id": client_id,
        "events": [{
            "name": event_name,
            "params": event_params
        }]
    });

    let client = reqwest::blocking::Client::builder()
        .timeout(std::time::Duration::from_secs(5))
        .build()
        .map_err(|e| e.to_string())?;

    let response = client
        .post(&url)
        .header("Content-Type", "application/json")
        .body(payload.to_string())
        .send()
        .map_err(|e| e.to_string())?;

    if response.status().is_success() {
        log::trace!("Analytics event '{}' sent successfully", event_name);
        Ok(())
    } else {
        Err(format!("HTTP {}", response.status()))
    }
}

#[cfg(target_arch = "wasm32")]
async fn send_event_wasm(
    client_id: &str,
    event_name: &str,
    params: &[(String, serde_json::Value)],
) -> Result<(), JsValue> {
    let url = format!(
        "https://www.google-analytics.com/mp/collect?measurement_id={}&api_secret={}",
        GA4_MEASUREMENT_ID, GA4_API_SECRET
    );

    let mut event_params = serde_json::Map::new();
    for (key, value) in params {
        event_params.insert(key.clone(), value.clone());
    }

    let payload = serde_json::json!({
        "client_id": client_id,
        "events": [{
            "name": event_name,
            "params": event_params
        }]
    });

    let client = reqwest::Client::new();
    let _response = client
        .post(&url)
        // Use text/plain to avoid CORS preflight (OPTIONS request) which fails
        // GA4 MP typically accepts JSON in body even with text/plain
        .header("Content-Type", "text/plain")
        .body(payload.to_string())
        .send()
        .await
        .map_err(|e| JsValue::from_str(&e.to_string()))?;

    log::trace!("Analytics event '{}' sent successfully", event_name);
    Ok(())
}

// ============================================================================
// Helper macro for creating params
// ============================================================================

/// Helper to create event parameters
#[macro_export]
macro_rules! analytics_params {
    ($($key:expr => $value:expr),* $(,)?) => {
        Some(&[
            $(($key, serde_json::json!($value))),*
        ] as &[(&str, serde_json::Value)])
    };
}

// ============================================================================
// Tab & Navigation Events
// ============================================================================

/// Log a tab selection event
pub fn log_tab_selected(tab_name: &str) {
    log_event("tab_selected", analytics_params!("tab_name" => tab_name));
}

// ============================================================================
// File & Flight Events
// ============================================================================

/// Log a file opened event
pub fn log_file_opened(file_name: &str, flight_count: usize) {
    let extension = file_name
        .rsplit('.')
        .next()
        .unwrap_or("unknown")
        .to_lowercase();
    log_event(
        "file_opened",
        analytics_params!(
            "file_type" => extension,
            "flight_count" => flight_count
        ),
    );
}

/// Log a file closed event
pub fn log_file_closed() {
    log_event("file_closed", None);
}

/// Log a flight selected event
pub fn log_flight_selected(flight_index: usize, file_index: usize) {
    log_event(
        "flight_selected",
        analytics_params!(
            "flight_index" => flight_index,
            "file_index" => file_index
        ),
    );
}

// ============================================================================
// AI Analysis Events
// ============================================================================

/// Log when AI analysis is started
pub fn log_ai_analysis_started(model_id: &str, focus: &str) {
    log_event(
        "ai_analysis_started",
        analytics_params!(
            "model_id" => model_id,
            "focus" => focus
        ),
    );
}

/// Log when AI analysis completes successfully
pub fn log_ai_analysis_completed(model_id: &str) {
    log_event(
        "ai_analysis_completed",
        analytics_params!("model_id" => model_id),
    );
}

/// Log when AI analysis fails
pub fn log_ai_analysis_failed(error_type: &str) {
    log_event(
        "ai_analysis_failed",
        analytics_params!("error_type" => error_type),
    );
}

/// Log when AI response is copied
pub fn log_ai_response_copied() {
    log_event("ai_response_copied", None);
}

/// Log when AI settings are saved
pub fn log_ai_settings_saved() {
    log_event("ai_settings_saved", None);
}

// ============================================================================
// Tuning & Analysis Events
// ============================================================================

/// Log when step response analysis settings are changed
pub fn log_step_response_settings(smoothing: &str, min_threshold: f32) {
    log_event(
        "step_response_settings",
        analytics_params!(
            "smoothing" => smoothing,
            "min_threshold" => min_threshold
        ),
    );
}

/// Log when anomaly is detected and user views it
pub fn log_anomaly_viewed(anomaly_type: &str, time_start: f64) {
    log_event(
        "anomaly_viewed",
        analytics_params!(
            "anomaly_type" => anomaly_type,
            "time_start" => time_start
        ),
    );
}

/// Log anomaly jump/navigation action
pub fn log_anomaly_jump(anomaly_type: &str) {
    log_event(
        "anomaly_jump",
        analytics_params!("anomaly_type" => anomaly_type),
    );
}

/// Log when CLI commands are copied
pub fn log_cli_commands_copied(command_count: usize, severity_filter: Option<&str>) {
    let severity = severity_filter.unwrap_or("all");
    log_event(
        "cli_commands_copied",
        analytics_params!(
            "command_count" => command_count,
            "severity" => severity
        ),
    );
}

// ============================================================================
// Vibe & FFT Events
// ============================================================================

/// Log when vibe analysis domain is changed
pub fn log_vibe_domain_changed(domain: &str) {
    log_event("vibe_domain_changed", analytics_params!("domain" => domain));
}

/// Log when FFT size is changed
pub fn log_fft_size_changed(fft_size: usize) {
    log_event(
        "fft_size_changed",
        analytics_params!("fft_size" => fft_size),
    );
}

/// Log when colorscheme is changed
pub fn log_colorscheme_changed(colorscheme: &str) {
    log_event(
        "colorscheme_changed",
        analytics_params!("colorscheme" => colorscheme),
    );
}

// ============================================================================
// Filter Analysis Events
// ============================================================================

/// Log when filter analysis is viewed
pub fn log_filter_analysis_viewed() {
    log_event("filter_analysis_viewed", None);
}

// ============================================================================
// UI Events
// ============================================================================

/// Log when dark/light mode is toggled
pub fn log_theme_changed(is_dark: bool) {
    let theme = if is_dark { "dark" } else { "light" };
    log_event("theme_changed", analytics_params!("theme" => theme));
}

/// Log when side panel is toggled
pub fn log_panel_toggled(panel_name: &str, is_open: bool) {
    log_event(
        "panel_toggled",
        analytics_params!(
            "panel" => panel_name,
            "is_open" => is_open
        ),
    );
}

/// Log when time range filter is applied
pub fn log_time_range_filter(enabled: bool, duration_secs: f64) {
    log_event(
        "time_range_filter",
        analytics_params!(
            "enabled" => enabled,
            "duration_secs" => duration_secs
        ),
    );
}

// ============================================================================
// Error & Performance Events
// ============================================================================

/// Log when a parse error occurs
pub fn log_parse_error(error_type: &str) {
    log_event("parse_error", analytics_params!("error_type" => error_type));
}

/// Log flight duration for engagement analysis
pub fn log_flight_duration(duration_secs: f64) {
    log_event(
        "flight_analyzed",
        analytics_params!("duration_secs" => duration_secs),
    );
}

// ============================================================================
// Export Events
// ============================================================================

/// Log when app logs are copied
pub fn log_logs_copied() {
    log_event("logs_copied", None);
}

/// Log when logs are cleared
pub fn log_logs_cleared() {
    log_event("logs_cleared", None);
}

// ============================================================================
// App Lifecycle Events
// ============================================================================

/// Log app start event
pub fn log_app_started() {
    log_event("app_started", None);
}

/// Log app version for tracking
pub fn log_app_version(version: &str) {
    log_event("app_info", analytics_params!("version" => version));
}

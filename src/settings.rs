//! Settings persistence module for PID Lab
//! Supports both native (file-based) and WASM (localStorage) targets

use serde::{Deserialize, Serialize};

/// Persisted AI integration settings
#[derive(Clone, Default, Serialize, Deserialize)]
pub struct AISettings {
    /// OpenRouter API key (stored encrypted in future versions)
    pub api_key: String,
    /// Selected model ID
    pub selected_model_id: Option<String>,
}

/// All persisted application settings
#[derive(Clone, Default, Serialize, Deserialize)]
pub struct AppSettings {
    pub ai: AISettings,
}

const SETTINGS_KEY: &str = "pid_lab_settings";

impl AppSettings {
    /// Load settings from persistent storage
    pub fn load() -> Self {
        #[cfg(target_arch = "wasm32")]
        {
            Self::load_wasm()
        }

        #[cfg(not(target_arch = "wasm32"))]
        {
            Self::load_native()
        }
    }

    /// Save settings to persistent storage
    pub fn save(&self) {
        #[cfg(target_arch = "wasm32")]
        {
            self.save_wasm();
        }

        #[cfg(not(target_arch = "wasm32"))]
        {
            self.save_native();
        }
    }

    // ========== Native Implementation ==========

    #[cfg(not(target_arch = "wasm32"))]
    fn config_path() -> Option<std::path::PathBuf> {
        dirs::config_dir().map(|p| p.join("pid-lab").join("settings.json"))
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn load_native() -> Self {
        let Some(path) = Self::config_path() else {
            log::warn!("Could not determine config directory");
            return Self::default();
        };

        match std::fs::read_to_string(&path) {
            Ok(content) => match serde_json::from_str(&content) {
                Ok(settings) => {
                    log::info!("Loaded settings from {:?}", path);
                    settings
                }
                Err(e) => {
                    log::warn!("Failed to parse settings: {}", e);
                    Self::default()
                }
            },
            Err(_) => {
                // File doesn't exist yet, that's fine
                Self::default()
            }
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn save_native(&self) {
        let Some(path) = Self::config_path() else {
            log::warn!("Could not determine config directory");
            return;
        };

        // Create parent directories
        if let Some(parent) = path.parent() {
            if let Err(e) = std::fs::create_dir_all(parent) {
                log::warn!("Failed to create config directory: {}", e);
                return;
            }
        }

        match serde_json::to_string_pretty(self) {
            Ok(content) => {
                if let Err(e) = std::fs::write(&path, content) {
                    log::warn!("Failed to save settings: {}", e);
                } else {
                    log::info!("Saved settings to {:?}", path);
                }
            }
            Err(e) => {
                log::warn!("Failed to serialize settings: {}", e);
            }
        }
    }

    // ========== WASM Implementation ==========

    #[cfg(target_arch = "wasm32")]
    fn load_wasm() -> Self {
        use wasm_bindgen::JsCast;

        let window = match web_sys::window() {
            Some(w) => w,
            None => return Self::default(),
        };

        let storage = match window.local_storage() {
            Ok(Some(s)) => s,
            _ => return Self::default(),
        };

        match storage.get_item(SETTINGS_KEY) {
            Ok(Some(value)) => match serde_json::from_str(&value) {
                Ok(settings) => {
                    log::info!("Loaded settings from localStorage");
                    settings
                }
                Err(e) => {
                    log::warn!("Failed to parse settings: {}", e);
                    Self::default()
                }
            },
            _ => Self::default(),
        }
    }

    #[cfg(target_arch = "wasm32")]
    fn save_wasm(&self) {
        use wasm_bindgen::JsCast;

        let window = match web_sys::window() {
            Some(w) => w,
            None => {
                log::warn!("No window object available");
                return;
            }
        };

        let storage = match window.local_storage() {
            Ok(Some(s)) => s,
            _ => {
                log::warn!("localStorage not available");
                return;
            }
        };

        match serde_json::to_string(self) {
            Ok(value) => {
                if let Err(e) = storage.set_item(SETTINGS_KEY, &value) {
                    log::warn!("Failed to save to localStorage: {:?}", e);
                } else {
                    log::info!("Saved settings to localStorage");
                }
            }
            Err(e) => {
                log::warn!("Failed to serialize settings: {}", e);
            }
        }
    }
}

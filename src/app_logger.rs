use std::sync::Mutex;

/// Log level for categorizing messages
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum LogLevel {
    Debug,
    Info,
    Warning,
    Error,
}

impl LogLevel {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Debug => "DEBUG",
            Self::Info => "INFO",
            Self::Warning => "WARN",
            Self::Error => "ERROR",
        }
    }
}

/// Category for filtering logs
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum LogCategory {
    General,
    AI,
    Analysis,
    File,
}

impl LogCategory {
    pub fn label(&self) -> &'static str {
        match self {
            Self::General => "General",
            Self::AI => "AI",
            Self::Analysis => "Analysis",
            Self::File => "File",
        }
    }

    pub fn all() -> &'static [LogCategory] {
        &[
            LogCategory::General,
            LogCategory::AI,
            LogCategory::Analysis,
            LogCategory::File,
        ]
    }
}

/// A single log entry
#[derive(Clone, Debug)]
pub struct LogEntry {
    pub timestamp: String,
    pub level: LogLevel,
    pub category: LogCategory,
    pub message: String,
}

/// Global application logger
pub struct AppLogger {
    entries: Mutex<Vec<LogEntry>>,
}

impl AppLogger {
    const fn new() -> Self {
        Self {
            entries: Mutex::new(Vec::new()),
        }
    }

    fn get_timestamp() -> String {
        #[cfg(not(target_arch = "wasm32"))]
        {
            use std::time::SystemTime;
            let now = SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap_or_default();
            let secs = now.as_secs();
            let hours = (secs / 3600) % 24;
            let minutes = (secs / 60) % 60;
            let seconds = secs % 60;
            let millis = now.subsec_millis();
            format!("{:02}:{:02}:{:02}.{:03}", hours, minutes, seconds, millis)
        }

        #[cfg(target_arch = "wasm32")]
        {
            let date = js_sys::Date::new_0();
            let hours = date.get_hours();
            let minutes = date.get_minutes();
            let seconds = date.get_seconds();
            let millis = date.get_milliseconds();
            format!("{:02}:{:02}:{:02}.{:03}", hours, minutes, seconds, millis)
        }
    }

    pub fn log(&self, level: LogLevel, category: LogCategory, message: String) {
        if let Ok(mut entries) = self.entries.lock() {
            entries.push(LogEntry {
                timestamp: Self::get_timestamp(),
                level,
                category,
                message,
            });
            // Keep max 1000 entries to prevent memory issues
            if entries.len() > 1000 {
                entries.remove(0);
            }
        }
    }

    pub fn get_entries(&self) -> Vec<LogEntry> {
        self.entries.lock().map(|e| e.clone()).unwrap_or_default()
    }

    pub fn clear(&self) {
        if let Ok(mut entries) = self.entries.lock() {
            entries.clear();
        }
    }
}

// Global logger instance
static LOGGER: AppLogger = AppLogger::new();

/// Get the global logger
pub fn logger() -> &'static AppLogger {
    &LOGGER
}

/// Log an info message
pub fn log_info(category: LogCategory, message: impl Into<String>) {
    logger().log(LogLevel::Info, category, message.into());
}

/// Log a debug message
pub fn log_debug(category: LogCategory, message: impl Into<String>) {
    logger().log(LogLevel::Debug, category, message.into());
}

/// Log a warning message
pub fn log_warning(category: LogCategory, message: impl Into<String>) {
    logger().log(LogLevel::Warning, category, message.into());
}

/// Log an error message
pub fn log_error(category: LogCategory, message: impl Into<String>) {
    logger().log(LogLevel::Error, category, message.into());
}

/// Log an AI prompt (special convenience function)
pub fn log_ai_prompt(prompt: impl Into<String>) {
    logger().log(
        LogLevel::Info,
        LogCategory::AI,
        format!("AI Prompt:\n{}", prompt.into()),
    );
}

/// Log an AI response
pub fn log_ai_response(response: impl Into<String>) {
    logger().log(
        LogLevel::Info,
        LogCategory::AI,
        format!("AI Response:\n{}", response.into()),
    );
}

mod anomalies;
mod dashboard;
mod error;
mod feedforward;
mod tuning_guide;

mod filter;
mod logs;
mod plot;
mod setup;
mod stats;
mod suggestions;
mod tune;
mod vibe;

use std::fmt::Display;

pub use anomalies::*;
pub use dashboard::*;
pub use error::*;
pub use feedforward::*;
pub use tuning_guide::*;

pub use filter::*;
pub use logs::*;
pub use plot::*;
pub use setup::*;
pub use stats::*;
pub use suggestions::*;
pub use tune::*;
pub use vibe::*;

const PLOT_HEIGHT: f32 = 300.0;
const MIN_WIDE_WIDTH: f32 = 1000.0;

#[derive(Default, Clone, Copy, PartialEq)]
pub enum FlightViewTab {
    #[default]
    Dashboard,
    Plot,
    Tune,
    Vibe,
    Stats,
    Anomalies,
    Error,
    Setup,
    Suggestions,
    Filter,
    Logs,
    Feedforward,
    TuningGuide,
}

impl Display for FlightViewTab {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let val = match self {
            Self::Dashboard => "▦ Dashboard",
            Self::Plot => "∿ Plot",
            Self::Tune => "⚙ Tune",
            Self::Vibe => "≈ Vibe",
            Self::Stats => "Σ Stats",
            Self::Anomalies => "⚠ Anomalies",
            Self::Error => "× Error",
            Self::Setup => "☰ Setup",
            Self::Suggestions => "★ Suggestions",
            Self::Filter => "⫶ Filter",
            Self::Logs => "≡ Logs",
            Self::Feedforward => "➤ Feedforward",
            Self::TuningGuide => "▷ Guide",
        };

        write!(f, "{val}",)
    }
}

impl FlightViewTab {
    /// Returns a clean name suitable for analytics tracking
    pub fn analytics_name(&self) -> &'static str {
        match self {
            Self::Dashboard => "Dashboard",
            Self::Plot => "Plot",
            Self::Tune => "Tune",
            Self::Vibe => "Vibe",
            Self::Stats => "Stats",
            Self::Anomalies => "Anomalies",
            Self::Error => "Error",
            Self::Setup => "Setup",
            Self::Suggestions => "Suggestions",
            Self::Filter => "Filter",
            Self::Logs => "Logs",
            Self::Feedforward => "Feedforward",
            Self::TuningGuide => "TuningGuide",
        }
    }
}

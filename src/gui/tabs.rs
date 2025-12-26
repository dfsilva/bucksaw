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
use egui_phosphor::regular as icons;

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

#[derive(Default, Clone, Copy, PartialEq, Debug)]
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

impl FlightViewTab {
    /// Returns display name with icon
    pub fn label(&self) -> String {
        match self {
            Self::Dashboard => format!("{} Dashboard", icons::SQUARES_FOUR),
            Self::Plot => format!("{} Plot", icons::CHART_LINE),
            Self::Tune => format!("{} Tune", icons::GEAR),
            Self::Vibe => format!("{} Vibe", icons::WAVE_SINE),
            Self::Stats => format!("{} Stats", icons::CHART_BAR),
            Self::Anomalies => format!("{} Anomalies", icons::WARNING),
            Self::Error => format!("{} Error", icons::X_CIRCLE),
            Self::Setup => format!("{} Setup", icons::SLIDERS),
            Self::Suggestions => format!("{} Suggestions", icons::LIGHTBULB),
            Self::Filter => format!("{} Filter", icons::FUNNEL),
            Self::Logs => format!("{} Logs", icons::LIST_BULLETS),
            Self::Feedforward => format!("{} Feedforward", icons::ARROW_FAT_RIGHT),
            Self::TuningGuide => format!("{} Guide", icons::BOOK_OPEN),
        }
    }

    /// Returns short label without icon (for compact displays)
    pub fn short_label(&self) -> &'static str {
        match self {
            Self::Dashboard => "Dashboard",
            Self::Plot => "Plot",
            Self::Tune => "Tune",
            Self::Vibe => "Vibe",
            Self::Stats => "Stats",
            Self::Anomalies => "Anomalies",
            Self::Error => "Error",
            Self::Setup => "Setup",
            Self::Suggestions => "Suggest",
            Self::Filter => "Filter",
            Self::Logs => "Logs",
            Self::Feedforward => "FF",
            Self::TuningGuide => "Guide",
        }
    }
    
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

impl Display for FlightViewTab {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.label())
    }
}

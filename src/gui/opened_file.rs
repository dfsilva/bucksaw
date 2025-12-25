use std::sync::Arc;

use blackbox_log::headers::ParseError;

use crate::flight_data::FlightData;
use crate::gui::flight_view::FlightView;

pub type FlightDataAndView = Vec<(Result<Arc<FlightData>, ParseError>, Option<FlightView>)>;

/// Represents an opened blackbox log file with its flights
pub struct OpenedFile {
    /// Name of the file (for display in sidebar)
    pub file_name: String,
    /// Parsed flights and their views
    pub flights: FlightDataAndView,
    /// Whether the file is expanded in the sidebar
    pub expanded: bool,
}

impl OpenedFile {
    pub fn new(file_name: String, flights: FlightDataAndView) -> Self {
        Self {
            file_name,
            flights,
            expanded: true, // Start expanded by default
        }
    }
}

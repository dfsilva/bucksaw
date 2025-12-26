use std::sync::Arc;

use egui::{Color32, RichText};
use egui_oszi::TimeseriesGroup;

use crate::flight_data::FlightData;
use crate::gui::tabs::*;

// Re-export NavigationAction for use by parent components
pub use crate::gui::tabs::NavigationAction;

/// Time range filter for focused analysis
#[derive(Clone)]
pub struct TimeRangeFilter {
    /// Whether filtering is enabled
    pub enabled: bool,
    /// Start time in seconds
    pub start: f64,
    /// End time in seconds
    pub end: f64,
    /// Total flight duration for reference
    total_duration: f64,
}

impl TimeRangeFilter {
    pub fn new(total_duration: f64) -> Self {
        Self {
            enabled: false,
            start: 0.0,
            end: total_duration,
            total_duration,
        }
    }

    /// Reset to full range
    pub fn reset(&mut self) {
        self.start = 0.0;
        self.end = self.total_duration;
        self.enabled = false;
    }

    /// Get the current range as a tuple
    pub fn range(&self) -> (f64, f64) {
        if self.enabled {
            (self.start, self.end)
        } else {
            (0.0, self.total_duration)
        }
    }

    /// Check if a time value is within the selected range
    pub fn contains(&self, time: f64) -> bool {
        if !self.enabled {
            return true;
        }
        time >= self.start && time <= self.end
    }

    /// Show the time range filter UI
    pub fn show(&mut self, ui: &mut egui::Ui) {
        egui::Frame::none()
            .fill(ui.style().visuals.extreme_bg_color)
            .rounding(4.0)
            .inner_margin(8.0)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    // Enable checkbox
                    ui.checkbox(&mut self.enabled, "");

                    ui.label(RichText::new("◔ Time Range:").strong());

                    if self.enabled {
                        // Start slider
                        ui.label("Start:");
                        ui.add(
                            egui::Slider::new(&mut self.start, 0.0..=self.total_duration)
                                .suffix("s")
                                .max_decimals(1),
                        );

                        // End slider
                        ui.label("End:");
                        ui.add(
                            egui::Slider::new(&mut self.end, 0.0..=self.total_duration)
                                .suffix("s")
                                .max_decimals(1),
                        );

                        // Ensure start < end
                        if self.start > self.end {
                            std::mem::swap(&mut self.start, &mut self.end);
                        }

                        // Duration display
                        let duration = self.end - self.start;
                        ui.label(RichText::new(format!("({:.1}s selected)", duration)).color(
                            if duration < 1.0 {
                                Color32::YELLOW
                            } else {
                                Color32::LIGHT_GREEN
                            },
                        ));

                        // Reset button
                        if ui.button("↺ Reset").clicked() {
                            self.reset();
                        }
                    } else {
                        ui.label(RichText::new("Full flight").weak());
                        ui.label(format!("({:.1}s)", self.total_duration));
                    }
                });
            });
    }
}

pub struct FlightView {
    plot_group: TimeseriesGroup,
    time_filter: TimeRangeFilter,
    dashboard_tab: DashboardTab,
    plot_tab: PlotTab,
    tune_tab: TuneTab,
    vibe_tab: VibeTab,
    stats_tab: StatsTab,
    error_tab: ErrorTab,
    setup_tab: SetupTab,
    suggestions_tab: SuggestionsTab,
    filter_tab: FilterTab,
    anomalies_tab: AnomaliesTab,
    logs_tab: LogsTab,
    feedforward_tab: FeedforwardTab,
    tuning_guide_tab: TuningGuideTab,
}

impl FlightView {
    pub fn new(ctx: &egui::Context, data: Arc<FlightData>) -> Self {
        let total_duration =
            data.times.last().copied().unwrap_or(0.0) - data.times.first().copied().unwrap_or(0.0);

        Self {
            time_filter: TimeRangeFilter::new(total_duration),
            dashboard_tab: DashboardTab::new(data.clone()),
            plot_tab: PlotTab::new(data.clone()),
            tune_tab: TuneTab::new(data.clone()),
            vibe_tab: VibeTab::new(ctx, data.clone()),
            stats_tab: StatsTab::new(data.clone()),
            error_tab: ErrorTab::new(data.clone()),
            setup_tab: SetupTab::new(data.clone()),
            suggestions_tab: SuggestionsTab::new(data.clone()),
            filter_tab: FilterTab::new(data.clone()),
            anomalies_tab: AnomaliesTab::new(data.clone()),
            logs_tab: LogsTab::new(),
            feedforward_tab: FeedforwardTab::new(data),
            tuning_guide_tab: TuningGuideTab::new(),
            plot_group: TimeseriesGroup::new("timeseries_plots", false),
        }
    }

    /// Set time range from external navigation (e.g., Anomalies tab)
    pub fn set_time_range(&mut self, start: f64, end: f64) {
        self.time_filter.start = start;
        self.time_filter.end = end;
        self.time_filter.enabled = true;
    }

    /// Returns an optional NavigationAction if the user requested navigation
    /// (e.g., jumping to a specific time from the Anomalies tab)
    pub fn show(&mut self, ui: &mut egui::Ui, tab: FlightViewTab) -> Option<NavigationAction> {
        let mut nav_action = None;

        // Show time range filter for relevant tabs
        let show_time_filter = matches!(
            tab,
            FlightViewTab::Plot
                | FlightViewTab::Tune
                | FlightViewTab::Vibe
                | FlightViewTab::Stats
                | FlightViewTab::Error
        );

        if show_time_filter {
            self.time_filter.show(ui);
            ui.add_space(4.0);
        }

        ui.vertical(|ui| match tab {
            FlightViewTab::Dashboard => self.dashboard_tab.show(ui),
            FlightViewTab::Plot => {
                egui::ScrollArea::vertical()
                    .show(ui, |ui| self.plot_tab.show(ui, &mut self.plot_group));
            }
            FlightViewTab::Tune => self.tune_tab.show(ui, &mut self.plot_group),
            FlightViewTab::Vibe => self.vibe_tab.show(ui),
            FlightViewTab::Stats => self.stats_tab.show(ui),
            FlightViewTab::Error => self.error_tab.show(ui),
            FlightViewTab::Setup => self.setup_tab.show(ui),
            FlightViewTab::Suggestions => self.suggestions_tab.show(ui),
            FlightViewTab::Filter => self.filter_tab.show(ui),
            FlightViewTab::Anomalies => {
                nav_action = self.anomalies_tab.show(ui);
            }
            FlightViewTab::Logs => self.logs_tab.show(ui),
            FlightViewTab::Feedforward => self.feedforward_tab.show(ui),
            FlightViewTab::TuningGuide => self.tuning_guide_tab.show(ui),
        });

        nav_action
    }
}

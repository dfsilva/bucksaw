use std::sync::Arc;

use egui_oszi::TimeseriesGroup;

use crate::flight_data::FlightData;
use crate::gui::tabs::*;

pub struct FlightView {
    plot_group: TimeseriesGroup,
    plot_tab: PlotTab,
    tune_tab: TuneTab,
    vibe_tab: VibeTab,
    stats_tab: StatsTab,
    error_tab: ErrorTab,
    setup_tab: SetupTab,
    suggestions_tab: SuggestionsTab,
    filter_tab: FilterTab,
}

impl FlightView {
    pub fn new(ctx: &egui::Context, data: Arc<FlightData>) -> Self {
        Self {
            plot_tab: PlotTab::new(data.clone()),
            tune_tab: TuneTab::new(data.clone()),
            vibe_tab: VibeTab::new(ctx, data.clone()),
            stats_tab: StatsTab::new(data.clone()),
            error_tab: ErrorTab::new(data.clone()),
            setup_tab: SetupTab::new(data.clone()),
            suggestions_tab: SuggestionsTab::new(data.clone()),
            filter_tab: FilterTab::new(data),
            plot_group: TimeseriesGroup::new("timeseries_plots", false),
        }
    }

    pub fn show(&mut self, ui: &mut egui::Ui, tab: FlightViewTab) {
        ui.vertical(|ui| match tab {
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
        });
    }
}

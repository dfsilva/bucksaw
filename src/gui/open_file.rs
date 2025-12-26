use egui::Align2;
use egui::Vec2;
use egui::{Color32, RichText};
use egui_phosphor::regular as icons;

use std::fs::File;
use std::io::Read;
use std::path::PathBuf;

use std::sync::mpsc::{channel, Receiver, Sender};

use egui::ProgressBar;

use crate::log_file::*;
use crate::utils::execute_in_background;
use crate::utils::BackgroundCompStore;

pub struct OpenFileDialog {
    file: BackgroundCompStore<Option<LogFile>>,
    file_progress_receiver: Receiver<f32>,
    flight_progress_receiver: Receiver<f32>,
    file_name_receiver: Receiver<String>,
    cancel_sender: Option<Sender<()>>,

    file_progress: f32,
    flight_progress: f32,
    file_name: String,
    cancelled: bool,
}

impl OpenFileDialog {
    pub fn new(path: Option<PathBuf>) -> Self {
        // Setup channels for receiving progress and results
        let (flight_progress_sender, flight_progress_receiver) = channel();
        let (file_progress_sender, file_progress_receiver) = channel();
        let (file_sender, file_receiver) = channel();
        let (file_name_sender, file_name_receiver) = channel();
        let (cancel_sender, cancel_receiver) = channel::<()>();
        let file = BackgroundCompStore::new(file_receiver);

        // File parsing happens in the background task
        execute_in_background(async move {
            match Self::pick_read_file(path.clone()).await {
                Some((name, file_path, bytes)) => {
                    // Send file name for display
                    let _ = file_name_sender.send(name.clone());
                    
                    // Check for cancellation
                    if cancel_receiver.try_recv().is_ok() {
                        let _ = file_sender.send(None);
                        return;
                    }

                    let log_data =
                        LogFile::parse(name, file_path, bytes, file_progress_sender, flight_progress_sender)
                            .await;

                    let _ = file_sender.send(Some(log_data));
                }
                None => {
                    let _ = file_sender.send(None);
                }
            }
        });

        Self {
            file,
            file_progress_receiver,
            flight_progress_receiver,
            file_name_receiver,
            cancel_sender: Some(cancel_sender),

            file_progress: 0.0,
            flight_progress: 0.0,
            file_name: String::new(),
            cancelled: false,
        }
    }

    // This function needs to be async due to blocking rfd::FileDialog is not available on wasm32
    async fn pick_read_file(path: Option<PathBuf>) -> Option<(String, Option<PathBuf>, Vec<u8>)> {
        match path {
            Some(path) => {
                let name = path
                    .file_name()?
                    .to_str()
                    .to_owned()
                    .map(|f| f.to_string())?;
                let mut bytes = Vec::new();
                let mut f = File::open(&path).ok()?;
                f.read_to_end(&mut bytes).ok()?;
                Some((name, Some(path), bytes))
            }
            None => {
                let file = rfd::AsyncFileDialog::new()
                    .add_filter("Blackbox Logs", &["bbl", "bfl", "txt"])
                    .add_filter("All Files", &["*"])
                    .pick_file()
                    .await?;
                
                #[cfg(not(target_arch = "wasm32"))]
                let file_path = Some(file.path().to_path_buf());
                #[cfg(target_arch = "wasm32")]
                let file_path = None;

                Some((file.file_name(), file_path, file.read().await))
            }
        }
    }

    /// Cancel the current operation
    pub fn cancel(&mut self) {
        if let Some(sender) = self.cancel_sender.take() {
            let _ = sender.send(());
        }
        self.cancelled = true;
    }

    // Show Loading&parsing progress bars popup
    pub fn show(&mut self, ctx: &egui::Context) -> Option<Option<LogFile>> {
        // Process incoming messages
        while let Ok(flight_progress) = self.flight_progress_receiver.try_recv() {
            self.flight_progress = flight_progress;
        }

        while let Ok(file_progress) = self.file_progress_receiver.try_recv() {
            self.file_progress = file_progress;
        }

        while let Ok(file_name) = self.file_name_receiver.try_recv() {
            self.file_name = file_name;
        }

        // If cancelled, return None immediately
        if self.cancelled {
            return Some(None);
        }

        let mut should_cancel = false;

        egui::Window::new("Loading Blackbox Log")
            .anchor(Align2::CENTER_CENTER, Vec2::splat(0.0))
            .movable(false)
            .resizable(false)
            .collapsible(false)
            .min_width(f32::min(400.0, ctx.available_rect().width() - 40.0))
            .max_width(f32::min(400.0, ctx.available_rect().width() - 40.0))
            .show(ctx, |ui| {
                ui.vertical(|ui| {
                    ui.add_space(8.0);

                    // File name with icon
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(icons::FILE).size(18.0));
                        if self.file_name.is_empty() {
                            ui.label(RichText::new("Selecting file...").weak().italics());
                        } else {
                            ui.label(RichText::new(&self.file_name).strong());
                        }
                    });

                    ui.add_space(12.0);

                    // Reading file progress
                    ui.horizontal(|ui| {
                        ui.label("Reading file:");
                        ui.add_space(8.0);
                        if self.file_progress >= 1.0 {
                            ui.label(RichText::new(icons::CHECK_CIRCLE).color(Color32::from_rgb(0x8e, 0xc0, 0x7c)));
                        }
                    });
                    let file_pb = ProgressBar::new(self.file_progress)
                        .desired_width(ui.available_width())
                        .show_percentage()
                        .animate(true);
                    ui.add(file_pb);

                    ui.add_space(8.0);

                    // Parsing flights progress
                    ui.horizontal(|ui| {
                        ui.label("Parsing flights:");
                        ui.add_space(8.0);
                        if self.flight_progress >= 1.0 && self.file_progress >= 1.0 {
                            ui.label(RichText::new(icons::CHECK_CIRCLE).color(Color32::from_rgb(0x8e, 0xc0, 0x7c)));
                        }
                    });
                    let flight_pb = ProgressBar::new(self.flight_progress)
                        .desired_width(ui.available_width())
                        .show_percentage()
                        .animate(true);
                    ui.add(flight_pb);

                    ui.add_space(16.0);

                    // Cancel button
                    ui.horizontal(|ui| {
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.button(format!("{} Cancel", icons::X)).clicked() {
                                should_cancel = true;
                            }
                        });
                    });

                    ui.add_space(4.0);
                });
            });

        if should_cancel {
            self.cancel();
            return Some(None);
        }

        self.file.take()
    }
}

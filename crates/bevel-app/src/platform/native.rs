// SPDX-License-Identifier: GPL-3.0-or-later

use crate::app::BevelApp;
use bevel_ui::LogBuffer;
use tracing_subscriber::layer::SubscriberExt as _;
use tracing_subscriber::util::SubscriberInitExt as _;

/// Starts the desktop app and blocks until its window closes.
pub fn run() {
    if let Err(err) = try_run() {
        eprintln!("bevel: {err:#}");
        std::process::exit(1);
    }
}

fn try_run() -> anyhow::Result<()> {
    let log = LogBuffer::default();
    tracing_subscriber::registry()
        .with(tracing_subscriber::fmt::layer().with_writer(std::io::stderr))
        .with(log.layer())
        .with(super::log_filter())
        .try_init()?;

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Bevel")
            .with_app_id("bevel")
            .with_inner_size([1440.0, 900.0])
            .with_min_inner_size([800.0, 500.0]),
        renderer: eframe::Renderer::Wgpu,
        ..eframe::NativeOptions::default()
    };
    eframe::run_native(
        "Bevel",
        options,
        Box::new(|cc| Ok(Box::new(BevelApp::new(cc, log, true)))),
    )
    .map_err(|err| anyhow::anyhow!("failed to start the window: {err}"))
}

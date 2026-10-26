// SPDX-License-Identifier: GPL-3.0-or-later

use crate::app::BevelApp;
use bevel_ui::LogBuffer;
use eframe::wasm_bindgen::JsCast as _;
use eframe::web_sys;
use tracing_subscriber::layer::SubscriberExt as _;
use tracing_subscriber::util::SubscriberInitExt as _;

/// Id of the canvas element in `web/index.html`.
const CANVAS_ID: &str = "bevel_canvas";
/// Id of the element that shows loading progress and start-up errors.
const STATUS_ID: &str = "bevel_status";

/// Starts the app in the page's canvas. Returns immediately; the app runs
/// on the browser's event loop.
pub fn run() {
    let log = LogBuffer::default();
    // Without a terminal, the in-app log panel is the only log sink.
    let _ = tracing_subscriber::registry()
        .with(log.layer())
        .with(super::log_filter())
        .try_init();

    wasm_bindgen_futures::spawn_local(async move {
        let document = web_sys::window().and_then(|w| w.document());
        let Some(document) = document else {
            web_sys::console::error_1(&"bevel: no document to run in".into());
            return;
        };
        let canvas = document
            .get_element_by_id(CANVAS_ID)
            .and_then(|e| e.dyn_into::<web_sys::HtmlCanvasElement>().ok());
        let Some(canvas) = canvas else {
            show_status(&document, &format!("Missing <canvas id=\"{CANVAS_ID}\">."));
            return;
        };

        let result = eframe::WebRunner::new()
            .start(
                canvas,
                eframe::WebOptions::default(),
                Box::new(|cc| Ok(Box::new(BevelApp::new(cc, log, false)))),
            )
            .await;
        match result {
            Ok(()) => {
                if let Some(status) = document.get_element_by_id(STATUS_ID) {
                    status.remove();
                }
            }
            Err(err) => {
                web_sys::console::error_1(&err);
                show_status(
                    &document,
                    "Bevel could not start. Your browser may not support WebGPU or WebGL2.",
                );
            }
        }
    });
}

fn show_status(document: &web_sys::Document, message: &str) {
    if let Some(status) = document.get_element_by_id(STATUS_ID) {
        status.set_text_content(Some(message));
    }
}

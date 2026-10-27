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

/// Offers `bytes` as a browser download named `file_name`.
///
/// # Errors
///
/// A message if the browser refused to create the download.
pub fn save_file(
    file_name: &str,
    _extension: &str,
    _description: &str,
    bytes: &[u8],
) -> Result<super::SaveOutcome, String> {
    let fail = |what: &str| format!("could not start the download: {what}");
    let array = eframe::wasm_bindgen::JsValue::from(js_sys_bytes(bytes));
    let parts = eframe::web_sys::js_sys::Array::of1(&array);
    let options = web_sys::BlobPropertyBag::new();
    options.set_type("application/octet-stream");
    let blob = web_sys::Blob::new_with_u8_array_sequence_and_options(&parts, &options)
        .map_err(|_| fail("blob"))?;
    let url = web_sys::Url::create_object_url_with_blob(&blob).map_err(|_| fail("url"))?;
    let document = web_sys::window()
        .and_then(|w| w.document())
        .ok_or_else(|| fail("no document"))?;
    let anchor = document
        .create_element("a")
        .map_err(|_| fail("anchor"))?
        .dyn_into::<web_sys::HtmlAnchorElement>()
        .map_err(|_| fail("anchor"))?;
    anchor.set_href(&url);
    anchor.set_download(file_name);
    anchor.click();
    let _ = web_sys::Url::revoke_object_url(&url);
    Ok(super::SaveOutcome::Saved(file_name.to_owned()))
}

fn js_sys_bytes(bytes: &[u8]) -> eframe::web_sys::js_sys::Uint8Array {
    eframe::web_sys::js_sys::Uint8Array::from(bytes)
}

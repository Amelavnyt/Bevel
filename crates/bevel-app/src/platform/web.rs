// SPDX-License-Identifier: GPL-3.0-or-later

use crate::app::BevelApp;
use crate::rebuild::{self, Built};
use bevel_model::{Part, Regenerator};
use bevel_ui::LogBuffer;
use eframe::wasm_bindgen::JsCast as _;
use eframe::wasm_bindgen::prelude::Closure;
use eframe::web_sys::{self, js_sys};
use std::cell::RefCell;
use std::rc::Rc;
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

/// Offers the document as a download named after `location` (the name it
/// was opened or last saved under) or `name`.
///
/// # Errors
///
/// A message if the browser refused to create the download.
pub fn save_document(
    location: Option<&str>,
    name: &str,
    bytes: &[u8],
) -> Result<super::SaveOutcome, String> {
    save_file(location.unwrap_or(name), "bevel", "Bevel document", bytes)
}

/// Shows the browser's file picker and reads the chosen file.
pub fn open_file(extension: &str, _description: &str) -> super::PendingFile {
    let pending = super::PendingFile::default();
    let slot = pending.clone();
    let input = web_sys::window()
        .and_then(|w| w.document())
        .and_then(|d| d.create_element("input").ok())
        .and_then(|e| e.dyn_into::<web_sys::HtmlInputElement>().ok());
    let Some(input) = input else {
        *slot.0.borrow_mut() = Some(super::Opened::Failed(
            "the browser cannot show a file picker".to_owned(),
        ));
        return pending;
    };
    input.set_type("file");
    input.set_accept(&format!(".{extension}"));
    let picked = input.clone();
    let onchange = Closure::<dyn FnMut()>::new(move || {
        let Some(file) = picked.files().and_then(|files| files.get(0)) else {
            *slot.0.borrow_mut() = Some(super::Opened::Cancelled);
            return;
        };
        let slot = slot.clone();
        wasm_bindgen_futures::spawn_local(async move {
            let name = file.name();
            let opened = match wasm_bindgen_futures::JsFuture::from(file.array_buffer()).await {
                Ok(buffer) => super::Opened::File(name, js_sys::Uint8Array::new(&buffer).to_vec()),
                Err(_) => super::Opened::Failed(format!("could not read {name}")),
            };
            *slot.0.borrow_mut() = Some(opened);
        });
    });
    input.set_onchange(Some(onchange.as_ref().unchecked_ref()));
    onchange.forget();
    input.click();
    pending
}

/// Runs rebuilds in a Web Worker (the `bevel-worker` binary), so the page
/// stays responsive. Until the worker is ready, the newest request waits;
/// if the worker cannot start, rebuilds run on the page instead.
pub struct Rebuilder {
    worker: Option<web_sys::Worker>,
    shared: Rc<RefCell<WorkerState>>,
    fallback: Option<Regenerator>,
    _onmessage: Option<Closure<dyn FnMut(web_sys::MessageEvent)>>,
}

#[derive(Default)]
struct WorkerState {
    ready: bool,
    queued: Option<String>,
    result: Option<Built>,
}

impl Rebuilder {
    /// Starts the worker; `ctx` is repainted when a result arrives.
    pub fn new(ctx: &egui::Context) -> Self {
        let shared: Rc<RefCell<WorkerState>> = Rc::default();
        match start_worker() {
            Ok(worker) => {
                let onmessage = on_reply(Rc::clone(&shared), worker.clone(), ctx.clone());
                worker.set_onmessage(Some(onmessage.as_ref().unchecked_ref()));
                Self {
                    worker: Some(worker),
                    shared,
                    fallback: None,
                    _onmessage: Some(onmessage),
                }
            }
            Err(err) => {
                tracing::warn!(%err, "no rebuild worker; rebuilding on the page");
                Self {
                    worker: None,
                    shared,
                    fallback: Some(Regenerator::new()),
                    _onmessage: None,
                }
            }
        }
    }

    /// Asks for `part` to be rebuilt; the result carries `generation`.
    pub fn request(&mut self, generation: u64, part: Part) {
        if let Some(regenerator) = self.fallback.as_mut() {
            if let Ok(built) = rebuild::build(regenerator, &part, generation, None) {
                self.shared.borrow_mut().result = Some(built);
            }
            return;
        }
        let request = match crate::wire::request(generation, part) {
            Ok(r) => r,
            Err(err) => {
                tracing::error!(%err, "could not send a rebuild request");
                return;
            }
        };
        let mut state = self.shared.borrow_mut();
        match (&self.worker, state.ready) {
            (Some(worker), true) => {
                let _ = worker.post_message(&request.into());
            }
            _ => state.queued = Some(request),
        }
    }

    /// The newest finished rebuild, if any arrived since the last call.
    pub fn poll(&mut self) -> Option<Built> {
        self.shared.borrow_mut().result.take()
    }
}

impl Drop for Rebuilder {
    fn drop(&mut self) {
        if let Some(worker) = &self.worker {
            worker.terminate();
        }
    }
}

/// Handles the worker's messages: "ready", a reply, or "error:...".
fn on_reply(
    shared: Rc<RefCell<WorkerState>>,
    worker: web_sys::Worker,
    ctx: egui::Context,
) -> Closure<dyn FnMut(web_sys::MessageEvent)> {
    Closure::new(move |event: web_sys::MessageEvent| {
        let Some(text) = event.data().as_string() else {
            return;
        };
        let mut state = shared.borrow_mut();
        if text == "ready" {
            tracing::info!("rebuild worker ready");
            state.ready = true;
            if let Some(request) = state.queued.take() {
                let _ = worker.post_message(&request.into());
            }
        } else if let Some(err) = text.strip_prefix("error:") {
            tracing::error!(%err, "rebuild failed in the worker");
        } else {
            match crate::wire::read_reply(&text) {
                Ok(built) => state.result = Some(built),
                Err(err) => tracing::error!(%err, "unreadable rebuild reply"),
            }
        }
        ctx.request_repaint();
    })
}

/// Starts `bevel-worker` from a small script that loads its JavaScript glue
/// and WebAssembly, which Trunk builds next to the page.
fn start_worker() -> Result<web_sys::Worker, String> {
    let href = web_sys::window()
        .ok_or("no window")?
        .location()
        .href()
        .map_err(|_| "no location")?;
    let base = href.rsplit_once('/').map_or(href.as_str(), |(b, _)| b);
    let script = format!(
        "importScripts('{base}/bevel-worker.js'); wasm_bindgen('{base}/bevel-worker_bg.wasm');"
    );
    let parts = js_sys::Array::of1(&script.into());
    let options = web_sys::BlobPropertyBag::new();
    options.set_type("application/javascript");
    let blob = web_sys::Blob::new_with_str_sequence_and_options(&parts, &options)
        .map_err(|_| "could not make the worker script")?;
    let url = web_sys::Url::create_object_url_with_blob(&blob).map_err(|_| "no object URL")?;
    let worker = web_sys::Worker::new(&url).map_err(|_| "could not start the worker".to_owned());
    let _ = web_sys::Url::revoke_object_url(&url);
    worker
}

// SPDX-License-Identifier: GPL-3.0-or-later

//! The rebuild worker. In the browser it runs in a Web Worker: it answers
//! each rebuild request with the rebuilt bodies, keeping its regeneration
//! cache between requests, and collapses requests that queue up while it
//! is busy into the newest. On the desktop the binary reads one request
//! from stdin and writes the reply to stdout, which helps when debugging
//! the worker's messages.

use crate::{rebuild, wire};
use bevel_model::Regenerator;

/// Answers `request` with `regenerator`.
fn answer(regenerator: &mut Regenerator, request: &str) -> Result<String, String> {
    let (generation, part) = wire::read_request(request)?;
    let built = rebuild::build(regenerator, &part, generation, None).map_err(|e| e.to_string())?;
    wire::reply(&built)
}

#[cfg(not(target_arch = "wasm32"))]
pub fn run() {
    use std::io::{Read as _, Write as _};
    let mut request = String::new();
    if let Err(err) = std::io::stdin().read_to_string(&mut request) {
        eprintln!("bevel-worker: {err}");
        std::process::exit(2);
    }
    match answer(&mut Regenerator::new(), &request) {
        Ok(reply) => {
            let _ = std::io::stdout().write_all(reply.as_bytes());
        }
        Err(err) => {
            eprintln!("bevel-worker: {err}");
            std::process::exit(1);
        }
    }
}

#[cfg(target_arch = "wasm32")]
pub fn run() {
    use eframe::wasm_bindgen::JsCast as _;
    use eframe::wasm_bindgen::prelude::Closure;
    use eframe::web_sys::{self, js_sys};
    use std::cell::RefCell;
    use std::rc::Rc;

    let scope: web_sys::DedicatedWorkerGlobalScope = js_sys::global().unchecked_into();
    let regenerator = Rc::new(RefCell::new(Regenerator::new()));
    let latest: Rc<RefCell<Option<String>>> = Rc::default();

    // Builds the newest queued request; scheduled once per burst of messages.
    let work = {
        let scope = scope.clone();
        let latest = Rc::clone(&latest);
        Closure::<dyn FnMut()>::new(move || {
            let Some(request) = latest.borrow_mut().take() else {
                return;
            };
            let reply = answer(&mut regenerator.borrow_mut(), &request);
            let message = match reply {
                Ok(reply) => reply,
                Err(err) => format!("error:{err}"),
            };
            let _ = scope.post_message(&message.into());
        })
    };
    let onmessage = {
        let scope = scope.clone();
        Closure::<dyn FnMut(web_sys::MessageEvent)>::new(move |event: web_sys::MessageEvent| {
            let Some(request) = event.data().as_string() else {
                return;
            };
            let first = latest.borrow_mut().replace(request).is_none();
            if first {
                let _ = scope.set_timeout_with_callback_and_timeout_and_arguments_0(
                    work.as_ref().unchecked_ref(),
                    0,
                );
            }
        })
    };
    scope.set_onmessage(Some(onmessage.as_ref().unchecked_ref()));
    onmessage.forget();
    // Messages sent before this point were lost; tell the app to start.
    let _ = scope.post_message(&"ready".into());
}

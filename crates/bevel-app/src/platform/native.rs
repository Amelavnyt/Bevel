// SPDX-License-Identifier: GPL-3.0-or-later

use crate::app::BevelApp;
use crate::rebuild::{self, Built};
use bevel_model::{Part, Regenerator};
use bevel_ui::LogBuffer;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, mpsc};
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

/// Asks where to save `bytes` (suggesting `file_name`, filtered to
/// `extension`) and writes them there.
///
/// # Errors
///
/// A message describing why the file could not be written.
pub fn save_file(
    file_name: &str,
    extension: &str,
    description: &str,
    bytes: &[u8],
) -> Result<super::SaveOutcome, String> {
    let Some(path) = rfd::FileDialog::new()
        .set_file_name(file_name)
        .add_filter(description, &[extension])
        .save_file()
    else {
        return Ok(super::SaveOutcome::Cancelled);
    };
    std::fs::write(&path, bytes)
        .map_err(|err| format!("could not write {}: {err}", path.display()))?;
    Ok(super::SaveOutcome::Saved(path.display().to_string()))
}

/// Asks for a file to open and reads it.
pub fn open_file(extension: &str, description: &str) -> super::PendingFile {
    let Some(path) = rfd::FileDialog::new()
        .add_filter(description, &[extension])
        .pick_file()
    else {
        return super::PendingFile::done(super::Opened::Cancelled);
    };
    super::PendingFile::done(match std::fs::read(&path) {
        Ok(bytes) => super::Opened::File(path.display().to_string(), bytes),
        Err(err) => super::Opened::Failed(format!("could not read {}: {err}", path.display())),
    })
}

/// Writes a document to `location`, or asks where (suggesting `name`) when
/// it has none.
///
/// # Errors
///
/// A message describing why the file could not be written.
pub fn save_document(
    location: Option<&str>,
    name: &str,
    bytes: &[u8],
) -> Result<super::SaveOutcome, String> {
    match location {
        Some(path) => {
            std::fs::write(path, bytes).map_err(|err| format!("could not write {path}: {err}"))?;
            Ok(super::SaveOutcome::Saved(path.to_owned()))
        }
        None => save_file(name, "bevel", "Bevel document", bytes),
    }
}

/// Runs rebuilds on a background thread, newest request first: a new
/// request cancels the one in progress, and requests that arrive while
/// one runs collapse into the latest.
pub struct Rebuilder {
    shared: Arc<Shared>,
    results: mpsc::Receiver<Built>,
}

struct Shared {
    pending: Mutex<Pending>,
    wake: Condvar,
}

#[derive(Default)]
struct Pending {
    request: Option<(u64, Part)>,
    running: Option<Arc<AtomicBool>>,
    shutdown: bool,
}

impl Rebuilder {
    /// Starts the background thread; `ctx` is repainted when a result is
    /// ready.
    pub fn new(ctx: &egui::Context) -> Self {
        let shared = Arc::new(Shared {
            pending: Mutex::new(Pending::default()),
            wake: Condvar::new(),
        });
        let (send, results) = mpsc::channel();
        let worker = Arc::clone(&shared);
        let ctx = ctx.clone();
        let spawned = std::thread::Builder::new()
            .name("bevel-rebuild".to_owned())
            .spawn(move || rebuild_loop(&worker, &send, &ctx));
        if let Err(err) = spawned {
            tracing::error!(%err, "could not start the rebuild thread");
        }
        Self { shared, results }
    }

    /// Asks for `part` to be rebuilt; the result carries `generation`.
    pub fn request(&mut self, generation: u64, part: Part) {
        if let Ok(mut pending) = self.shared.pending.lock() {
            pending.request = Some((generation, part));
            if let Some(running) = &pending.running {
                running.store(true, Ordering::Relaxed);
            }
        }
        self.shared.wake.notify_one();
    }

    /// The newest finished rebuild, if any arrived since the last call.
    pub fn poll(&mut self) -> Option<Built> {
        self.results.try_iter().last()
    }
}

impl Drop for Rebuilder {
    fn drop(&mut self) {
        if let Ok(mut pending) = self.shared.pending.lock() {
            pending.shutdown = true;
            if let Some(running) = &pending.running {
                running.store(true, Ordering::Relaxed);
            }
        }
        self.shared.wake.notify_one();
    }
}

fn rebuild_loop(shared: &Shared, send: &mpsc::Sender<Built>, ctx: &egui::Context) {
    let mut regenerator = Regenerator::new();
    loop {
        let (generation, part, cancel) = {
            let Ok(mut pending) = shared.pending.lock() else {
                return;
            };
            loop {
                if pending.shutdown {
                    return;
                }
                if let Some((generation, part)) = pending.request.take() {
                    let cancel = Arc::new(AtomicBool::new(false));
                    pending.running = Some(Arc::clone(&cancel));
                    break (generation, part, cancel);
                }
                pending = match shared.wake.wait(pending) {
                    Ok(p) => p,
                    Err(_) => return,
                };
            }
        };
        if let Ok(built) = rebuild::build(&mut regenerator, &part, generation, Some(&cancel)) {
            if send.send(built).is_err() {
                return;
            }
            ctx.request_repaint();
        }
    }
}

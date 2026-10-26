// SPDX-License-Identifier: GPL-3.0-or-later

//! In-app log: a `tracing` layer that records events into a ring buffer,
//! and the panel that shows them.

use std::collections::VecDeque;
use std::fmt::Write as _;
use std::sync::{Arc, Mutex, PoisonError};
use tracing::field::{Field, Visit};
use tracing::{Event, Level, Subscriber};
use tracing_subscriber::layer::Context;

/// Number of lines kept; older lines are dropped.
const CAPACITY: usize = 2000;

/// One recorded log event.
#[derive(Clone, Debug)]
pub struct LogLine {
    /// Severity.
    pub level: Level,
    /// Module path of the code that logged it.
    pub target: String,
    /// The message followed by any other fields, as `key=value`.
    pub message: String,
}

/// Shared ring buffer of recent log lines.
#[derive(Clone, Default)]
pub struct LogBuffer {
    lines: Arc<Mutex<VecDeque<LogLine>>>,
}

impl LogBuffer {
    /// A `tracing` layer that appends every event to this buffer.
    pub fn layer(&self) -> LogLayer {
        LogLayer {
            buffer: self.clone(),
        }
    }

    /// A copy of the buffered lines, oldest first.
    pub fn lines(&self) -> Vec<LogLine> {
        self.lock().iter().cloned().collect()
    }

    fn push(&self, line: LogLine) {
        let mut lines = self.lock();
        if lines.len() == CAPACITY {
            lines.pop_front();
        }
        lines.push_back(line);
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, VecDeque<LogLine>> {
        // A panic while holding the lock can't leave a VecDeque of plain
        // lines in a bad state, so a poisoned lock is still usable.
        self.lines.lock().unwrap_or_else(PoisonError::into_inner)
    }
}

/// `tracing` layer feeding a [`LogBuffer`].
pub struct LogLayer {
    buffer: LogBuffer,
}

impl<S: Subscriber> tracing_subscriber::Layer<S> for LogLayer {
    fn on_event(&self, event: &Event<'_>, _ctx: Context<'_, S>) {
        let mut visitor = MessageVisitor::default();
        event.record(&mut visitor);
        self.buffer.push(LogLine {
            level: *event.metadata().level(),
            target: event.metadata().target().to_owned(),
            message: visitor.finish(),
        });
    }
}

#[derive(Default)]
struct MessageVisitor {
    message: String,
    fields: String,
}

impl MessageVisitor {
    fn finish(self) -> String {
        if self.fields.is_empty() {
            self.message
        } else if self.message.is_empty() {
            self.fields
        } else {
            format!("{} {}", self.message, self.fields)
        }
    }
}

impl Visit for MessageVisitor {
    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            let _ = write!(self.message, "{value:?}");
        } else {
            if !self.fields.is_empty() {
                self.fields.push(' ');
            }
            let _ = write!(self.fields, "{}={value:?}", field.name());
        }
    }
}

/// Shows the buffered lines, newest at the bottom.
pub(crate) fn show(ui: &mut egui::Ui, log: &LogBuffer) {
    ui.horizontal(|ui| {
        ui.strong("Log");
    });
    ui.separator();
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .stick_to_bottom(true)
        .show(ui, |ui| {
            for line in log.lines() {
                let color = match line.level {
                    Level::ERROR => ui.visuals().error_fg_color,
                    Level::WARN => ui.visuals().warn_fg_color,
                    Level::INFO => ui.visuals().text_color(),
                    Level::DEBUG | Level::TRACE => ui.visuals().weak_text_color(),
                };
                ui.label(
                    egui::RichText::new(format!(
                        "{:>5} {}: {}",
                        line.level, line.target, line.message
                    ))
                    .monospace()
                    .color(color),
                );
            }
        });
}

#[cfg(test)]
mod tests {
    use super::*;
    use tracing_subscriber::layer::SubscriberExt as _;

    #[test]
    fn layer_records_message_and_fields() {
        let buffer = LogBuffer::default();
        let subscriber = tracing_subscriber::registry().with(buffer.layer());
        tracing::subscriber::with_default(subscriber, || {
            tracing::warn!(width = 3, "box too small");
        });
        let lines = buffer.lines();
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].level, Level::WARN);
        assert_eq!(lines[0].message, "box too small width=3");
    }

    #[test]
    fn buffer_drops_oldest_lines_at_capacity() {
        let buffer = LogBuffer::default();
        for i in 0..CAPACITY + 5 {
            buffer.push(LogLine {
                level: Level::INFO,
                target: String::new(),
                message: i.to_string(),
            });
        }
        let lines = buffer.lines();
        assert_eq!(lines.len(), CAPACITY);
        assert_eq!(lines[0].message, "5");
    }
}

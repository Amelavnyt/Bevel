// SPDX-License-Identifier: GPL-3.0-or-later

//! Reading ids back from the id buffer.
//!
//! The id pass renders into an `R32Uint` texture. To pick, one texel of it
//! is copied into a small mappable buffer after the frame that contains the
//! pass, and the buffer is mapped asynchronously. The application polls for
//! the result without blocking: on native targets [`Picker::poll`] drives
//! the device's callbacks, on the web the browser fires them on its own.

use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};

/// The outcome of a pick request, see
/// [`ViewportRenderer::poll_pick`](crate::ViewportRenderer::poll_pick).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PickResult {
    /// The pixel that was requested, in viewport pixels from the top left.
    /// If it lay outside the viewport the id of the nearest edge pixel was
    /// read.
    pub pixel: [u32; 2],
    /// The id under the pixel, or `None` if nothing pickable is there.
    pub id: Option<u32>,
}

/// Size of the readback buffer: one row of `COPY_BYTES_PER_ROW_ALIGNMENT`
/// bytes, which holds the single 4-byte texel with padding.
pub(crate) const READBACK_BYTES: u64 = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT as u64;

/// Bytes per texel of the id texture (`R32Uint`).
const TEXEL_BYTES: u32 = 4;

/// The pixel moved into a `size` target, per axis.
pub(crate) fn clamp_pixel(pixel: [u32; 2], size: [u32; 2]) -> [u32; 2] {
    [0, 1].map(|axis| pixel[axis].min(size[axis].saturating_sub(1)))
}

/// `bytes` rounded up to the row alignment texture-to-buffer copies need.
pub(crate) fn aligned_bytes_per_row(bytes: u32) -> u32 {
    bytes.div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT) * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT
}

/// The row pitch of the one-texel copy.
pub(crate) fn readback_bytes_per_row() -> u32 {
    aligned_bytes_per_row(TEXEL_BYTES)
}

/// The id stored in the first four bytes of the readback buffer, little
/// endian. 0 means nothing pickable.
pub(crate) fn decode_id(bytes: &[u8]) -> Option<u32> {
    let texel = <[u8; 4]>::try_from(bytes.get(..4)?).ok()?;
    Some(u32::from_le_bytes(texel))
}

/// Which pick requests are waiting and which one the GPU is working on.
///
/// Only one pick is in flight at a time. A request made while another one
/// is still waiting for the next frame replaces it, and one made while a
/// readback is in flight waits until that readback has been collected.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct PickQueue {
    requested: Option<[u32; 2]>,
    in_flight: Option<[u32; 2]>,
}

impl PickQueue {
    /// Asks for `pixel`, replacing any request that has not started yet.
    pub(crate) fn request(&mut self, pixel: [u32; 2]) {
        self.requested = Some(pixel);
    }

    /// Whether the next frame should render the id pass.
    pub(crate) fn wants_pass(&self) -> bool {
        self.requested.is_some() && self.in_flight.is_none()
    }

    /// Starts the waiting request, if a readback can start now.
    pub(crate) fn start(&mut self) -> Option<[u32; 2]> {
        if !self.wants_pass() {
            return None;
        }
        self.in_flight = self.requested.take();
        self.in_flight
    }

    /// The pixel of the readback in flight.
    pub(crate) fn in_flight(&self) -> Option<[u32; 2]> {
        self.in_flight
    }

    /// Marks the readback in flight as collected (or lost).
    pub(crate) fn finish(&mut self) -> Option<[u32; 2]> {
        self.in_flight.take()
    }
}

/// How far the map request of a readback has got.
const MAP_PENDING: u8 = 0;
const MAP_DONE: u8 = 1;
const MAP_FAILED: u8 = 2;

/// The readback buffer and the state of the pick in flight.
pub(crate) struct Picker {
    device: wgpu::Device,
    buffer: wgpu::Buffer,
    queue: PickQueue,
    /// Set by the map callback of the readback in flight.
    mapped: Arc<AtomicU8>,
}

impl Picker {
    pub(crate) fn new(device: &wgpu::Device) -> Self {
        Self {
            device: device.clone(),
            buffer: create_buffer(device),
            queue: PickQueue::default(),
            mapped: Arc::new(AtomicU8::new(MAP_PENDING)),
        }
    }

    pub(crate) fn request(&mut self, pixel: [u32; 2]) {
        self.queue.request(pixel);
    }

    pub(crate) fn wants_pass(&self) -> bool {
        self.queue.wants_pass()
    }

    /// Starts the waiting request and records the copy of its texel from
    /// `texture` (the id texture of size `size`) into the readback buffer.
    /// Call [`Picker::map`] after submitting the encoder. Returns whether a
    /// copy was recorded.
    pub(crate) fn copy_pixel(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        texture: &wgpu::Texture,
        size: [u32; 2],
    ) -> bool {
        let Some(pixel) = self.queue.start() else {
            return false;
        };
        let [x, y] = clamp_pixel(pixel, size);
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture,
                mip_level: 0,
                origin: wgpu::Origin3d { x, y, z: 0 },
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &self.buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(readback_bytes_per_row()),
                    rows_per_image: None,
                },
            },
            wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1,
            },
        );
        true
    }

    /// Maps the readback buffer; call after the copy has been submitted.
    pub(crate) fn map(&mut self) {
        let mapped = Arc::new(AtomicU8::new(MAP_PENDING));
        self.mapped = Arc::clone(&mapped);
        self.buffer
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |result| {
                let state = if result.is_ok() { MAP_DONE } else { MAP_FAILED };
                mapped.store(state, Ordering::Release);
            });
    }

    /// The result of the pick in flight, once the buffer is mapped. Never
    /// blocks.
    pub(crate) fn poll(&mut self) -> Option<PickResult> {
        let pixel = self.queue.in_flight()?;
        // Runs the map callback once the copy has finished. A no-op on the
        // web, where the callback fires from the event loop.
        if let Err(error) = self.device.poll(wgpu::PollType::Poll) {
            tracing::warn!(%error, "polling the device for a pick failed");
        }
        match self.mapped.load(Ordering::Acquire) {
            MAP_DONE => {
                let id = {
                    let view = self.buffer.slice(..).get_mapped_range();
                    view.ok().and_then(|bytes| decode_id(&bytes))
                };
                self.buffer.unmap();
                self.queue.finish();
                Some(PickResult {
                    pixel,
                    id: id.filter(|&id| id != 0),
                })
            }
            MAP_FAILED => {
                tracing::warn!("mapping the pick buffer failed; dropping the pick");
                // The buffer is not mapped after a failure, so it can be reused.
                self.queue.finish();
                None
            }
            _ => None,
        }
    }
}

fn create_buffer(device: &wgpu::Device) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("viewport pick readback"),
        size: READBACK_BYTES,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pixels_are_clamped_into_the_target() {
        assert_eq!(clamp_pixel([10, 20], [100, 50]), [10, 20]);
        assert_eq!(clamp_pixel([100, 50], [100, 50]), [99, 49]);
        assert_eq!(clamp_pixel([u32::MAX, 3], [640, 480]), [639, 3]);
        assert_eq!(clamp_pixel([5, 5], [1, 1]), [0, 0]);
        assert_eq!(clamp_pixel([5, 5], [0, 0]), [0, 0]);
    }

    #[test]
    fn rows_are_aligned_to_256_bytes() {
        assert_eq!(aligned_bytes_per_row(0), 0);
        assert_eq!(aligned_bytes_per_row(4), 256);
        assert_eq!(aligned_bytes_per_row(256), 256);
        assert_eq!(aligned_bytes_per_row(257), 512);
        assert_eq!(aligned_bytes_per_row(500 * 4), 2048);
        assert_eq!(readback_bytes_per_row(), 256);
        // One aligned row holds the texel, and the buffer is exactly that.
        assert_eq!(READBACK_BYTES, u64::from(readback_bytes_per_row()));
    }

    #[test]
    fn ids_decode_from_little_endian_bytes() {
        assert_eq!(decode_id(&[1, 0, 0, 0, 9, 9]), Some(1));
        assert_eq!(decode_id(&[0x78, 0x56, 0x34, 0x12]), Some(0x1234_5678));
        assert_eq!(decode_id(&[0; 256]), Some(0));
        assert_eq!(decode_id(&[1, 2, 3]), None);
    }

    #[test]
    fn newer_request_replaces_an_older_waiting_one() {
        let mut queue = PickQueue::default();
        assert!(!queue.wants_pass());
        queue.request([1, 1]);
        queue.request([2, 2]);
        assert!(queue.wants_pass());
        assert_eq!(queue.start(), Some([2, 2]));
        assert_eq!(queue.start(), None);
    }

    #[test]
    fn only_one_pick_is_in_flight() {
        let mut queue = PickQueue::default();
        queue.request([1, 1]);
        assert_eq!(queue.start(), Some([1, 1]));
        assert_eq!(queue.in_flight(), Some([1, 1]));
        // A request during the readback waits: no pass is rendered for it.
        queue.request([5, 6]);
        assert!(!queue.wants_pass());
        assert_eq!(queue.start(), None);
        assert_eq!(queue.in_flight(), Some([1, 1]));
        // Once collected, the waiting request starts.
        assert_eq!(queue.finish(), Some([1, 1]));
        assert!(queue.wants_pass());
        assert_eq!(queue.start(), Some([5, 6]));
        assert_eq!(queue.finish(), Some([5, 6]));
        assert_eq!(queue.finish(), None);
        assert!(!queue.wants_pass());
    }
}

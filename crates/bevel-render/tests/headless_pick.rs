// SPDX-License-Identifier: GPL-3.0-or-later

//! Renders a tiny scene on a real adapter and reads ids back from it.
//!
//! These tests need a GPU (or a software adapter such as WARP or lavapipe),
//! which CI doesn't have, so they are ignored by default. Run them locally
//! with:
//!
//! ```sh
//! cargo test -p bevel-render --test headless_pick -- --ignored --nocapture
//! ```
//!
//! They ask for a fallback (software) adapter first and use any adapter
//! otherwise, and fail if the machine has none.

// Test helpers outside `#[test]` functions panic on setup failures.
#![allow(clippy::panic, clippy::expect_used)]

use bevel_math::Point3;
use bevel_render::ViewportRenderer;
use bevel_render::{Camera, LineSet, PickResult, Projection, StandardView, TriangleMesh};
use std::future::Future;
use std::task::{Context, Poll, Waker};
use std::time::Duration;

const SIZE: u32 = 64;
const FACE: u32 = 7;
const EDGE: u32 = 9;

/// Runs a future to completion; the wgpu setup futures resolve without a
/// real executor on native targets.
fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = std::pin::pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    loop {
        if let Poll::Ready(value) = future.as_mut().poll(&mut cx) {
            return value;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
}

/// The first adapter (software ones first) that gives a working device.
fn device() -> (wgpu::Adapter, wgpu::Device, wgpu::Queue) {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
    for force_fallback_adapter in [true, false] {
        let options = wgpu::RequestAdapterOptions {
            force_fallback_adapter,
            ..wgpu::RequestAdapterOptions::default()
        };
        let Ok(adapter) = block_on(instance.request_adapter(&options)) else {
            continue;
        };
        let descriptor = wgpu::DeviceDescriptor {
            required_limits: adapter.limits(),
            ..wgpu::DeviceDescriptor::default()
        };
        match block_on(adapter.request_device(&descriptor)) {
            Ok((device, queue)) => {
                println!("using adapter {:?}", adapter.get_info());
                return (adapter, device, queue);
            }
            Err(error) => println!("skipping adapter {:?}: {error}", adapter.get_info().name),
        }
    }
    panic!("no usable wgpu adapter available");
}

/// A square face of id `FACE` around the origin, seen from above, with an
/// edge of id `EDGE` along the X axis through its middle.
fn scene() -> (TriangleMesh, LineSet) {
    let h = 20.0;
    let face = TriangleMesh {
        positions: vec![[-h, -h, 0.0], [h, -h, 0.0], [h, h, 0.0], [-h, h, 0.0]],
        normals: vec![[0.0, 0.0, 1.0]; 4],
        indices: vec![0, 1, 2, 0, 2, 3],
        color: [0.5, 0.5, 0.5, 1.0],
        ids: vec![FACE; 4],
    };
    let edge = LineSet {
        positions: vec![[-h, 0.0, 0.0], [h, 0.0, 0.0]],
        ids: vec![EDGE],
        color: [0.0, 0.0, 0.0, 1.0],
        ..LineSet::default()
    };
    (face, edge)
}

fn top_camera() -> Camera {
    let mut camera = Camera {
        target: Point3::ZERO,
        distance: 150.0,
        projection: Projection::Orthographic,
        ..Camera::default()
    };
    camera.set_view(StandardView::Top);
    camera
}

fn pick(
    renderer: &mut ViewportRenderer,
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    pixel: [u32; 2],
) -> PickResult {
    renderer.request_pick(pixel);
    renderer.render(device, queue, [SIZE, SIZE], &top_camera());
    for _ in 0..1000 {
        if let Some(result) = renderer.poll_pick() {
            return result;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    panic!("pick of {pixel:?} did not complete");
}

#[test]
#[ignore = "needs a wgpu adapter; run with --ignored"]
fn picks_faces_edges_and_background() {
    let (adapter, device, queue) = device();
    let mut renderer = ViewportRenderer::new(&device, &adapter);
    let (face, edge) = scene();
    renderer.set_scene(&device, &[face], &[edge]);

    // Orthographic top view, 150 mm distance: about 1.7 mm per pixel, so the
    // 40 mm square spans pixels 20 to 44 and its middle edge is 7.5 px wide
    // in the id pass.
    let center = SIZE / 2;
    assert_eq!(
        pick(&mut renderer, &device, &queue, [center, center]),
        PickResult {
            pixel: [center, center],
            id: Some(EDGE)
        }
    );
    assert_eq!(
        pick(&mut renderer, &device, &queue, [center, 24]).id,
        Some(FACE)
    );
    assert_eq!(pick(&mut renderer, &device, &queue, [2, 2]).id, None);
    // Outside the viewport reads the nearest edge pixel (the background).
    let outside = pick(&mut renderer, &device, &queue, [500, 500]);
    assert_eq!(outside.pixel, [500, 500]);
    assert_eq!(outside.id, None);

    // A newer request replaces one that was not rendered yet.
    renderer.request_pick([center, center]);
    renderer.request_pick([center, 24]);
    renderer.render(&device, &queue, [SIZE, SIZE], &top_camera());
    let result = loop {
        if let Some(result) = renderer.poll_pick() {
            break result;
        }
        std::thread::sleep(Duration::from_millis(2));
    };
    assert_eq!(result.pixel, [center, 24]);
    assert_eq!(result.id, Some(FACE));
    assert!(renderer.poll_pick().is_none());
}

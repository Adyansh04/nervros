//! Keep-out zones: a translucent red floor over each, outlined and named.

use std::collections::BTreeSet;
use std::sync::Arc;

use nervros_core::zones::{Zone, Zones};
use rerun::RecordingStream;
use tokio::time::{MissedTickBehavior, interval};

use crate::spawner::Layer;
use crate::{OBJECTS_PERIOD, f32s, layers, put_static};

const ZONE_RED: [u8; 3] = [230, 60, 60];
/// Enough to tint the map under a zone without hiding its walls.
const FILL_ALPHA: u8 = 90;
/// Over the map, the coverage and the rooms' outlines.
const ZONE_Z: f32 = 0.03;

/// Draws the zones whenever they change; hidden, they are cleared until shown again.
pub(super) async fn zones(rec: RecordingStream, zones: Arc<Zones>, layer: Layer) {
    let changes = zones.changes();
    let mut tick = interval(OBJECTS_PERIOD);
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let (mut last, mut drawn) = (None, BTreeSet::new());
    loop {
        tick.tick().await;
        if !layer.shown() {
            if last.take().is_some() {
                layer.clear(&rec);
                drawn.clear();
            }
            continue;
        }
        let version = *changes.borrow();
        if last != Some(version) {
            draw_zones(&rec, &layer.path, &zones.all(), &mut drawn);
            last = Some(version);
        }
    }
}

/// One entity per zone, so a click names it; zones removed since the last draw are cleared.
fn draw_zones(rec: &RecordingStream, path: &str, zones: &[Zone], drawn: &mut BTreeSet<String>) {
    let [r, g, b] = ZONE_RED;
    let mut now = BTreeSet::new();
    for zone in zones {
        let corners: Vec<[f32; 3]> = zone
            .polygon
            .iter()
            .map(|&corner| {
                let xy = f32s(corner);
                [xy[0], xy[1], ZONE_Z]
            })
            .collect();
        let mut outline = corners.clone();
        outline.extend(corners.first().copied());
        let entity = format!("{path}/{}", layers::slug(&zone.name));
        let set_in = if zone.fixed { ", from the profile" } else { "" };
        put_static(
            rec,
            &entity,
            &rerun::LineStrips3D::new([outline])
                .with_labels([format!("{}: keep out{set_in}", zone.name)])
                .with_colors([rerun::Color::from_rgb(r, g, b)])
                .with_radii([0.04]),
        );
        put_static(
            rec,
            &format!("{entity}/fill"),
            &rerun::Mesh3D::new(corners)
                .with_triangle_indices(triangles(&zone.polygon))
                .with_albedo_factor(rerun::Rgba32::from_unmultiplied_rgba(r, g, b, FILL_ALPHA)),
        );
        now.insert(entity);
    }
    for gone in drawn.difference(&now) {
        put_static(rec, gone, &rerun::Clear::recursive());
    }
    *drawn = now;
}

/// Twice the signed area of the triangle `a b c`: positive when it turns left.
fn turn(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// The polygon cut into triangles by ear clipping, since a room's outline can be concave.
pub(super) fn triangles(polygon: &[[f64; 2]]) -> Vec<[u32; 3]> {
    let count = polygon.len();
    let winding = (0..count)
        .map(|at| {
            let ([x1, y1], [x2, y2]) = (polygon[at], polygon[(at + 1) % count]);
            x1 * y2 - x2 * y1
        })
        .sum::<f64>()
        .signum();
    // Positive when three corners turn the polygon's own way, whichever way it was drawn.
    let inward =
        |[from, via, to]: [usize; 3]| winding * turn(polygon[from], polygon[via], polygon[to]);
    let mut left: Vec<usize> = (0..count).collect();
    let mut out = Vec::with_capacity(count.saturating_sub(2));
    while left.len() >= 3 {
        let len = left.len();
        let around = |at: usize| [left[(at + len - 1) % len], left[at], left[(at + 1) % len]];
        let ear = (0..len).find(|&at| {
            let [prev, corner, next] = around(at);
            inward([prev, corner, next]) > 0.0
                && !left.iter().any(|&other| {
                    ![prev, corner, next].contains(&other)
                        && inward([prev, corner, other]) >= 0.0
                        && inward([corner, next, other]) >= 0.0
                        && inward([next, prev, other]) >= 0.0
                })
        });
        // No ear only when the outline crosses itself; what was cut so far still shows.
        let Some(ear) = ear else { break };
        #[expect(
            clippy::cast_possible_truncation,
            reason = "a zone has a handful of corners"
        )]
        out.push(around(ear).map(|corner| corner as u32));
        left.remove(ear);
    }
    out
}

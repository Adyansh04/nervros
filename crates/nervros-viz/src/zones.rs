//! Zones on the floor: keep-out zones outlined and hatched in red, slow zones in amber with their
//! speed, each named at a corner.

use std::collections::BTreeSet;
use std::sync::Arc;

use nervros_core::zones::{Rule, Zone, Zones};
use rerun::RecordingStream;
use tokio::time::{MissedTickBehavior, interval};

use crate::spawner::Layer;
use crate::{OBJECTS_PERIOD, f32s, layers, put_static};

const ZONE_RED: [u8; 3] = [230, 60, 60];
/// The window's warning amber: a slow zone is a caution, not a wall.
const ZONE_AMBER: [u8; 3] = [243, 166, 43];
/// Between the hatch's lines, in metres: lines rather than a fill, so the map shows through, and
/// no transparency for the viewer to sort against the map's own layers. A slow zone's are sparser,
/// so the two read apart at a glance.
const HATCH_M: f64 = 0.25;
const SLOW_HATCH_M: f64 = 0.5;
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

fn on_floor(corner: [f64; 2]) -> [f32; 3] {
    let xy = f32s(corner);
    [xy[0], xy[1], ZONE_Z]
}

/// One entity per zone, so a click names it; zones removed since the last draw are cleared.
fn draw_zones(rec: &RecordingStream, path: &str, zones: &[Zone], drawn: &mut BTreeSet<String>) {
    let mut now = BTreeSet::new();
    for zone in zones {
        let ([r, g, b], spacing, says) = match zone.rule {
            Rule::KeepOut => (ZONE_RED, HATCH_M, "keep out".to_owned()),
            Rule::Slow(limit) => (ZONE_AMBER, SLOW_HATCH_M, format!("slow, {}", limit.words())),
        };
        let colour = rerun::Color::from_rgb(r, g, b);
        let mut outline: Vec<[f32; 3]> = zone.polygon.iter().copied().map(on_floor).collect();
        outline.extend(outline.first().copied());
        // Names that differ only in what a slug drops would share an entity, and one go undrawn.
        let mut entity = format!("{path}/{}", layers::slug(&zone.name));
        while now.contains(&entity) {
            entity.push('_');
        }
        put_static(
            rec,
            &entity,
            &rerun::LineStrips3D::new([outline])
                .with_colors([colour])
                .with_radii([0.04]),
        );
        let strokes: Vec<[[f32; 3]; 2]> = hatch(&zone.polygon, spacing)
            .into_iter()
            .map(|stroke| stroke.map(on_floor))
            .collect();
        put_static(
            rec,
            &format!("{entity}/hatch"),
            &rerun::LineStrips3D::new(strokes)
                .with_colors([colour])
                .with_radii([0.012]),
        );
        // At its northernmost corner: a room's own name sits at its middle.
        if let Some(&corner) = zone
            .polygon
            .iter()
            .max_by(|one, other| one[1].total_cmp(&other[1]))
        {
            let set_in = if zone.fixed { ", from the profile" } else { "" };
            put_static(
                rec,
                &format!("{entity}/name"),
                &rerun::Points3D::new([on_floor(corner)])
                    .with_labels([format!("{}: {says}{set_in}", zone.name)])
                    .with_colors([colour])
                    .with_radii([0.06]),
            );
        }
        now.insert(entity);
    }
    for gone in drawn.difference(&now) {
        put_static(rec, gone, &rerun::Clear::recursive());
    }
    *drawn = now;
}

/// Diagonal lines `spacing` metres apart across the polygon, clipped to it: the crossings of each
/// line with the outline, paired in order along it, bound the stretches inside, concave or not.
pub(super) fn hatch(polygon: &[[f64; 2]], spacing: f64) -> Vec<[[f64; 2]; 2]> {
    // In these coordinates the lines are `across` = constant, and `along` runs down each one.
    let turned: Vec<(f64, f64)> = polygon.iter().map(|&[px, py]| (px + py, px - py)).collect();
    let (first, last) = turned.iter().fold(
        (f64::INFINITY, f64::NEG_INFINITY),
        |(lo, hi), &(across, _)| (lo.min(across), hi.max(across)),
    );
    let step = spacing * std::f64::consts::SQRT_2;
    let back =
        |across: f64, along: f64| [f64::midpoint(across, along), f64::midpoint(across, -along)];
    let mut strokes = Vec::new();
    let mut line = first + step / 2.0;
    while step > 0.0 && line < last {
        let mut crossings: Vec<f64> = (0..turned.len())
            .filter_map(|index| {
                let ((u1, v1), (u2, v2)) = (turned[index], turned[(index + 1) % turned.len()]);
                ((u1 <= line) != (u2 <= line)).then(|| v1 + (line - u1) * (v2 - v1) / (u2 - u1))
            })
            .collect();
        crossings.sort_by(f64::total_cmp);
        strokes.extend(
            crossings
                .chunks_exact(2)
                .map(|pair| [back(line, pair[0]), back(line, pair[1])]),
        );
        line += step;
    }
    strokes
}

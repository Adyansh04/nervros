//! Rooms: their outlines, names and areas.

use std::collections::BTreeSet;

use rerun::RecordingStream;
use serde_json::Value;

use crate::grid::{seen, seen_colour};
use crate::{put, put_static, xyz};

/// The area of a polygon, by the shoelace formula.
pub(super) fn area(points: &[[f32; 3]]) -> f64 {
    let n = points.len();
    let twice: f64 = (0..n)
        .map(|i| {
            let (a, b) = (points[i], points[(i + 1) % n]);
            f64::from(a[0]) * f64::from(b[1]) - f64::from(b[0]) * f64::from(a[1])
        })
        .sum();
    twice.abs() / 2.0
}

/// Room outlines from a `canopy_msgs/msg/RoomArray`, one entity each so a click in the viewer
/// names the room, labelled and coloured by how much the camera has seen of it; rooms that went
/// away are cleared. The share seen overall, by area, goes on the "Exploring" plot.
pub(super) fn draw_rooms(
    rec: &RecordingStream,
    path: &str,
    msg: &Value,
    drawn: &mut BTreeSet<String>,
) {
    let list = msg["rooms"].as_array().map_or(&[][..], Vec::as_slice);
    let mut now = BTreeSet::new();
    let (mut seen_area, mut total_area) = (0.0, 0.0);
    for room in list {
        let (Some(id), Some(points)) = (
            room["id"].as_str().filter(|i| !i.is_empty()),
            room["outline"]["points"].as_array(),
        ) else {
            continue;
        };
        let mut strip: Vec<[f32; 3]> = points
            .iter()
            .map(|p| [xyz(p)[0], xyz(p)[1], 0.02])
            .collect();
        let room_seen = seen(room);
        if let Some(fraction) = room_seen {
            let a = area(&strip);
            seen_area += fraction * a;
            total_area += a;
        }
        if let Some(first) = strip.first().copied() {
            strip.push(first);
        }
        // canopy's names are "room C" until someone names the room; its type says more.
        let name = room["type"]
            .as_str()
            .filter(|t| !t.is_empty())
            .or_else(|| room["name"].as_str())
            .unwrap_or_default();
        let label = match room_seen {
            Some(f) => format!("{id} {name} {:.0}%", f * 100.0),
            None => format!("{id} {name}"),
        };
        put_static(
            rec,
            &format!("{path}/{id}"),
            &rerun::LineStrips3D::new([strip])
                .with_labels([label])
                .with_colors([room_seen.map_or(rerun::Color::from_rgb(200, 200, 210), seen_colour)])
                .with_radii([0.03]),
        );
        now.insert(id.to_owned());
    }
    for gone in drawn.difference(&now) {
        put_static(rec, &format!("{path}/{gone}"), &rerun::Clear::flat());
    }
    #[expect(clippy::cast_precision_loss, reason = "a handful of rooms")]
    put(
        rec,
        "mapping/rooms",
        &rerun::Scalars::single(now.len() as f64),
    );
    if total_area > 0.0 {
        put(
            rec,
            "mapping/seen %",
            &rerun::Scalars::single(100.0 * seen_area / total_area),
        );
    }
    *drawn = now;
}

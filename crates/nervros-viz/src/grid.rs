//! Occupancy grids drawn as images on the floor: the map, and what the camera has covered.

use rerun::RecordingStream;
use serde_json::Value;

use crate::{SEEN, TO_SEE, WRITTEN_OFF, f32s, num, put_static};

/// An occupancy grid as image bytes, top row first as images are, with `N` bytes per cell.
pub(super) struct Grid {
    pub(super) bytes: Vec<u8>,
    pub(super) size: [u32; 2],
    pub(super) cell_m: f32,
    /// The lower-left corner in the map frame.
    pub(super) corner: [f32; 3],
}

/// A `nav_msgs/msg/OccupancyGrid` as a [`Grid`], each cell's value mapped by `cell`.
// ponytail: ignores the origin's rotation, which map servers leave at zero; rotate the grid if a
// robot publishes a rotated one.
pub(super) fn grid<const N: usize>(msg: &Value, cell: impl Fn(i64) -> [u8; N]) -> Option<Grid> {
    let info = &msg["info"];
    let res = num(&info["resolution"]);
    let width = usize::try_from(info["width"].as_u64()?).ok()?;
    let height = usize::try_from(info["height"].as_u64()?).ok()?;
    let data = msg["data"].as_array()?;
    if res <= 0.0 || width == 0 || data.len() != width * height {
        return None;
    }
    let mut bytes = Vec::with_capacity(data.len() * N);
    // ROS rows start at the origin, the bottom of the map; image rows start at the top.
    for row in data.chunks(width).rev() {
        for v in row {
            bytes.extend_from_slice(&cell(v.as_i64().unwrap_or(-1)));
        }
    }
    let origin = &info["origin"]["position"];
    let [cell_m, x, y] = f32s([res, num(&origin["x"]), num(&origin["y"])]);
    Some(Grid {
        bytes,
        size: [u32::try_from(width).ok()?, u32::try_from(height).ok()?],
        cell_m,
        corner: [x, y, 0.0],
    })
}

pub(super) fn put_grid(
    rec: &RecordingStream,
    path: &str,
    grid: Grid,
    model: rerun::ColorModel,
    lift: f32,
) {
    let format = rerun::components::ImageFormat::from_color_model(
        grid.size,
        model,
        rerun::ChannelDatatype::U8,
    );
    let [x, y, _] = grid.corner;
    // Below full opacity the viewer blends a grid, its clear cells included; at full opacity it
    // draws them black and hides whatever lies under them.
    let layer = rerun::GridMap::new(grid.bytes, format, grid.cell_m)
        .with_translation([x, y, lift])
        .with_opacity(0.999);
    // Static: only the newest grid is kept, where a new one a second would pile up.
    put_static(rec, path, &layer);
}

/// ROS occupancy (-1 unknown, 0 free, 100 occupied) for a dark view: the floor a shade above the
/// background, walls light, and the unknown clear, so the map reads as the building alone.
pub(super) fn map_colour(v: i64) -> [u8; 4] {
    match v {
        ..0 => [0, 0, 0, 0],
        0..50 => [0x2e, 0x35, 0x40, 255],
        _ => [0xc9, 0xcf, 0xd8, 255],
    }
}

/// canopy's coverage values as colours over the map: still to see a strong amber, as it is what
/// the operator looks for; seen a faint green and written off a faint grey, so a well-seen
/// building stays a plan and does not turn into a green field; the rest clear.
pub(super) fn coverage_colour(v: i64) -> [u8; 4] {
    match v {
        SEEN => [70, 180, 90, 38],
        TO_SEE => [240, 160, 40, 150],
        WRITTEN_OFF => [130, 130, 130, 50],
        _ => [0, 0, 0, 0],
    }
}

/// Where a grid has known cells, free or occupied: `[x0, y0, x1, y1]` in the map frame. Unknown
/// cells can reach far past the building and would frame a view on nothing.
pub(super) fn known_extent(msg: &Value) -> Option<[f32; 4]> {
    let info = &msg["info"];
    let res = num(&info["resolution"]);
    let width = usize::try_from(info["width"].as_u64()?).ok()?;
    let data = msg["data"].as_array()?;
    if res <= 0.0 || width == 0 {
        return None;
    }
    let (mut c0, mut r0, mut c1, mut r1) = (usize::MAX, usize::MAX, 0, 0);
    for (i, v) in data.iter().enumerate() {
        if v.as_i64().is_some_and(|v| v >= 0) {
            let (c, r) = (i % width, i / width);
            (c0, r0, c1, r1) = (c0.min(c), r0.min(r), c1.max(c), r1.max(r));
        }
    }
    if c0 > c1 {
        return None;
    }
    let origin = &info["origin"]["position"];
    let (x, y) = (num(&origin["x"]), num(&origin["y"]));
    #[expect(clippy::cast_precision_loss, reason = "cell counts far below 2^52")]
    let at = |cell: usize| cell as f64 * res;
    Some(f32s([
        x + at(c0),
        y + at(r0),
        x + at(c1 + 1),
        y + at(r1 + 1),
    ]))
}

pub(super) fn draw_map(rec: &RecordingStream, path: &str, msg: &Value) {
    if let Some(g) = grid(msg, map_colour) {
        put_grid(rec, path, g, rerun::ColorModel::RGBA, 0.0);
    }
}

pub(super) fn draw_coverage(rec: &RecordingStream, path: &str, msg: &Value) {
    if let Some(g) = grid(msg, coverage_colour) {
        put_grid(rec, path, g, rerun::ColorModel::RGBA, 0.01);
    }
}

/// How much of a room the camera has seen, the mean of its floor and walls (canopy's fields).
pub(super) fn seen(room: &Value) -> Option<f64> {
    let floor = room["floor_coverage"].as_f64()?;
    let faces = room["face_coverage"].as_f64().unwrap_or(floor);
    Some(f64::midpoint(floor, faces).clamp(0.0, 1.0))
}

/// Red for unseen through amber to green for seen.
pub(super) fn seen_colour(seen: f64) -> rerun::Color {
    let lerp = |a: u8, b: u8, t: f64| {
        let v = f64::from(a) + (f64::from(b) - f64::from(a)) * t;
        #[expect(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "v is within 0..=255"
        )]
        let byte = v.round().clamp(0.0, 255.0) as u8;
        byte
    };
    let (from, to, t) = if seen < 0.5 {
        ((220, 80, 60), (235, 170, 40), seen * 2.0)
    } else {
        ((235, 170, 40), (70, 180, 90), (seen - 0.5) * 2.0)
    };
    rerun::Color::from_rgb(
        lerp(from.0, to.0, t),
        lerp(from.1, to.1, t),
        lerp(from.2, to.2, t),
    )
}

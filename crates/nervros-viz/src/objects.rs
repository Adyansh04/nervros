//! The world model's objects: boxes coloured by state, with their names.

use std::collections::BTreeMap;

use rerun::RecordingStream;
use serde_json::{Value, json};

use crate::{OBJECT_PALETTE, REMOVED, STALE, f32s, layers, put, put_static, xyz};

/// A colour by label: FNV-1a over its bytes into the palette.
pub(super) fn object_colour(label: &str, alpha: u8) -> rerun::Color {
    let hash = label.bytes().fold(0xcbf2_9ce4_8422_2325_u64, |h, b| {
        (h ^ u64::from(b)).wrapping_mul(0x100_0000_01b3)
    });
    let (r, g, b) = OBJECT_PALETTE[usize::try_from(hash % 8).unwrap_or(0)];
    rerun::Color::from_unmultiplied_rgba(r, g, b, alpha)
}

/// What an object is called: its name, or its label until it has one.
fn object_name(o: &Value) -> &str {
    o["name"]
        .as_str()
        .filter(|n| !n.is_empty())
        .or_else(|| o["label"].as_str())
        .unwrap_or_default()
}

/// Every object's name above it, as one batch redrawn whole.
pub(super) fn draw_object_names(rec: &RecordingStream, path: &str, msg: &Value) {
    let list = msg["objects"].as_array().map_or(&[][..], Vec::as_slice);
    let (mut at, mut names) = (Vec::new(), Vec::new());
    for o in list.iter().filter(|o| o["state"].as_u64() != Some(REMOVED)) {
        let [x, y, z] = xyz(&o["pose"]["position"]);
        at.push([x, y, z + xyz(&o["size"])[2] / 2.0 + 0.05]);
        names.push(format!(
            "{} {}",
            o["id"].as_str().unwrap_or_default(),
            object_name(o)
        ));
    }
    put_static(
        rec,
        path,
        &rerun::Points3D::new(at)
            .with_labels(names)
            .with_show_labels(true)
            .with_radii([0.01]),
    );
}

/// Objects from a `canopy_msgs/msg/WorldObjectArray`, one entity each so a click in the viewer
/// names the object; stale ones are faded, and ones that went away are cleared.
pub(super) fn draw_objects(
    rec: &RecordingStream,
    path: &str,
    msg: &Value,
    drawn: &mut BTreeMap<String, Value>,
) {
    let list = msg["objects"].as_array().map_or(&[][..], Vec::as_slice);
    let mut now = BTreeMap::new();
    for o in list.iter().filter(|o| o["state"].as_u64() != Some(REMOVED)) {
        let Some(id) = o["id"].as_str().filter(|i| !i.is_empty()) else {
            continue;
        };
        // Only what changes how it is drawn; one object's change redraws that object alone.
        let looks = json!([o["pose"], o["size"], o["state"], o["label"], o["name"]]);
        let unchanged = drawn.get(id) == Some(&looks);
        now.insert(id.to_owned(), looks);
        if unchanged {
            continue;
        }
        let pose = layers::pose(&o["pose"]);
        let label = o["label"].as_str().unwrap_or_default();
        let name = object_name(o);
        let alpha = if o["state"].as_u64() == Some(STALE) {
            90
        } else {
            255
        };
        put_static(
            rec,
            &format!("{path}/{id}"),
            &rerun::Boxes3D::from_centers_and_half_sizes(
                [f32s(pose.translation)],
                [xyz(&o["size"]).map(|v| v / 2.0)],
            )
            .with_quaternions([rerun::Quaternion::from_xyzw(f32s(pose.rotation))])
            .with_labels([format!("{id} {name}")])
            .with_show_labels(false)
            .with_colors([object_colour(label, alpha)]),
        );
    }
    for gone in drawn.keys().filter(|id| !now.contains_key(*id)) {
        put_static(rec, &format!("{path}/{gone}"), &rerun::Clear::flat());
    }
    #[expect(clippy::cast_precision_loss, reason = "far fewer objects than 2^52")]
    put(
        rec,
        "mapping/objects",
        &rerun::Scalars::single(now.len() as f64),
    );
    *drawn = now;
}

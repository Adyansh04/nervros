//! The world model as the editor reads it: rooms, objects, the map's frame and the floor plan.

use std::sync::Arc;

use rerun::external::egui;
use rerun::external::egui::{Color32, Rect, Vec2};
use serde::Deserialize;
use serde_json::Value;

use super::Picked;

pub(super) const OBJECT: Color32 = Color32::from_rgb(110, 160, 255);

pub(super) const CHECKED: Color32 = Color32::from_rgb(80, 190, 110);

pub(super) const SUSPECT: Color32 = Color32::from_rgb(240, 170, 50);

pub(super) const REMOVED: Color32 = Color32::from_rgb(225, 85, 70);

/// The world as `/api/world` gives it; only what the editor draws and edits.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub(crate) struct World {
    pub(super) map: MapMeta,
    pub(super) rooms: Vec<Room>,
    pub(super) objects: Vec<Object>,
    pub(super) suggestions: Vec<Suggestion>,
    pub(super) labels: Vec<String>,
    pub(super) room_types: Vec<String>,
    pub(super) can_undo: bool,
    pub(super) can_redo: bool,
    pub(super) unsaved: u64,
}

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub(super) struct MapMeta {
    pub(super) resolution: f64,
    pub(super) origin: [f64; 2],
    pub(super) width: u32,
    pub(super) height: u32,
}

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub(super) struct Room {
    pub(super) id: String,
    pub(super) name: String,
    #[serde(rename = "type")]
    pub(super) kind: String,
    pub(super) type_source: String,
    pub(super) type_confidence: f64,
    pub(super) checked: bool,
    pub(super) outline: Vec<[f64; 2]>,
    pub(super) x: f64,
    pub(super) y: f64,
}

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
#[expect(clippy::struct_excessive_bools, reason = "the editor's own fields")]
pub(super) struct Object {
    pub(super) id: u64,
    pub(super) label: String,
    pub(super) operator_label: String,
    pub(super) name: String,
    pub(super) caption: String,
    pub(super) votes: serde_json::Map<String, Value>,
    pub(super) weight: f64,
    pub(super) observations: u64,
    pub(super) state: String,
    pub(super) removed_by: String,
    pub(super) centre: [f64; 2],
    pub(super) size: [f64; 2],
    pub(super) yaw: f64,
    pub(super) z_min: f64,
    pub(super) z_max: f64,
    pub(super) voxels: u64,
    pub(super) box_pinned: bool,
    pub(super) checked: bool,
    pub(super) crop: bool,
    pub(super) shown: bool,
}

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub(super) struct Suggestion {
    pub(super) kind: String,
    pub(super) id: Value,
    pub(super) reasons: Vec<String>,
    pub(super) phantom: bool,
}

impl Suggestion {
    pub(super) fn picked(&self) -> Option<Picked> {
        if self.kind == "object" {
            self.id.as_u64().map(Picked::Object)
        } else {
            self.id.as_str().map(|r| Picked::Room(r.to_owned()))
        }
    }
}

/// A name to draw once every shape is down.
pub(super) struct Label {
    /// The selected or hovered one first, then rooms, then objects, larger ones before smaller.
    pub(super) first: u8,
    pub(super) area: f64,
    pub(super) rect: Rect,
    pub(super) galley: Arc<egui::Galley>,
}

/// Draws each label that covers none placed before it, so a crowded corner keeps its larger
/// objects' names and the pointer finds the rest.
pub(super) fn place(painter: &egui::Painter, mut labels: Vec<Label>) {
    labels.sort_by(|a, b| a.first.cmp(&b.first).then(b.area.total_cmp(&a.area)));
    let mut taken: Vec<Rect> = Vec::new();
    for label in labels {
        if taken.iter().any(|r| r.intersects(label.rect)) {
            continue;
        }
        // A dark backing, so the outlines under a name do not cross it.
        painter.rect_filled(label.rect, 3.0, Color32::from_black_alpha(170));
        // The galley carries its colour; the fallback is for placeholder text only.
        painter.galley(
            label.rect.min + Vec2::new(4.0, 1.0),
            label.galley,
            Color32::WHITE,
        );
        taken.push(label.rect.expand(1.5));
    }
}

/// The floor plan in the app's dark palette: free space slate, walls light, unknown near black.
pub(super) fn floor_plan(png: &[u8]) -> Option<egui::ColorImage> {
    let grey = image::load_from_memory_with_format(png, image::ImageFormat::Png)
        .ok()?
        .to_luma8();
    let size = [grey.width(), grey.height()].map(|v| usize::try_from(v).unwrap_or(0));
    let rgba: Vec<u8> = grey
        .pixels()
        .flat_map(|p| match p.0[0] {
            250.. => [44, 47, 55, 255],
            0..=60 => [190, 195, 205, 255],
            _ => [24, 25, 29, 255],
        })
        .collect();
    Some(egui::ColorImage::from_rgba_unmultiplied(size, &rgba))
}

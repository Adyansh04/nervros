//! The world editor, native: canopy's saved world on its floor plan, where the operator picks,
//! moves, resizes, turns, splits, merges and adds objects and types rooms, as on canopy's own
//! editor page. Every change goes through the editor's HTTP API, so the agent's edits and the
//! operator's are one history, and each shows in the other within a poll.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use nervros_core::editor::EditorClient;
use rerun::external::egui::{self, Pos2, Rect, Vec2};
use serde_json::Value;

mod canvas;
mod geometry;
mod panel;
mod world;

use geometry::Shape2;
use world::World;
use world::floor_plan;

/// How often the world is read again, for edits made elsewhere (the agent, the web page).
const POLL: Duration = Duration::from_secs(2);
const HANDLE_PX: f32 = 7.0;
const ROTATE_PX: f32 = 26.0;

/// A box's smallest side, in metres, as the web page keeps it.
const MIN_SIDE_M: f64 = 0.05;

/// What is selected.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum Picked {
    Object(u64),
    Room(String),
}

/// What the next gesture on the map means.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    Browse,
    Add,
    Split,
    Merge,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Grip {
    Move,
    Rotate,
    Corner(usize),
}

#[derive(Debug, Clone, Copy)]
enum Drag {
    Pan {
        from: Pos2,
        offset: Vec2,
    },
    Draw {
        start: [f64; 2],
        yaw: f64,
    },
    Box {
        grip: Grip,
        from: [f64; 2],
        at: Pos2,
        origin: Shape2,
        moved: bool,
    },
}

/// Where the map is on screen: map pixel (0, 0) at `offset`, `scale` screen points per pixel.
#[derive(Debug, Clone, Copy)]
struct View {
    scale: f32,
    offset: Vec2,
}

/// What the requests running on the runtime hand back.
#[derive(Default)]
struct Shared {
    /// Shared with each frame that draws it: it changes every few seconds, frames come far more
    /// often.
    world: Option<Arc<World>>,
    map: Option<egui::ColorImage>,
    crops: HashMap<u64, Option<Arc<egui::ColorImage>>>,
    status: Option<(String, bool)>,
    busy: bool,
    select: Option<Picked>,
    reachable: Option<bool>,
    /// Requests sent so far: an answer to an older one than the newest brings an older world.
    sent: u64,
}

impl Shared {
    /// Takes a world the editor sent; one this window cannot read keeps the last and says why,
    /// rather than leaving an empty canvas that waits for ever.
    fn take(&mut self, world: Value) -> Result<(), String> {
        match serde_json::from_value(world) {
            Ok(w) => {
                self.world = Some(Arc::new(w));
                Ok(())
            }
            Err(e) => Err(format!(
                "the editor sent a world this window cannot read: {e}"
            )),
        }
    }
}

/// The form fields of the selection card.
#[derive(Default)]
struct Form {
    label: String,
    name: String,
    kind: String,
    low: f64,
    high: f64,
    /// Whose fields these are, so a new selection fills them afresh.
    of: Option<Picked>,
}

/// A request to the editor.
enum Call {
    Edit(Value, String),
    Command(&'static str, &'static str),
    Save,
}

/// The editor's state in the app.
pub(crate) struct WorldEditor {
    client: Arc<EditorClient>,
    runtime: tokio::runtime::Handle,
    shared: Arc<Mutex<Shared>>,
    map: Option<egui::TextureHandle>,
    crops: HashMap<u64, egui::TextureHandle>,
    view: Option<View>,
    selected: Option<Picked>,
    mode: Mode,
    drag: Option<Drag>,
    preview: Option<(u64, Shape2)>,
    drawn: Option<Shape2>,
    merge_with: Option<u64>,
    confirm_delete: bool,
    confirm_phantoms: bool,
    form: Form,
    last_poll: Option<Instant>,
    /// Where the canvas was last drawn, for centring on a thing picked from the dock.
    canvas: Rect,
}

fn lock(m: &Mutex<Shared>) -> MutexGuard<'_, Shared> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

impl WorldEditor {
    /// An editor for the world model behind `client`, its calls run on `runtime`.
    pub(crate) fn new(client: Arc<EditorClient>, runtime: tokio::runtime::Handle) -> Self {
        Self {
            client,
            runtime,
            shared: Arc::default(),
            map: None,
            crops: HashMap::new(),
            view: None,
            selected: None,
            mode: Mode::Browse,
            drag: None,
            preview: None,
            drawn: None,
            merge_with: None,
            confirm_delete: false,
            confirm_phantoms: false,
            form: Form::default(),
            last_poll: None,
            canvas: Rect::NOTHING,
        }
    }

    /// A world already in hand, as a test shows one.
    #[cfg(test)]
    pub(crate) fn show_world(&mut self, world: &Value, map: egui::ColorImage) {
        let mut shared = lock(&self.shared);
        shared.world = serde_json::from_value(world.clone()).ok().map(Arc::new);
        shared.map = Some(map);
        shared.reachable = Some(true);
    }

    /// Selects what a click in the viewer would.
    #[cfg(test)]
    pub(crate) fn select(&mut self, picked: Picked) {
        self.selected = Some(picked);
        self.last_poll = Some(Instant::now());
    }

    fn world(&self) -> Option<Arc<World>> {
        lock(&self.shared).world.clone()
    }

    /// Reads the world, and the floor plan the first time, unless a request is running.
    fn poll(&mut self, ctx: &egui::Context) {
        let due = self.last_poll.is_none_or(|t| t.elapsed() >= POLL);
        if !due || self.drag.is_some() || lock(&self.shared).busy {
            return;
        }
        self.last_poll = Some(Instant::now());
        let (client, shared, ctx) = (
            Arc::clone(&self.client),
            Arc::clone(&self.shared),
            ctx.clone(),
        );
        let want_map = self.map.is_none() && lock(&self.shared).map.is_none();
        let asked = lock(&self.shared).sent;
        self.runtime.spawn(async move {
            let world = client.world().await;
            let map = if want_map {
                client.map_png().await.ok().and_then(|p| floor_plan(&p))
            } else {
                None
            };
            let mut s = lock(&shared);
            match world {
                // An edit sent since brings a newer world than this read.
                Ok(_) if s.sent != asked => s.reachable = Some(true),
                Ok(w) => {
                    s.reachable = Some(true);
                    if let Err(why) = s.take(w) {
                        s.status = Some((why, true));
                    }
                }
                Err(e) => {
                    s.reachable = Some(false);
                    s.status = Some((e.to_string(), true));
                }
            }
            if map.is_some() {
                s.map = map;
            }
            drop(s);
            ctx.request_repaint();
        });
    }

    fn call(&self, ctx: &egui::Context, call: Call) {
        let (client, shared, ctx) = (
            Arc::clone(&self.client),
            Arc::clone(&self.shared),
            ctx.clone(),
        );
        let mine = {
            let mut s = lock(&self.shared);
            s.busy = true;
            s.sent += 1;
            s.sent
        };
        self.runtime.spawn(async move {
            let answer = match call {
                Call::Edit(op, message) => client.edit(&op).await.map(|e| {
                    let select = e.created.first().map(|id| Picked::Object(*id));
                    (message, e.world, select)
                }),
                Call::Command(name, message) => client.command(name).await.map(|(said, w)| {
                    (
                        if said.is_empty() {
                            message.to_owned()
                        } else {
                            said
                        },
                        w,
                        None,
                    )
                }),
                Call::Save => client.save().await.map(|(said, w)| (said, w, None)),
            };
            let mut s = lock(&shared);
            // Only the newest request's answer sets the world: two edits can end out of order.
            let newest = s.sent == mine;
            if newest {
                s.busy = false;
            }
            match answer {
                Ok((message, world, select)) if newest => match s.take(world) {
                    Ok(()) => {
                        s.status = Some((message, false));
                        s.select = select;
                    }
                    Err(why) => s.status = Some((format!("{message}, but {why}"), true)),
                },
                Ok(_) => {}
                Err(e) => s.status = Some((e.to_string(), true)),
            }
            drop(s);
            ctx.request_repaint();
        });
    }

    fn edit(&self, ctx: &egui::Context, op: Value, message: String) {
        self.call(ctx, Call::Edit(op, message));
    }

    /// The camera's view of an object, fetched once.
    fn crop(&mut self, ctx: &egui::Context, id: u64) -> Option<egui::TextureHandle> {
        if let Some(t) = self.crops.get(&id) {
            return Some(t.clone());
        }
        let cached = lock(&self.shared).crops.get(&id).cloned();
        match cached {
            Some(Some(pixels)) => {
                let texture =
                    ctx.load_texture(format!("crop-{id}"), pixels, egui::TextureOptions::LINEAR);
                self.crops.insert(id, texture.clone());
                Some(texture)
            }
            Some(None) => None,
            None => {
                lock(&self.shared).crops.insert(id, None);
                let (client, shared, ctx) = (
                    Arc::clone(&self.client),
                    Arc::clone(&self.shared),
                    ctx.clone(),
                );
                self.runtime.spawn(async move {
                    let image = client
                        .crop(id)
                        .await
                        .ok()
                        .flatten()
                        .and_then(|b| crate::chat::decode(&b));
                    lock(&shared).crops.insert(id, image.map(Arc::new));
                    ctx.request_repaint();
                });
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use rerun::external::egui::Color32;
    use rerun::external::re_ui::UiExt as _;
    use serde_json::json;

    use super::geometry::{in_polygon, wall_yaw};
    use super::world::Object;
    use super::*;

    #[test]
    fn a_box_knows_its_corners_and_what_it_holds() {
        let shape = Shape2 {
            centre: [1.0, 2.0],
            size: [2.0, 1.0],
            yaw: std::f64::consts::FRAC_PI_2,
        };
        let corners = shape.corners();
        // Turned a quarter: its length runs along y.
        assert!(
            (corners[0][0] - 0.5).abs() < 1e-9 && (corners[0][1] - 3.0).abs() < 1e-9,
            "{corners:?}"
        );
        assert!(shape.contains([1.0, 2.9]));
        assert!(!shape.contains([1.9, 2.0]));
        let back = shape.to_box(shape.to_map([0.3, -0.2]));
        assert!((back[0] - 0.3).abs() < 1e-9 && (back[1] + 0.2).abs() < 1e-9);
    }

    /// A 10 x 6 m floor plan at 5 cm: walls round the edge and between the two rooms.
    fn plan() -> egui::ColorImage {
        let (w, h) = (200_usize, 120_usize);
        let pixels = (0..w * h)
            .map(|i| {
                let (x, y) = (i % w, i / w);
                let wall = x < 3
                    || y < 3
                    || x >= w - 3
                    || y >= h - 3
                    || (x == 100 && !(50..70).contains(&y));
                if wall {
                    Color32::from_rgb(190, 195, 205)
                } else {
                    Color32::from_rgb(44, 47, 55)
                }
            })
            .collect();
        egui::ColorImage::new([w, h], pixels)
    }

    fn fixture() -> Value {
        json!({
            "map": {"resolution": 0.05, "origin": [0.0, 0.0], "width": 200, "height": 120},
            "rooms": [
                {"id": "R1", "type": "kitchen", "name": "", "outline": [[0.1, 0.1], [5.0, 0.1], [5.0, 5.9], [0.1, 5.9]]},
                {"id": "R2", "type": "", "name": "room B", "outline": [[5.0, 0.1], [9.9, 0.1], [9.9, 5.9], [5.0, 5.9]]}
            ],
            "objects": [
                {"id": 1, "label": "table", "centre": [2.0, 3.0], "size": [1.2, 0.8], "yaw": 0.0, "shown": true, "checked": true, "state": "active", "votes": {"table": 3.2}, "observations": 9, "z_max": 0.75},
                {"id": 2, "label": "chair", "centre": [3.2, 2.0], "size": [0.5, 0.5], "yaw": 0.3, "shown": true, "state": "active",
                 "votes": {"chair": 1.2, "stool": 1.0}, "observations": 3, "weight": 2.2, "caption": "a wooden chair", "z_max": 0.9, "voxels": 41},
                {"id": 3, "label": "sofa", "centre": [7.5, 3.0], "size": [2.0, 0.9], "yaw": 0.0, "shown": true, "state": "active", "box_pinned": true},
                {"id": 4, "label": "box", "centre": [8.5, 1.0], "size": [0.4, 0.4], "yaw": 0.0, "shown": false, "state": "removed", "removed_by": "no depth behind it"}
            ],
            "suggestions": [
                {"kind": "object", "id": 2, "reasons": ["split vote: chair 1.2, stool 1.0"], "phantom": false},
                {"kind": "room", "id": "R2", "reasons": ["no room type yet"], "phantom": false}
            ],
            "labels": ["chair", "sofa", "stool", "table"],
            "room_types": ["bedroom", "kitchen", "living room"],
            "can_undo": true, "can_redo": false, "unsaved": 2
        })
    }

    #[test]
    fn snapshot_editor() {
        let runtime = tokio::runtime::Runtime::new().unwrap();
        let config: nervros_core::profile::EditorConfig =
            serde_json::from_value(json!({"url": "http://127.0.0.1:9"})).unwrap();
        let client = Arc::new(EditorClient::new(&config).unwrap());
        let mut editor = WorldEditor::new(client, runtime.handle().clone());
        editor.show_world(&fixture(), plan());
        editor.select(Picked::Object(2));
        let mut harness = egui_kittest::Harness::builder()
            .wgpu()
            .with_size(egui::vec2(1100.0, 560.0))
            .build_ui(move |ui| {
                egui::Panel::right("editor_panel")
                    .exact_size(340.0)
                    .frame(
                        egui::Frame::new()
                            .fill(ui.tokens().panel_bg_color)
                            .inner_margin(12),
                    )
                    .show(ui, |ui| editor.panel(ui));
                egui::CentralPanel::no_frame().show(ui, |ui| editor.canvas(ui));
            });
        crate::testkit::style_for_tests(&harness.ctx);
        harness.run_steps(4);
        crate::testkit::compare(
            &mut harness,
            "editor",
            &egui_kittest::SnapshotOptions::new(),
        );
    }

    #[test]
    fn rooms_are_found_by_outline_and_the_walls_by_the_common_yaw() {
        let square = vec![[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]];
        assert!(in_polygon(&square, [1.0, 1.0]));
        assert!(!in_polygon(&square, [5.0, 1.0]));
        let objects = [0.1, 0.1 + std::f64::consts::FRAC_PI_2, 0.4].map(|yaw| Object {
            yaw,
            ..Object::default()
        });
        assert!((wall_yaw(&objects) - 0.1).abs() < 0.02);
    }
}

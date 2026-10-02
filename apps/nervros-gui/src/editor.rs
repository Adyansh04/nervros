//! The world editor, native: canopy's saved world on its floor plan, where the operator picks,
//! moves, resizes, turns, splits, merges and adds objects and types rooms, as on canopy's own
//! editor page. Every change goes through the editor's HTTP API, so the agent's edits and the
//! operator's are one history, and each shows in the other within a poll.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use nervros_core::editor::EditorClient;
use rerun::external::egui::{
    self, Align2, Color32, FontId, Key, Modifiers, Pos2, Rect, RichText, Sense, Shape, Stroke, Vec2,
};
use rerun::external::re_ui::{ReButton, UiExt as _};
use serde::Deserialize;
use serde_json::{Value, json};

/// How often the world is read again, for edits made elsewhere (the agent, the web page).
const POLL: Duration = Duration::from_secs(2);
const HANDLE_PX: f32 = 7.0;
const ROTATE_PX: f32 = 26.0;
/// A box's smallest side, in metres, as the web page keeps it.
const MIN_SIDE_M: f64 = 0.05;

const OBJECT: Color32 = Color32::from_rgb(110, 160, 255);
const CHECKED: Color32 = Color32::from_rgb(80, 190, 110);
const SUSPECT: Color32 = Color32::from_rgb(240, 170, 50);
const REMOVED: Color32 = Color32::from_rgb(225, 85, 70);

/// The world as `/api/world` gives it; only what the editor draws and edits.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub(crate) struct World {
    map: MapMeta,
    rooms: Vec<Room>,
    objects: Vec<Object>,
    suggestions: Vec<Suggestion>,
    labels: Vec<String>,
    room_types: Vec<String>,
    can_undo: bool,
    can_redo: bool,
    unsaved: u64,
}

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
struct MapMeta {
    resolution: f64,
    origin: [f64; 2],
    width: u32,
    height: u32,
}

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
struct Room {
    id: String,
    name: String,
    #[serde(rename = "type")]
    kind: String,
    type_source: String,
    type_confidence: f64,
    checked: bool,
    outline: Vec<[f64; 2]>,
    x: f64,
    y: f64,
}

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
#[expect(clippy::struct_excessive_bools, reason = "the editor's own fields")]
struct Object {
    id: u64,
    label: String,
    operator_label: String,
    name: String,
    caption: String,
    votes: serde_json::Map<String, Value>,
    weight: f64,
    observations: u64,
    state: String,
    removed_by: String,
    centre: [f64; 2],
    size: [f64; 2],
    yaw: f64,
    z_min: f64,
    z_max: f64,
    voxels: u64,
    box_pinned: bool,
    checked: bool,
    crop: bool,
    shown: bool,
}

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
struct Suggestion {
    kind: String,
    id: Value,
    reasons: Vec<String>,
    phantom: bool,
}

impl Suggestion {
    fn picked(&self) -> Option<Picked> {
        if self.kind == "object" {
            self.id.as_u64().map(Picked::Object)
        } else {
            self.id.as_str().map(|r| Picked::Room(r.to_owned()))
        }
    }
}

/// A name to draw once every shape is down.
struct Label {
    /// The selected or hovered one first, then rooms, then objects, larger ones before smaller.
    first: u8,
    area: f64,
    rect: Rect,
    galley: Arc<egui::Galley>,
}

/// Draws each label that covers none placed before it, so a crowded corner keeps its larger
/// objects' names and the pointer finds the rest.
fn place(painter: &egui::Painter, mut labels: Vec<Label>) {
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

/// A box on the floor: centre and yaw in the map frame, size along its own axes, all in metres.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Shape2 {
    centre: [f64; 2],
    size: [f64; 2],
    yaw: f64,
}

impl Shape2 {
    fn of(o: &Object) -> Self {
        Self {
            centre: o.centre,
            size: o.size,
            yaw: o.yaw,
        }
    }

    /// A map point in the box's own axes.
    fn to_box(self, [x, y]: [f64; 2]) -> [f64; 2] {
        let (s, c) = self.yaw.sin_cos();
        let (dx, dy) = (x - self.centre[0], y - self.centre[1]);
        [c * dx + s * dy, -s * dx + c * dy]
    }

    /// A point in the box's axes on the map.
    fn to_map(self, [u, v]: [f64; 2]) -> [f64; 2] {
        let (s, c) = self.yaw.sin_cos();
        [
            self.centre[0] + c * u - s * v,
            self.centre[1] + s * u + c * v,
        ]
    }

    /// Corners in the web page's order: (+,+), (-,+), (-,-), (+,-).
    fn corners(&self) -> [[f64; 2]; 4] {
        let [hu, hv] = [self.size[0] / 2.0, self.size[1] / 2.0];
        [[hu, hv], [-hu, hv], [-hu, -hv], [hu, -hv]].map(|p| self.to_map(p))
    }

    fn contains(&self, p: [f64; 2]) -> bool {
        let [u, v] = self.to_box(p);
        u.abs() <= self.size[0] / 2.0 && v.abs() <= self.size[1] / 2.0
    }
}

/// A box moved, turned or resized by a grip dragged from `from` to `at`, as the web page does it.
fn reshape(origin: Shape2, grip: Grip, from: [f64; 2], at: [f64; 2]) -> Shape2 {
    let mut shape = origin;
    match grip {
        Grip::Move => {
            shape.centre = [
                origin.centre[0] + at[0] - from[0],
                origin.centre[1] + at[1] - from[1],
            ];
        }
        Grip::Rotate => {
            shape.yaw = (at[1] - origin.centre[1]).atan2(at[0] - origin.centre[0])
                - std::f64::consts::FRAC_PI_2;
        }
        Grip::Corner(i) => {
            // The opposite corner stays where it is.
            let signs = [[1.0, 1.0], [-1.0, 1.0], [-1.0, -1.0], [1.0, -1.0]][i];
            let anchor = Shape2 {
                centre: origin.to_map([
                    -signs[0] * origin.size[0] / 2.0,
                    -signs[1] * origin.size[1] / 2.0,
                ]),
                size: [0.0, 0.0],
                yaw: origin.yaw,
            };
            let [u, v] = anchor.to_box(at);
            shape.size = [u.abs().max(MIN_SIDE_M), v.abs().max(MIN_SIDE_M)];
            shape.centre = anchor.to_map([u / 2.0, v / 2.0]);
        }
    }
    shape
}

fn in_polygon(polygon: &[[f64; 2]], [x, y]: [f64; 2]) -> bool {
    let points: Vec<(f64, f64)> = polygon.iter().map(|&[a, b]| (a, b)).collect();
    nervros_core::builtins::inside((x, y), &points)
}

/// The walls' angle, which canopy lays boxes along: the most common yaw, a quarter turn apart.
fn wall_yaw(objects: &[Object]) -> f64 {
    let quarter = std::f64::consts::FRAC_PI_2;
    let mut counts: HashMap<i64, usize> = HashMap::new();
    for o in objects {
        #[expect(
            clippy::cast_possible_truncation,
            reason = "a bucket of a quarter turn"
        )]
        let bucket = (o.yaw.rem_euclid(quarter) * 50.0).round() as i64;
        *counts.entry(bucket).or_default() += 1;
    }
    #[expect(clippy::cast_precision_loss, reason = "a small bucket number")]
    let best = counts
        .into_iter()
        .max_by_key(|&(_, n)| n)
        .map_or(0.0, |(b, _)| b as f64 / 50.0);
    best
}

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

/// The floor plan in the app's dark palette: free space slate, walls light, unknown near black.
fn floor_plan(png: &[u8]) -> Option<egui::ColorImage> {
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

    fn to_screen(&self, world: &World, [x, y]: [f64; 2]) -> Pos2 {
        let (m, v) = (
            &world.map,
            self.view.unwrap_or(View {
                scale: 1.0,
                offset: Vec2::ZERO,
            }),
        );
        let res = m.resolution.max(1e-6);
        #[expect(clippy::cast_possible_truncation, reason = "screen coordinates")]
        let px = [
            ((x - m.origin[0]) / res) as f32,
            (f64::from(m.height) - (y - m.origin[1]) / res) as f32,
        ];
        Pos2::new(px[0] * v.scale + v.offset.x, px[1] * v.scale + v.offset.y)
    }

    fn to_world(&self, world: &World, p: Pos2) -> [f64; 2] {
        let (m, v) = (
            &world.map,
            self.view.unwrap_or(View {
                scale: 1.0,
                offset: Vec2::ZERO,
            }),
        );
        let px = [
            f64::from((p.x - v.offset.x) / v.scale),
            f64::from((p.y - v.offset.y) / v.scale),
        ];
        [
            m.origin[0] + px[0] * m.resolution,
            m.origin[1] + (f64::from(m.height) - px[1]) * m.resolution,
        ]
    }

    fn fit(&mut self, world: &World, rect: Rect) {
        #[expect(clippy::cast_precision_loss, reason = "map sizes are far below 2^24")]
        let (w, h) = (
            world.map.width.max(1) as f32,
            world.map.height.max(1) as f32,
        );
        let scale = (rect.width() / w).min(rect.height() / h) * 0.95;
        let offset = rect.center().to_vec2() - Vec2::new(w, h) * scale / 2.0;
        self.view = Some(View { scale, offset });
    }

    fn shape_of(&self, o: &Object) -> Shape2 {
        match self.preview {
            Some((id, shape)) if id == o.id => shape,
            _ => Shape2::of(o),
        }
    }

    fn selected_object<'w>(&self, world: &'w World) -> Option<&'w Object> {
        match &self.selected {
            Some(Picked::Object(id)) => world.objects.iter().find(|o| o.id == *id),
            _ => None,
        }
    }

    fn rotate_knob(&self, world: &World, shape: &Shape2) -> Pos2 {
        let edge = self.to_screen(world, shape.to_map([0.0, shape.size[1] / 2.0]));
        let centre = self.to_screen(world, shape.centre);
        let dir = (edge - centre).normalized();
        edge + dir * ROTATE_PX
    }

    /// Which part of the selected box the pointer is on.
    fn grip_at(&self, world: &World, p: Pos2) -> Option<Grip> {
        let object = self.selected_object(world)?;
        let shape = self.shape_of(object);
        for (i, corner) in shape.corners().iter().enumerate() {
            if self.to_screen(world, *corner).distance(p) <= HANDLE_PX + 2.0 {
                return Some(Grip::Corner(i));
            }
        }
        if self.rotate_knob(world, &shape).distance(p) <= HANDLE_PX + 2.0 {
            return Some(Grip::Rotate);
        }
        shape
            .contains(self.to_world(world, p))
            .then_some(Grip::Move)
    }

    fn object_at<'w>(&self, world: &'w World, p: [f64; 2]) -> Option<&'w Object> {
        world
            .objects
            .iter()
            .filter(|o| o.shown && self.shape_of(o).contains(p))
            .min_by(|a, b| (a.size[0] * a.size[1]).total_cmp(&(b.size[0] * b.size[1])))
    }

    fn room_at(world: &World, p: [f64; 2]) -> Option<&Room> {
        world
            .rooms
            .iter()
            .find(|r| r.outline.len() > 2 && in_polygon(&r.outline, p))
            .or_else(|| {
                world
                    .rooms
                    .iter()
                    .filter(|r| r.outline.len() <= 2)
                    .map(|r| (r, (r.x - p[0]).hypot(r.y - p[1])))
                    .filter(|(_, d)| *d < 1.5)
                    .min_by(|a, b| a.1.total_cmp(&b.1))
                    .map(|(r, _)| r)
            })
    }

    fn suspicion<'w>(world: &'w World, picked: &Picked) -> Option<&'w Suggestion> {
        world
            .suggestions
            .iter()
            .find(|s| s.picked().as_ref() == Some(picked))
    }

    fn start_mode(&mut self, mode: Mode, world: &World) {
        if matches!(mode, Mode::Split | Mode::Merge) && self.selected_object(world).is_none() {
            lock(&self.shared).status = Some(("select an object first".to_owned(), true));
            return;
        }
        self.mode = mode;
        self.drawn = None;
        self.merge_with = None;
        self.form.label.clear();
        self.form.name.clear();
        (self.form.low, self.form.high) = (0.0, 0.8);
    }

    fn stop_mode(&mut self) {
        self.mode = Mode::Browse;
        self.drawn = None;
        self.merge_with = None;
        self.form.of = None;
    }

    fn centre_on(&mut self, world: &World, p: [f64; 2]) {
        let screen = self.to_screen(world, p);
        let centre = self.canvas.center();
        if let Some(v) = &mut self.view
            && self.canvas.is_positive()
        {
            v.offset += centre - screen;
        }
    }

    /// The map, drawn, with every gesture on it.
    pub(crate) fn canvas(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        self.poll(&ctx);
        let (response, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let rect = response.rect;
        self.canvas = rect;
        painter.rect_filled(rect, 0.0, Color32::from_rgb(18, 19, 22));
        let Some(world) = self.world() else {
            let reachable = lock(&self.shared).reachable;
            let text = match reachable {
                Some(false) => {
                    "The world editor does not answer. Start it on the world:\ncanopy's editor/canopy_editor.py <world_dir>"
                }
                _ => "Reading the saved world…",
            };
            painter.text(
                rect.center(),
                Align2::CENTER_CENTER,
                text,
                FontId::proportional(14.0),
                ui.tokens().text_subdued,
            );
            return;
        };
        let select = lock(&self.shared).select.take();
        if let Some(select) = select {
            self.stop_mode();
            self.selected = Some(select);
        }
        let image = if self.map.is_none() {
            lock(&self.shared).map.take()
        } else {
            None
        };
        if let Some(image) = image {
            self.map = Some(ctx.load_texture("floor-plan", image, egui::TextureOptions::NEAREST));
        }
        if self.view.is_none() && world.map.width > 0 {
            self.fit(&world, rect);
        }
        self.gestures(ui, &response, &world);
        self.keys(ui, &world);
        let painter = painter.with_clip_rect(rect);
        let hovered = response
            .hover_pos()
            .and_then(|p| self.object_at(&world, self.to_world(&world, p)))
            .map(|o| o.id);
        self.draw(&painter, &world, hovered);
        self.overlay(ui, &painter, rect);
    }

    fn gestures(&mut self, ui: &egui::Ui, response: &egui::Response, world: &World) {
        if response.hovered() {
            self.zoom(ui, response);
        }
        let pointer = response.interact_pointer_pos();
        if response.drag_started_by(egui::PointerButton::Primary)
            && let Some(p) = pointer
        {
            self.drag = Some(self.start_drag(world, p));
        }
        if response.dragged()
            && let (Some(p), Some(drag)) = (pointer, self.drag)
        {
            self.move_drag(world, p, drag);
        }
        if response.drag_stopped()
            && let Some(drag) = self.drag.take()
        {
            self.end_drag(ui.ctx(), drag);
        }
        if response.clicked()
            && let Some(p) = pointer
        {
            self.click(world, p);
        }
    }

    /// Scroll zooms about the pointer.
    fn zoom(&mut self, ui: &egui::Ui, response: &egui::Response) {
        let scroll = ui.input(|i| i.smooth_scroll_delta().y);
        if scroll != 0.0
            && let (Some(p), Some(v)) = (response.hover_pos(), &mut self.view)
        {
            let scale = (v.scale * (scroll * 0.0025).exp()).clamp(0.2, 40.0);
            v.offset = p.to_vec2() - (p.to_vec2() - v.offset) * (scale / v.scale);
            v.scale = scale;
        }
    }

    /// A drag draws a box in the add and split modes, takes the selected box by a grip, or pans.
    fn start_drag(&mut self, world: &World, p: Pos2) -> Drag {
        let at = self.to_world(world, p);
        if matches!(self.mode, Mode::Add | Mode::Split) {
            let yaw = match (self.mode, self.selected_object(world)) {
                (Mode::Split, Some(o)) => o.yaw,
                _ => wall_yaw(&world.objects),
            };
            return Drag::Draw { start: at, yaw };
        }
        match self.grip_at(world, p).zip(self.selected_object(world)) {
            Some((grip, object)) if self.mode == Mode::Browse => {
                let origin = self.shape_of(object);
                self.preview = Some((object.id, origin));
                Drag::Box {
                    grip,
                    from: at,
                    at: p,
                    origin,
                    moved: false,
                }
            }
            _ => Drag::Pan {
                from: p,
                offset: self.view.map_or(Vec2::ZERO, |v| v.offset),
            },
        }
    }

    fn move_drag(&mut self, world: &World, p: Pos2, drag: Drag) {
        let at = self.to_world(world, p);
        match drag {
            Drag::Pan { from, offset } => {
                if let Some(v) = &mut self.view {
                    v.offset = offset + (p - from);
                }
            }
            Drag::Draw { start, yaw } => {
                let frame = Shape2 {
                    centre: start,
                    size: [0.0, 0.0],
                    yaw,
                };
                let [u, v] = frame.to_box(at);
                self.drawn = Some(Shape2 {
                    centre: frame.to_map([u / 2.0, v / 2.0]),
                    size: [u.abs(), v.abs()],
                    yaw,
                });
            }
            Drag::Box {
                grip,
                from,
                at: start,
                origin,
                ..
            } => {
                let shape = reshape(origin, grip, from, at);
                self.preview = Some((self.preview.map_or(0, |p| p.0), shape));
                if let Some(Drag::Box { moved, .. }) = &mut self.drag {
                    *moved = *moved || p.distance(start) > 3.0;
                }
            }
        }
    }

    fn end_drag(&mut self, ctx: &egui::Context, drag: Drag) {
        match drag {
            Drag::Draw { .. } => {
                if self
                    .drawn
                    .is_some_and(|d| d.size[0] < MIN_SIDE_M || d.size[1] < MIN_SIDE_M)
                {
                    self.drawn = None;
                }
            }
            Drag::Box { moved, .. } => {
                // A click on the box is no edit: only a drag pins it.
                if moved && let Some((id, s)) = self.preview {
                    let op = json!({"op": "box", "id": id, "centre": s.centre, "size": s.size, "yaw": s.yaw});
                    self.edit(ctx, op, format!("O{id}'s box set by hand"));
                }
                self.preview = None;
            }
            Drag::Pan { .. } => {}
        }
    }

    fn click(&mut self, world: &World, p: Pos2) {
        let at = self.to_world(world, p);
        let object = self.object_at(world, at).map(|o| o.id);
        if self.mode == Mode::Merge {
            if let (Some(other), Some(Picked::Object(into))) = (object, &self.selected)
                && other != *into
            {
                self.merge_with = Some(other);
            }
        } else if self.mode == Mode::Browse {
            self.confirm_delete = false;
            self.selected = match object {
                Some(id) => Some(Picked::Object(id)),
                None => Self::room_at(world, at).map(|r| Picked::Room(r.id.clone())),
            };
        }
    }

    fn keys(&mut self, ui: &egui::Ui, world: &World) {
        let ctx = ui.ctx().clone();
        let command = Modifiers::COMMAND;
        let (save, redo, undo) = ui.input_mut(|i| {
            (
                i.consume_key(command, Key::S),
                i.consume_key(command | Modifiers::SHIFT, Key::Z) || i.consume_key(command, Key::Y),
                i.consume_key(command, Key::Z),
            )
        });
        if save {
            self.call(&ctx, Call::Save);
        }
        if redo {
            self.call(&ctx, Call::Command("redo", "redone"));
        } else if undo {
            self.call(&ctx, Call::Command("undo", "undone"));
        }
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let (escape, delete, check, next) = ui.input_mut(|i| {
            (
                i.consume_key(Modifiers::NONE, Key::Escape),
                i.consume_key(Modifiers::NONE, Key::Delete),
                i.consume_key(Modifiers::NONE, Key::C),
                i.consume_key(Modifiers::NONE, Key::N),
            )
        });
        if escape {
            if self.mode == Mode::Browse {
                self.selected = None;
            } else {
                self.stop_mode();
            }
        }
        if delete && self.selected_object(world).is_some() {
            self.confirm_delete = true;
        }
        if check {
            match &self.selected {
                Some(Picked::Object(id)) => {
                    if let Some(o) = world.objects.iter().find(|o| o.id == *id) {
                        let now = if o.checked { "unchecked" } else { "checked" };
                        self.edit(
                            &ctx,
                            json!({"op": "check", "id": id, "checked": !o.checked}),
                            format!("O{id} {now}"),
                        );
                    }
                }
                Some(Picked::Room(id)) => {
                    if let Some(r) = world.rooms.iter().find(|r| r.id == *id) {
                        let now = if r.checked { "unchecked" } else { "checked" };
                        self.edit(
                            &ctx,
                            json!({"op": "room_check", "room": id, "checked": !r.checked}),
                            format!("{id} {now}"),
                        );
                    }
                }
                None => {}
            }
        }
        if next && !world.suggestions.is_empty() {
            let at = world
                .suggestions
                .iter()
                .position(|s| s.picked() == self.selected)
                .map_or(0, |i| (i + 1) % world.suggestions.len());
            self.go_to(world, &world.suggestions[at]);
        }
    }

    fn go_to(&mut self, world: &World, item: &Suggestion) {
        let Some(picked) = item.picked() else { return };
        let target = match &picked {
            Picked::Object(id) => world.objects.iter().find(|o| o.id == *id).map(|o| o.centre),
            Picked::Room(id) => world.rooms.iter().find(|r| r.id == *id).map(room_centre),
        };
        self.selected = Some(picked);
        if let Some(p) = target {
            self.centre_on(world, p);
        }
    }

    fn draw(&self, painter: &egui::Painter, world: &World, hovered: Option<u64>) {
        if let (Some(texture), Some(v)) = (&self.map, self.view) {
            #[expect(clippy::cast_precision_loss, reason = "map sizes are far below 2^24")]
            let size = Vec2::new(world.map.width as f32, world.map.height as f32) * v.scale;
            let uv = Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0));
            let at = Rect::from_min_size(v.offset.to_pos2(), size);
            painter.image(texture.id(), at, uv, Color32::WHITE);
        }
        let mut labels = Vec::new();
        for (i, room) in world.rooms.iter().enumerate() {
            labels.push(self.draw_room(painter, world, i, room));
        }
        for object in world
            .objects
            .iter()
            .filter(|o| o.shown || self.selected == Some(Picked::Object(o.id)))
        {
            labels.push(self.draw_object(painter, world, object, hovered == Some(object.id)));
        }
        place(painter, labels);
        if let Some(drawn) = &self.drawn {
            let mut points: Vec<Pos2> = drawn
                .corners()
                .iter()
                .map(|c| self.to_screen(world, *c))
                .collect();
            points.push(points[0]);
            let stroke = Stroke::new(2.0, SUSPECT);
            painter.extend(Shape::dashed_line(&points, stroke, 6.0, 4.0));
        }
    }

    fn draw_room(
        &self,
        painter: &egui::Painter,
        world: &World,
        index: usize,
        room: &Room,
    ) -> Label {
        #[expect(clippy::cast_precision_loss, reason = "a handful of rooms")]
        let hue = (index as f32 * 67.0 % 360.0) / 360.0;
        let colour: Color32 = egui::ecolor::Hsva::new(hue, 0.55, 0.85, 1.0).into();
        let picked = Picked::Room(room.id.clone());
        let chosen = self.selected.as_ref() == Some(&picked);
        if room.outline.len() > 2 {
            let points: Vec<Pos2> = room
                .outline
                .iter()
                .map(|p| self.to_screen(world, *p))
                .collect();
            let (width, alpha) = if chosen { (3.0, 1.0) } else { (1.2, 0.6) };
            let stroke = Stroke::new(width, colour.gamma_multiply(alpha));
            painter.add(Shape::closed_line(points, stroke));
        }
        let kind = if room.kind.is_empty() {
            &room.name
        } else {
            &room.kind
        };
        let doubt = if Self::suspicion(world, &picked).is_some() {
            " ?"
        } else {
            ""
        };
        let at = self.to_screen(world, room_centre(room));
        let text = format!("{} {kind}{doubt}", room.id);
        let galley = painter.layout_no_wrap(text, FontId::proportional(14.0), colour);
        Label {
            first: u8::from(!chosen),
            area: 0.0,
            rect: Rect::from_center_size(at, galley.size() + Vec2::new(8.0, 2.0)),
            galley,
        }
    }

    fn draw_object(
        &self,
        painter: &egui::Painter,
        world: &World,
        object: &Object,
        hovered: bool,
    ) -> Label {
        let picked = Picked::Object(object.id);
        let chosen = self.selected.as_ref() == Some(&picked);
        let colour = if !object.shown {
            REMOVED
        } else if object.checked {
            CHECKED
        } else if Self::suspicion(world, &picked).is_some() {
            SUSPECT
        } else {
            OBJECT
        };
        let shape = self.shape_of(object);
        let points: Vec<Pos2> = shape
            .corners()
            .iter()
            .map(|c| self.to_screen(world, *c))
            .collect();
        let line = Stroke::new(if chosen { 2.5 } else { 1.4 }, colour);
        if chosen {
            let fill = Color32::from_rgba_unmultiplied(80, 130, 230, 40);
            painter.add(Shape::convex_polygon(points.clone(), fill, Stroke::NONE));
        }
        if object.box_pinned {
            let mut closed = points.clone();
            closed.push(points[0]);
            painter.extend(Shape::dashed_line(&closed, line, 5.0, 3.0));
        } else {
            painter.add(Shape::closed_line(points.clone(), line));
        }
        if chosen {
            for p in &points {
                let handle = Rect::from_center_size(*p, Vec2::splat(HANDLE_PX));
                painter.rect_filled(handle, 1.0, colour);
            }
            let knob = self.rotate_knob(world, &shape);
            let edge = self.to_screen(world, shape.to_map([0.0, shape.size[1] / 2.0]));
            painter.line_segment([edge, knob], line);
            painter.circle_filled(knob, HANDLE_PX / 1.4, colour);
        }
        let at = self.to_screen(world, shape.centre);
        let galley =
            painter.layout_no_wrap(object.label.clone(), FontId::proportional(11.5), colour);
        Label {
            first: if chosen || hovered { 0 } else { 2 },
            area: shape.size[0] * shape.size[1],
            rect: Rect::from_center_size(at, galley.size() + Vec2::new(8.0, 2.0)),
            galley,
        }
    }

    /// The hint for the gesture a mode waits for, and a small legend.
    fn overlay(&self, ui: &egui::Ui, painter: &egui::Painter, rect: Rect) {
        let hint = match self.mode {
            Mode::Browse => None,
            Mode::Add => {
                Some("Drag a box over the object the models missed. Esc cancels.".to_owned())
            }
            Mode::Split => Some("Drag a box over the part to split off. Esc cancels.".to_owned()),
            Mode::Merge => {
                Some("Click the object to merge into the selected one. Esc cancels.".to_owned())
            }
        };
        if let Some(hint) = hint {
            let galley = painter.layout_no_wrap(hint, FontId::proportional(13.0), Color32::WHITE);
            let pill = Rect::from_center_size(
                Pos2::new(rect.center().x, rect.top() + 22.0),
                galley.size() + Vec2::new(20.0, 10.0),
            );
            painter.rect_filled(pill, 6.0, Color32::from_rgba_unmultiplied(40, 44, 54, 235));
            painter.galley(pill.min + Vec2::new(10.0, 5.0), galley, Color32::WHITE);
        }
        let mut x = rect.left() + 12.0;
        let y = rect.bottom() - 16.0;
        for (colour, name) in [
            (OBJECT, "object"),
            (SUSPECT, "to review"),
            (CHECKED, "checked"),
            (REMOVED, "removed"),
        ] {
            painter.rect_filled(
                Rect::from_center_size(Pos2::new(x + 5.0, y), Vec2::splat(9.0)),
                2.0,
                colour,
            );
            let galley = painter.layout_no_wrap(
                name.to_owned(),
                FontId::proportional(11.0),
                ui.tokens().text_subdued,
            );
            let width = galley.size().x;
            painter.galley(
                Pos2::new(x + 14.0, y - galley.size().y / 2.0),
                galley,
                ui.tokens().text_subdued,
            );
            x += width + 30.0;
        }
        let dashed = painter.layout_no_wrap(
            "dashed: box set by hand".to_owned(),
            FontId::proportional(11.0),
            ui.tokens().text_subdued,
        );
        painter.galley(
            Pos2::new(x, y - dashed.size().y / 2.0),
            dashed,
            ui.tokens().text_subdued,
        );
    }

    /// The dock's side of the editor: the toolbar, the selection, the review and removed lists.
    pub(crate) fn panel(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        let Some(world) = self.world() else {
            ui.label(
                RichText::new("No world from the editor yet.").color(ui.tokens().text_subdued),
            );
            return;
        };
        let (busy, status) = {
            let s = lock(&self.shared);
            (s.busy, s.status.clone())
        };
        ui.horizontal_wrapped(|ui| {
            ui.add_enabled_ui(!busy, |ui| {
                if ui
                    .add_enabled(world.can_undo, ReButton::new("Undo").small())
                    .clicked()
                {
                    self.call(&ctx, Call::Command("undo", "undone"));
                }
                if ui
                    .add_enabled(world.can_redo, ReButton::new("Redo").small())
                    .clicked()
                {
                    self.call(&ctx, Call::Command("redo", "redone"));
                }
                if ui.add(ReButton::new("Add object").small()).clicked() {
                    self.start_mode(Mode::Add, &world);
                }
                let save = if world.unsaved > 0 {
                    format!("Save ({})", world.unsaved)
                } else {
                    "Save".to_owned()
                };
                if ui
                    .add_enabled(world.unsaved > 0, ReButton::new(save).small().primary())
                    .clicked()
                {
                    self.call(&ctx, Call::Save);
                }
                if ui
                    .add(ReButton::new("Reload").small().ghost())
                    .on_hover_text("Read the saved world again, dropping edits not saved")
                    .clicked()
                {
                    self.call(
                        &ctx,
                        Call::Command("reload", "read the world from disk again"),
                    );
                }
            });
        });
        if let Some((text, error)) = status {
            let colour = if error {
                REMOVED
            } else {
                ui.tokens().text_subdued
            };
            ui.label(RichText::new(text).small().color(colour));
        }
        ui.add_space(6.0);
        ui.full_span_separator();
        ui.add_space(6.0);
        self.card(ui, &world);
        ui.add_space(8.0);
        self.review(ui, &world);
    }

    fn card(&mut self, ui: &mut egui::Ui, world: &World) {
        let ctx = ui.ctx().clone();
        if self.mode == Mode::Add
            && let Some(drawn) = self.drawn
        {
            ui.label(RichText::new("New object").strong());
            ui.label(
                RichText::new("Saved as one the operator saw: canopy keeps its box.")
                    .small()
                    .color(ui.tokens().text_subdued),
            );
            text_field(ui, "What it is", &mut self.form.label, &world.labels);
            text_field(ui, "Name", &mut self.form.name, &[]);
            ui.horizontal(|ui| {
                ui.label("From");
                ui.add(
                    egui::DragValue::new(&mut self.form.low)
                        .speed(0.01)
                        .range(0.0..=3.0)
                        .suffix(" m"),
                );
                ui.label("to");
                ui.add(
                    egui::DragValue::new(&mut self.form.high)
                        .speed(0.01)
                        .range(0.0..=3.0)
                        .suffix(" m"),
                );
                ui.label("high");
            });
            ui.horizontal(|ui| {
                let ok = !self.form.label.trim().is_empty() && self.form.high > self.form.low;
                if ui.add_enabled(ok, ReButton::new("Add").small().primary()).clicked() {
                    let op = json!({"op": "add", "label": self.form.label, "name": self.form.name, "centre": drawn.centre,
                        "size": drawn.size, "yaw": drawn.yaw, "z_min": self.form.low, "z_max": self.form.high});
                    self.edit(&ctx, op, format!("added a {}", self.form.label.trim()));
                }
                if ui.add(ReButton::new("Cancel").small().ghost()).clicked() {
                    self.stop_mode();
                }
            });
            return;
        }
        if self.mode == Mode::Split
            && let (Some(drawn), Some(object)) = (self.drawn, self.selected_object(world))
        {
            let id = object.id;
            ui.label(RichText::new(format!("Split off part of O{id}")).strong());
            ui.label(
                RichText::new("The voxels inside the drawn box become a new object.")
                    .small()
                    .color(ui.tokens().text_subdued),
            );
            text_field(ui, "What the part is", &mut self.form.label, &world.labels);
            text_field(ui, "Name", &mut self.form.name, &[]);
            ui.horizontal(|ui| {
                if ui.add_enabled(!self.form.label.trim().is_empty(), ReButton::new("Split").small().primary()).clicked() {
                    let op = json!({"op": "split", "id": id, "polygon": drawn.corners(), "label": self.form.label, "name": self.form.name});
                    self.edit(&ctx, op, format!("split a {} off O{id}", self.form.label.trim()));
                }
                if ui.add(ReButton::new("Cancel").small().ghost()).clicked() {
                    self.stop_mode();
                }
            });
            return;
        }
        match self.selected.clone() {
            Some(Picked::Object(id)) => match world.objects.iter().find(|o| o.id == id) {
                Some(object) => self.object_card(ui, world, object),
                None => self.selected = None,
            },
            Some(Picked::Room(id)) => match world.rooms.iter().find(|r| r.id == id) {
                Some(room) => self.room_card(ui, world, room),
                None => self.selected = None,
            },
            None => {
                ui.label(
                    RichText::new(
                        "Click an object or a room on the map. Drag to pan, scroll to zoom.",
                    )
                    .color(ui.tokens().text_subdued),
                );
            }
        }
    }

    fn object_card(&mut self, ui: &mut egui::Ui, world: &World, object: &Object) {
        let ctx = ui.ctx().clone();
        let (id, picked) = (object.id, Picked::Object(object.id));
        if self.form.of.as_ref() != Some(&picked) {
            self.form = Form {
                label: object.label.clone(),
                name: object.name.clone(),
                of: Some(picked.clone()),
                ..Form::default()
            };
        }
        ui.horizontal_wrapped(|ui| {
            let title = RichText::new(format!("O{id} {}", object.label));
            ui.label(title.strong().size(15.0));
            if object.checked {
                badge(ui, "checked", CHECKED);
            }
            if object.box_pinned {
                badge(ui, "box set by hand", OBJECT);
            }
            if object.state != "active" {
                badge(ui, &object.state, REMOVED);
            }
        });
        if object.crop
            && let Some(texture) = self.crop(&ctx, id)
        {
            let size = texture.size_vec2();
            let width = size.x.min(ui.available_width()).min(300.0);
            let fit = Vec2::new(width, size.y * width / size.x.max(1.0));
            ui.add(
                egui::Image::new(&texture)
                    .fit_to_exact_size(fit)
                    .corner_radius(4.0),
            );
        }
        let reasons = Self::suspicion(world, &picked).map_or(&[][..], |s| s.reasons.as_slice());
        for reason in reasons {
            ui.label(RichText::new(format!("• {reason}")).small().color(SUSPECT));
        }
        if !object.removed_by.is_empty() {
            let why = RichText::new(format!("• {}", object.removed_by));
            ui.label(why.small().color(REMOVED));
        }
        ui.add_space(4.0);
        self.object_fields(ui, world, object);
        ui.add_space(6.0);
        if !self.object_confirm(ui, world, object) {
            self.object_actions(ui, world, object);
        }
    }

    /// The label and name, with the facts behind them.
    fn object_fields(&mut self, ui: &mut egui::Ui, world: &World, object: &Object) {
        let (ctx, id) = (ui.ctx().clone(), object.id);
        text_field(ui, "What it is", &mut self.form.label, &world.labels);
        ui.horizontal(|ui| {
            if ui.add(ReButton::new("Set label").small()).clicked() {
                let label = self.form.label.trim().to_owned();
                let op = json!({"op": "label", "id": id, "label": label});
                self.edit(&ctx, op, format!("O{id} is now {label}"));
            }
            let votes = ReButton::new("Use the votes").small().ghost();
            if !object.operator_label.is_empty() && ui.add(votes).clicked() {
                let op = json!({"op": "label", "id": id, "label": ""});
                self.edit(&ctx, op, format!("O{id} goes back to the detector's label"));
            }
        });
        text_field(ui, "Name", &mut self.form.name, &[]);
        if ui.add(ReButton::new("Set name").small()).clicked() {
            let op = json!({"op": "name", "id": id, "name": self.form.name.trim()});
            self.edit(&ctx, op, format!("O{id} named"));
        }
        ui.add_space(4.0);
        let mut votes: Vec<(&String, f64)> = object
            .votes
            .iter()
            .map(|(k, v)| (k, v.as_f64().unwrap_or(0.0)))
            .collect();
        votes.sort_by(|a, b| b.1.total_cmp(&a.1));
        let votes: Vec<String> = votes
            .iter()
            .take(4)
            .map(|(k, v)| format!("{k} {v:.1}"))
            .collect();
        let or = |text: String, none: &str| {
            if text.is_empty() {
                none.to_owned()
            } else {
                text
            }
        };
        let facts = [
            ("votes", or(votes.join(" · "), "none")),
            ("describer", or(object.caption.clone(), "no caption")),
            (
                "sightings",
                format!("{}, vote weight {:.1}", object.observations, object.weight),
            ),
            (
                "size",
                format!(
                    "{:.2} × {:.2} m, {:.2}–{:.2} m high",
                    object.size[0], object.size[1], object.z_min, object.z_max
                ),
            ),
            ("voxels", object.voxels.to_string()),
        ];
        egui::Grid::new(("facts", id))
            .num_columns(2)
            .spacing([10.0, 2.0])
            .show(ui, |ui| {
                let dim = ui.tokens().text_subdued;
                for (k, v) in facts {
                    ui.label(RichText::new(k).small().color(dim));
                    ui.label(RichText::new(v).small());
                    ui.end_row();
                }
            });
    }

    /// A merge or a delete waiting for the operator's word; `false` when there is none.
    fn object_confirm(&mut self, ui: &mut egui::Ui, world: &World, object: &Object) -> bool {
        let (ctx, id) = (ui.ctx().clone(), object.id);
        if let Some(other) = self.merge_with {
            let label = world
                .objects
                .iter()
                .find(|o| o.id == other)
                .map_or("?", |o| o.label.as_str());
            ui.label(format!("Merge O{other} ({label}) into O{id}?"));
            ui.horizontal(|ui| {
                if ui.add(ReButton::new("Merge").small().primary()).clicked() {
                    self.stop_mode();
                    let op = json!({"op": "merge", "into": id, "ids": [id, other]});
                    self.edit(&ctx, op, format!("O{other} merged into O{id}"));
                }
                if ui.add(ReButton::new("Cancel").small().ghost()).clicked() {
                    self.stop_mode();
                }
            });
            return true;
        }
        if !self.confirm_delete {
            return false;
        }
        ui.label(format!(
            "Delete O{id} ({}) for good? Remove keeps it, restorable.",
            object.label
        ));
        ui.horizontal(|ui| {
            if ui.add(ReButton::new("Delete").small().primary()).clicked() {
                self.confirm_delete = false;
                self.edit(
                    &ctx,
                    json!({"op": "delete", "id": id}),
                    format!("O{id} deleted"),
                );
            }
            if ui.add(ReButton::new("Remove instead").small()).clicked() {
                self.confirm_delete = false;
                let op = json!({"op": "remove", "ids": [id], "reason": "removed by hand"});
                self.edit(&ctx, op, format!("O{id} removed"));
            }
            if ui.add(ReButton::new("Cancel").small().ghost()).clicked() {
                self.confirm_delete = false;
            }
        });
        true
    }

    fn object_actions(&mut self, ui: &mut egui::Ui, world: &World, object: &Object) {
        let (ctx, id) = (ui.ctx().clone(), object.id);
        ui.horizontal_wrapped(|ui| {
            let restore = ReButton::new("Restore").small().primary();
            if object.state == "removed" && ui.add(restore).clicked() {
                let op = json!({"op": "restore", "ids": [id]});
                self.edit(&ctx, op, format!("O{id} restored, and checked"));
            }
            let (check, done) = if object.checked {
                ("Uncheck", "unchecked")
            } else {
                ("Mark checked", "checked")
            };
            if ui
                .add(ReButton::new(check).small())
                .on_hover_text("C")
                .clicked()
            {
                let op = json!({"op": "check", "id": id, "checked": !object.checked});
                self.edit(&ctx, op, format!("O{id} {done}"));
            }
            if ui.add(ReButton::new("Merge with…").small()).clicked() {
                self.start_mode(Mode::Merge, world);
            }
            if ui.add(ReButton::new("Split off…").small()).clicked() {
                self.start_mode(Mode::Split, world);
            }
            let delete = ReButton::new("Delete").small().ghost();
            if ui.add(delete).on_hover_text("Del").clicked() {
                self.confirm_delete = true;
            }
        });
    }

    fn room_card(&mut self, ui: &mut egui::Ui, world: &World, room: &Room) {
        let ctx = ui.ctx().clone();
        let (id, picked) = (room.id.clone(), Picked::Room(room.id.clone()));
        if self.form.of.as_ref() != Some(&picked) {
            self.form = Form {
                kind: room.kind.clone(),
                name: room.name.clone(),
                of: Some(picked.clone()),
                ..Form::default()
            };
        }
        ui.horizontal_wrapped(|ui| {
            ui.label(
                RichText::new(format!(
                    "{id} {}",
                    if room.kind.is_empty() {
                        &room.name
                    } else {
                        &room.kind
                    }
                ))
                .strong()
                .size(15.0),
            );
            if room.checked {
                badge(ui, "checked", CHECKED);
            }
        });
        if let Some(s) = Self::suspicion(world, &picked) {
            for reason in &s.reasons {
                ui.label(RichText::new(format!("• {reason}")).small().color(SUSPECT));
            }
        }
        text_field(ui, "Type", &mut self.form.kind, &world.room_types);
        if ui.add(ReButton::new("Set type").small()).clicked() {
            let kind = self.form.kind.trim().to_owned();
            self.edit(
                &ctx,
                json!({"op": "room_type", "room": id, "type": kind}),
                format!("{id} is a {kind}"),
            );
        }
        text_field(ui, "Name", &mut self.form.name, &[]);
        if ui.add(ReButton::new("Set name").small()).clicked() {
            let name = self.form.name.trim().to_owned();
            self.edit(
                &ctx,
                json!({"op": "room_name", "room": id, "name": name}),
                format!("{id} renamed"),
            );
        }
        let typed = if room.type_source.is_empty() {
            "nothing yet".to_owned()
        } else {
            format!("{}, {:.2}", room.type_source, room.type_confidence)
        };
        ui.label(
            RichText::new(format!("typed by {typed}"))
                .small()
                .color(ui.tokens().text_subdued),
        );
        let check = if room.checked {
            "Uncheck"
        } else {
            "Mark checked"
        };
        if ui
            .add(ReButton::new(check).small())
            .on_hover_text("C")
            .clicked()
        {
            self.edit(
                &ctx,
                json!({"op": "room_check", "room": id, "checked": !room.checked}),
                format!(
                    "{id} {}",
                    if room.checked { "unchecked" } else { "checked" }
                ),
            );
        }
    }

    fn review(&mut self, ui: &mut egui::Ui, world: &World) {
        let ctx = ui.ctx().clone();
        let phantoms: Vec<u64> = world
            .suggestions
            .iter()
            .filter(|s| s.phantom)
            .filter_map(|s| s.id.as_u64())
            .collect();
        ui.horizontal(|ui| {
            ui.label(RichText::new(format!("To review ({})", world.suggestions.len())).strong());
            if !phantoms.is_empty()
                && ui
                    .add(
                        ReButton::new(format!("Remove {} likely phantoms…", phantoms.len()))
                            .small()
                            .ghost(),
                    )
                    .clicked()
            {
                self.confirm_phantoms = true;
            }
        });
        if self.confirm_phantoms {
            ui.label(RichText::new(format!("Remove {} objects that look like no object at all? They stay in the file and each can be restored.", phantoms.len())).small());
            ui.horizontal(|ui| {
                if ui.add(ReButton::new("Remove them").small().primary()).clicked() {
                    self.confirm_phantoms = false;
                    let op = json!({"op": "remove", "ids": phantoms, "reason": "removed by hand: a likely phantom"});
                    self.edit(&ctx, op, format!("{} likely phantoms removed", phantoms.len()));
                }
                if ui.add(ReButton::new("Cancel").small().ghost()).clicked() {
                    self.confirm_phantoms = false;
                }
            });
        }
        if world.suggestions.is_empty() {
            ui.label(
                RichText::new("Nothing to review.")
                    .small()
                    .color(ui.tokens().text_subdued),
            );
        }
        for item in &world.suggestions {
            let title = match item.picked() {
                Some(Picked::Object(id)) => format!(
                    "O{id} {}",
                    world
                        .objects
                        .iter()
                        .find(|o| o.id == id)
                        .map_or("", |o| o.label.as_str())
                ),
                Some(Picked::Room(id)) => format!(
                    "{id} {}",
                    world
                        .rooms
                        .iter()
                        .find(|r| r.id == id)
                        .map_or("", |r| r.kind.as_str())
                ),
                None => continue,
            };
            let reason = item.reasons.first().cloned().unwrap_or_default();
            let chosen = item.picked() == self.selected;
            let text = RichText::new(format!("{title} · {reason}")).small();
            if ui.selectable_label(chosen, text).clicked() {
                self.go_to(world, item);
            }
        }
        let removed: Vec<&Object> = world
            .objects
            .iter()
            .filter(|o| o.state == "removed" && !o.removed_by.is_empty())
            .collect();
        if !removed.is_empty() {
            ui.add_space(6.0);
            egui::CollapsingHeader::new(
                RichText::new(format!("Removed ({})", removed.len())).strong(),
            )
            .default_open(false)
            .show(ui, |ui| {
                for o in removed {
                    let chosen = self.selected == Some(Picked::Object(o.id));
                    if ui
                        .selectable_label(
                            chosen,
                            RichText::new(format!("O{} {} · {}", o.id, o.label, o.removed_by))
                                .small(),
                        )
                        .clicked()
                    {
                        self.selected = Some(Picked::Object(o.id));
                    }
                }
            });
        }
    }
}

fn room_centre(room: &Room) -> [f64; 2] {
    if room.outline.is_empty() {
        return [room.x, room.y];
    }
    #[expect(clippy::cast_precision_loss, reason = "a few hundred corners at most")]
    let n = room.outline.len() as f64;
    let (sx, sy) = room
        .outline
        .iter()
        .fold((0.0, 0.0), |(x, y), p| (x + p[0], y + p[1]));
    [sx / n, sy / n]
}

fn badge(ui: &mut egui::Ui, text: &str, colour: Color32) {
    egui::Frame::new()
        .fill(colour.gamma_multiply(0.18))
        .corner_radius(4.0)
        .inner_margin(egui::Margin::symmetric(6, 1))
        .show(ui, |ui| ui.label(RichText::new(text).small().color(colour)));
}

/// A labelled text field with a menu of known words beside it.
fn text_field(ui: &mut egui::Ui, title: &str, value: &mut String, words: &[String]) {
    ui.label(RichText::new(title).small().color(ui.tokens().text_subdued));
    ui.horizontal(|ui| {
        let width = if words.is_empty() {
            ui.available_width()
        } else {
            ui.available_width() - 30.0
        };
        ui.add(egui::TextEdit::singleline(value).desired_width(width.max(60.0)));
        if !words.is_empty() {
            ui.menu_button("…", |ui| {
                egui::ScrollArea::vertical()
                    .max_height(260.0)
                    .show(ui, |ui| {
                        for word in words {
                            if ui.button(word).clicked() {
                                value.clone_from(word);
                                ui.close();
                            }
                        }
                    });
            });
        }
    });
}

#[cfg(test)]
mod tests {
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
        crate::chat::style_for_tests(&harness.ctx);
        harness.run_steps(4);
        crate::chat::compare(
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

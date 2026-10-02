//! The canvas: the floor plan drawn with its rooms and objects, and picking, dragging and keys on it.

use rerun::external::egui;
use rerun::external::egui::{
    Align2, Color32, FontId, Key, Modifiers, Pos2, Rect, Sense, Shape, Stroke, Vec2,
};
use serde_json::json;

use super::geometry::{Shape2, in_polygon, reshape, room_centre, wall_yaw};
use super::world::{
    CHECKED, Label, OBJECT, Object, REMOVED, Room, SUSPECT, Suggestion, World, place,
};
use super::{
    Call, Drag, Grip, HANDLE_PX, MIN_SIDE_M, Mode, Picked, ROTATE_PX, View, WorldEditor, lock,
};
use crate::theme;

impl WorldEditor {
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

    pub(super) fn selected_object<'w>(&self, world: &'w World) -> Option<&'w Object> {
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

    pub(super) fn suspicion<'w>(world: &'w World, picked: &Picked) -> Option<&'w Suggestion> {
        world
            .suggestions
            .iter()
            .find(|s| s.picked().as_ref() == Some(picked))
    }

    pub(super) fn start_mode(&mut self, mode: Mode, world: &World) {
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

    pub(super) fn stop_mode(&mut self) {
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
                theme::DIM,
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
        self.overlay(&painter, rect);
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

    pub(super) fn go_to(&mut self, world: &World, item: &Suggestion) {
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
    fn overlay(&self, painter: &egui::Painter, rect: Rect) {
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
            let galley =
                painter.layout_no_wrap(name.to_owned(), FontId::proportional(11.0), theme::DIM);
            let width = galley.size().x;
            painter.galley(
                Pos2::new(x + 14.0, y - galley.size().y / 2.0),
                galley,
                theme::DIM,
            );
            x += width + 30.0;
        }
        let dashed = painter.layout_no_wrap(
            "dashed: box set by hand".to_owned(),
            FontId::proportional(11.0),
            theme::DIM,
        );
        painter.galley(Pos2::new(x, y - dashed.size().y / 2.0), dashed, theme::DIM);
    }
}

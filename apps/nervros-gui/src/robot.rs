//! The Robot tab: how the robot is (upright, able to walk, what each hand holds, where it is and
//! in which room, its motors and battery when it reports them), and driving it by hand.
//!
//! Driving goes through the executor, which owns the base: it refuses while a mission runs,
//! clamps and ramps each command, and stops the base when commands pause, so a closed window or
//! a dropped link stops the robot too.

use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use nervros_core::mission::held_by;
use nervros_core::profile::Profile;
use nervros_ros::{Publisher, RobotPort};
use rerun::external::egui::{self, Key, RichText};
use rerun::external::re_ui::{ReButton, UiExt as _};
use serde_json::{Value, json};

/// How often a held command is sent; the executor stops the base after 0.4 s without one.
const SEND_EVERY: Duration = Duration::from_millis(100);
const SERVICE_TIMEOUT: Duration = Duration::from_secs(3);

/// Where the robot stands on the map: x, y and heading in radians.
pub type Pose = (f64, f64, f64);

/// What the tab shows, read from the robot by the window's background tasks.
#[derive(Debug, Default, Clone)]
pub struct Status {
    /// The executor's `RobotState`.
    pub state: Option<Value>,
    pub pose: Option<Pose>,
    /// The world model's rooms message.
    pub rooms: Option<Value>,
    /// Each object's name by id.
    pub names: std::collections::HashMap<String, String>,
    /// `sensor_msgs/msg/BatteryState`.
    pub battery: Option<Value>,
    /// `diagnostic_msgs/msg/DiagnosticArray` with motor temperatures.
    pub motors: Option<Value>,
    /// Whether the robot is armed: driving by hand is motion, which Observe only keeps off.
    pub armed: bool,
}

/// The room whose outline holds `(x, y)`.
pub fn room_at(rooms: &Value, x: f64, y: f64) -> Option<&Value> {
    rooms["rooms"].as_array()?.iter().find(|r| {
        let points: Vec<(f64, f64)> = r["outline"]["points"]
            .as_array()
            .map_or(&[][..], Vec::as_slice)
            .iter()
            .filter_map(|p| Some((p["x"].as_f64()?, p["y"].as_f64()?)))
            .collect();
        nervros_core::builtins::inside((x, y), &points)
    })
}

/// Whether a polygon holds a point, by counting the edges a ray to the right crosses.
/// The hottest motor in a diagnostics message, as `(name, °C)`; none when all read zero, as in
/// the simulator.
pub fn hottest(diagnostics: &Value) -> Option<(String, f64)> {
    diagnostics["status"]
        .as_array()?
        .iter()
        .filter_map(|s| {
            let celsius = s["values"]
                .as_array()?
                .iter()
                .find(|kv| kv["key"] == "winding_temperature_C")?["value"]
                .as_str()?
                .parse::<f64>()
                .ok()?;
            Some((s["name"].as_str()?.to_owned(), celsius))
        })
        .filter(|(_, c)| *c > 0.0)
        .max_by(|a, b| a.1.total_cmp(&b.1))
}

/// What driving by hand has reached, as the background calls report it.
#[derive(Debug, Default)]
struct Link {
    on: bool,
    busy: bool,
    note: Option<String>,
    commands: Option<Publisher>,
    /// When the executor last handed the base over: its state takes a moment to say so.
    since: Option<Instant>,
}

/// Driving the base by hand through the executor's `Teleop` service.
pub struct Drive {
    robot: Arc<dyn RobotPort>,
    service: String,
    topic: String,
    /// Full speed forward, sideways and turning.
    speed: [f64; 3],
    runtime: tokio::runtime::Handle,
    link: Arc<Mutex<Link>>,
    sent: Option<Instant>,
    slow: bool,
}

impl Drive {
    /// When the profile names the executor's teleop service and command topic.
    pub fn new(
        profile: &Profile,
        robot: Arc<dyn RobotPort>,
        runtime: tokio::runtime::Handle,
    ) -> Option<Self> {
        let mission = profile.mission.as_ref()?;
        Some(Self {
            robot,
            service: mission.teleop.clone()?,
            topic: mission.teleop_cmd.clone()?,
            speed: mission.teleop_speed,
            runtime,
            link: Arc::default(),
            sent: None,
            slow: false,
        })
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Asks the executor to hand the base over, or to take it back.
    pub fn set(&self, on: bool) {
        self.link().busy = true;
        let (robot, link) = (Arc::clone(&self.robot), Arc::clone(&self.link));
        let (service, topic) = (self.service.clone(), self.topic.clone());
        self.runtime.spawn(async move {
            let commands = if on {
                match robot.publisher(&topic, "geometry_msgs/msg/Twist").await {
                    Ok(p) => Some(p),
                    Err(e) => {
                        let mut l = link.lock().unwrap_or_else(PoisonError::into_inner);
                        l.busy = false;
                        l.note = Some(format!("cannot send drive commands: {e}"));
                        return;
                    }
                }
            } else {
                None
            };
            let reply = robot
                .call(
                    &service,
                    "nervros_interfaces/srv/Teleop",
                    json!({"enable": on}),
                    SERVICE_TIMEOUT,
                )
                .await;
            let mut l = link.lock().unwrap_or_else(PoisonError::into_inner);
            l.busy = false;
            match reply {
                Ok(r) if r["ok"].as_bool() == Some(true) => {
                    l.on = on;
                    l.commands = commands;
                    l.note = None;
                    l.since = on.then(Instant::now);
                }
                Ok(r) => l.note = r["message"].as_str().map(str::to_owned),
                Err(e) => l.note = Some(e.to_string()),
            }
        });
    }

    /// Ends driving on this side when the executor has, as after its idle limit or a stop; a
    /// state polled before it took the base over does not count.
    fn follow(&self, state: Option<&Value>) {
        let mut l = self.link();
        let settled = l
            .since
            .is_none_or(|t| t.elapsed() > nervros_core::mission::STATE_FRESH);
        if l.on && !l.busy && settled && state.and_then(|s| s["teleop"].as_bool()) == Some(false) {
            l.on = false;
            l.commands = None;
            l.note = Some("the executor ended driving: idle, a stop or a mission".to_owned());
        }
    }

    /// Sends what is held now, ten times a second while driving; nothing held sends a stop.
    fn send(&mut self, ctx: &egui::Context, held: [f64; 3]) {
        let l = self.link();
        let Some(commands) = l.commands.as_ref().filter(|_| l.on) else {
            return;
        };
        if self.sent.is_some_and(|t| t.elapsed() < SEND_EVERY) {
            ctx.request_repaint_after(SEND_EVERY);
            return;
        }
        let scale = if self.slow { 0.5 } else { 1.0 };
        let [x, y, yaw] = [0, 1, 2].map(|i| held[i] * self.speed[i] * scale);
        commands.send(json!({
            "linear": {"x": x, "y": y, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": yaw}
        }));
        drop(l);
        self.sent = Some(Instant::now());
        ctx.request_repaint_after(SEND_EVERY);
    }
}

/// The keys held now as a command: W and S forward and back, A and D turn, Q and E sideways.
/// Not the arrows, which the viewer takes to step its time cursor. None while a text field has
/// the keyboard.
fn keys_held(ctx: &egui::Context) -> [f64; 3] {
    if ctx.egui_wants_keyboard_input() {
        return [0.0; 3];
    }
    ctx.input(|i| {
        let axis = |plus: Key, minus: Key| {
            f64::from(i8::from(i.key_down(plus)) - i8::from(i.key_down(minus)))
        };
        [
            axis(Key::W, Key::S),
            axis(Key::Q, Key::E),
            axis(Key::A, Key::D),
        ]
    })
}

/// The tab.
pub fn tab(ui: &mut egui::Ui, status: &Status, drive: Option<&mut Drive>) {
    state_section(ui, status);
    ui.add_space(12.0);
    if let Some(drive) = drive {
        drive_section(ui, status, drive);
    } else {
        ui.label(RichText::new("Drive by hand").strong());
        ui.label(
            RichText::new("The profile names no teleop service, so the window cannot drive.")
                .small()
                .color(ui.tokens().text_subdued),
        );
    }
}

fn state_section(ui: &mut egui::Ui, status: &Status) {
    let t = ui.tokens();
    let subdued = |text: String| RichText::new(text).small().color(t.text_subdued);
    ui.label(RichText::new("Robot").strong());
    let Some(state) = &status.state else {
        ui.label(subdued(
            "No word from the executor: its state shows here once it runs.".to_owned(),
        ));
        return;
    };
    let can_move = state["can_move"].as_bool().unwrap_or(true);
    ui.horizontal(|ui| {
        let colour = if can_move {
            t.success_text_color
        } else {
            t.error_fg_color
        };
        ui.bullet(colour);
        let text = if can_move {
            "Upright and able to walk".to_owned()
        } else {
            format!(
                "Cannot walk: {}",
                state["cannot_move_reason"].as_str().unwrap_or("it says so")
            )
        };
        ui.label(RichText::new(text).color(colour));
    });
    if let Some(tilt) = state["tilt_deg"].as_f64().filter(|t| t.is_finite()) {
        ui.label(subdued(format!("Tilted {tilt:.0}° from upright")));
    }
    if let Some((x, y, yaw)) = status.pose {
        let room = status
            .rooms
            .as_ref()
            .and_then(|rooms| room_at(rooms, x, y))
            .map(|r| {
                let id = r["id"].as_str().unwrap_or_default();
                let kind = r["type"].as_str().filter(|k| !k.is_empty());
                kind.map_or_else(|| format!(" · in {id}"), |k| format!(" · in {id} ({k})"))
            })
            .unwrap_or_default();
        ui.label(subdued(format!(
            "At x {x:.2}, y {y:.2}, facing {:.0}°{room}",
            yaw.to_degrees()
        )));
    }
    for hand in ["left", "right"] {
        let held = held_by(state, hand);
        let text = if held.is_empty() {
            format!("{} hand free", crate::chat::capitalise(hand))
        } else {
            let name = status
                .names
                .get(&held)
                .map_or_else(String::new, |n| format!(" ({n})"));
            format!("{} hand holds {held}{name}", crate::chat::capitalise(hand))
        };
        ui.label(subdued(text));
    }
    if let Some(step) = state["mission_step"].as_str().filter(|s| !s.is_empty()) {
        ui.label(subdued(format!("Running a mission, at {step}")));
    }
    if let Some((motor, celsius)) = status.motors.as_ref().and_then(hottest) {
        ui.label(subdued(format!("Hottest motor {motor}, {celsius:.0} °C")));
    }
    if let Some(battery) = &status.battery {
        let percent = battery["percentage"].as_f64().filter(|p| p.is_finite());
        let volts = battery["voltage"].as_f64().filter(|v| v.is_finite());
        let text = match (percent, volts) {
            (Some(p), Some(v)) => format!("Battery {:.0}% · {v:.1} V", p * 100.0),
            (Some(p), None) => format!("Battery {:.0}%", p * 100.0),
            (None, Some(v)) => format!("Battery {v:.1} V"),
            (None, None) => String::new(),
        };
        if !text.is_empty() {
            ui.label(subdued(text));
        }
    }
}

fn drive_section(ui: &mut egui::Ui, status: &Status, drive: &mut Drive) {
    let t = ui.tokens();
    drive.follow(status.state.as_ref());
    let (on, busy, note) = {
        let l = drive.link();
        (l.on, l.busy, l.note.clone())
    };
    // Disarming hands the base back at once.
    if on && !busy && !status.armed {
        drive.set(false);
    }
    ui.horizontal(|ui| {
        ui.label(RichText::new("Drive by hand").strong());
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            let label = if on { "Stop driving" } else { "Drive" };
            let button = if on {
                ReButton::new(label).small().primary()
            } else {
                ReButton::new(label).small().secondary()
            };
            let response = ui
                .add_enabled(!busy && (on || status.armed), button)
                .on_disabled_hover_text("Arm the robot in the top bar to drive it");
            if response.clicked() {
                drive.set(!on);
            }
            let slow = ReButton::new("Slow")
                .small()
                .secondary()
                .selected(drive.slow);
            if ui.add(slow).on_hover_text("Half speed").clicked() {
                drive.slow = !drive.slow;
            }
        });
    });
    if let Some(note) = note {
        ui.label(RichText::new(note).small().color(t.warn_fg_color));
    }
    if !on {
        ui.label(
            RichText::new(
                "The executor takes the base while you drive: no mission runs, and the base \
                 stops when commands pause.",
            )
            .small()
            .color(t.text_subdued),
        );
        return;
    }
    ui.label(
        RichText::new("Hold W S to walk, A D to turn, Q E to step sideways, or hold a button.")
            .small()
            .color(t.text_subdued),
    );
    let mut held = keys_held(ui.ctx());
    egui::Grid::new("drive_pad")
        .spacing([4.0, 4.0])
        .show(ui, |ui| {
            let mut pad = |ui: &mut egui::Ui, text: &str, axis: usize, sign: f64| {
                let pressed = ui
                    .add(ReButton::new(text).secondary())
                    .is_pointer_button_down_on();
                if pressed {
                    held[axis] = sign;
                }
            };
            pad(ui, "Q · step left", 1, 1.0);
            pad(ui, "W · forward", 0, 1.0);
            pad(ui, "E · step right", 1, -1.0);
            ui.end_row();
            pad(ui, "A · turn left", 2, 1.0);
            pad(ui, "S · back", 0, -1.0);
            pad(ui, "D · turn right", 2, -1.0);
            ui.end_row();
        });
    drive.send(ui.ctx(), held);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_point_is_in_the_room_whose_outline_holds_it() {
        let rooms = json!({"rooms": [
            {"id": "R1", "outline": {"points": [
                {"x": 0.0, "y": 0.0}, {"x": 4.0, "y": 0.0}, {"x": 4.0, "y": 3.0}, {"x": 0.0, "y": 3.0}]}},
            {"id": "R2", "outline": {"points": [
                {"x": 4.0, "y": 0.0}, {"x": 8.0, "y": 0.0}, {"x": 6.0, "y": 3.0}]}}
        ]});
        assert_eq!(room_at(&rooms, 1.0, 1.0).unwrap()["id"], "R1");
        assert_eq!(room_at(&rooms, 6.0, 1.0).unwrap()["id"], "R2");
        assert!(room_at(&rooms, 7.5, 2.5).is_none());
    }

    #[test]
    fn the_hottest_motor_is_named_and_a_simulator_reports_none() {
        let motor = |name: &str, c: &str| json!({"name": name, "values": [{"key": "winding_temperature_C", "value": c}]});
        let real = json!({"status": [motor("left_knee", "41"), motor("right_hip_pitch", "57")]});
        assert_eq!(hottest(&real), Some(("right_hip_pitch".to_owned(), 57.0)));
        let sim = json!({"status": [motor("left_knee", "0")]});
        assert_eq!(hottest(&sim), None);
    }

    fn status() -> Status {
        let rooms = json!({"rooms": [{"id": "R1", "type": "kitchen", "outline": {"points": [
            {"x": 0.0, "y": 0.0}, {"x": 4.0, "y": 0.0}, {"x": 4.0, "y": 3.0}, {"x": 0.0, "y": 3.0}]}}]});
        Status {
            state: Some(
                json!({"can_move": true, "tilt_deg": 2.4, "holding_left": "",
                "holding_right": "O17", "message": "", "mission_step": "", "teleop": true}),
            ),
            pose: Some((1.2, 2.5, std::f64::consts::FRAC_PI_2)),
            rooms: Some(rooms),
            names: [("O17".to_owned(), "red mug".to_owned())].into(),
            battery: Some(json!({"percentage": 0.87, "voltage": 52.1})),
            motors: Some(json!({"status": [{"name": "left_knee",
                "values": [{"key": "winding_temperature_C", "value": "48"}]}]})),
            armed: true,
        }
    }

    fn render(name: &str, status: Status, on: bool) {
        let runtime = tokio::runtime::Runtime::new().unwrap();
        let mut drive = Drive {
            robot: Arc::new(nervros_ros::fake::FakeRobot::new()),
            service: "/x/teleop".to_owned(),
            topic: "/x/teleop_cmd".to_owned(),
            speed: [0.5, 0.3, 0.8],
            runtime: runtime.handle().clone(),
            link: Arc::default(),
            sent: None,
            slow: false,
        };
        drive.link().on = on;
        let mut harness = egui_kittest::Harness::builder()
            .wgpu()
            .with_size(egui::vec2(360.0, 600.0))
            .build_ui(move |ui| {
                egui::Frame::new()
                    .fill(ui.tokens().panel_bg_color)
                    .inner_margin(egui::Margin::same(12))
                    .show(ui, |ui| tab(ui, &status, Some(&mut drive)));
            });
        crate::chat::style_for_tests(&harness.ctx);
        harness.run_steps(3);
        harness.fit_contents();
        crate::chat::compare(&mut harness, name, &egui_kittest::SnapshotOptions::new());
    }

    #[test]
    fn snapshot_robot_tab_driving() {
        render("robot_driving", status(), true);
    }

    #[test]
    fn snapshot_robot_tab_fallen() {
        let mut fallen = status();
        fallen.state = Some(json!({"can_move": false,
            "cannot_move_reason": "fallen: tilted 104 degrees", "tilt_deg": 104.0,
            "holding_left": "", "holding_right": "", "message": "left hand unknown",
            "mission_step": "", "teleop": false}));
        fallen.battery = None;
        fallen.motors = None;
        render("robot_fallen", fallen, false);
    }
}

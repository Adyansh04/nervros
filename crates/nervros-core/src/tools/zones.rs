//! Zones: polygons on the map that keep the robot out, or slow it down.
//!
//! The profile's zones are fixed; the operator adds others in chat (`geofence`), around a room,
//! an object, a place or a polygon, and they are kept across sessions beside the places. The guard
//! refuses a pose goal inside a keep-out zone, the plan check a mission that would end in one or
//! walk through one, and with `[nav2]` Nav2 keeps its own paths out of them too and slows down in
//! the slow zones. Adding a zone or slowing one only makes the robot more careful, so it is free;
//! removing one or speeding one up asks the operator, a keep-out zone every time.

use std::borrow::Cow;
use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::RobotPort;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use tokio::sync::watch;

use crate::places::Places;
use crate::profile::Profile;
use crate::tools::{Assessment, Risk, Tool, ToolOutcome, ToolSpec};

/// A zone smaller than this is a mistake, not a fence.
const MIN_AREA_M2: f64 = 0.05;
/// Kept clear around an object or a place, by default.
const MARGIN_M: f64 = 0.3;
/// Half the side of the square around a place: about the robot's own footprint.
const PLACE_HALF_M: f64 = 0.5;
const WORLD_WAIT: Duration = Duration::from_secs(3);

/// What a zone does to the robot.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Rule {
    /// The robot may not be sent in.
    #[default]
    KeepOut,
    /// The robot may go in, no faster than this.
    Slow(SpeedLimit),
}

/// How fast a slow zone lets the robot go.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpeedLimit {
    /// A share of the robot's top speed, in per cent.
    Percent(f64),
    /// Metres a second.
    Mps(f64),
}

impl SpeedLimit {
    /// As a share of the robot's top speed, in per cent; a speed in m/s needs the top speed.
    #[must_use]
    pub fn percent(self, top_mps: Option<f64>) -> Option<f64> {
        match self {
            Self::Percent(p) => Some(p),
            Self::Mps(v) => top_mps.filter(|t| *t > 0.0).map(|t| v / t * 100.0),
        }
    }

    /// As people say it: `30%`, `0.30 m/s`.
    #[must_use]
    pub fn words(self) -> String {
        match self {
            Self::Percent(p) => format!("{p:.0}%"),
            Self::Mps(v) => format!("{v:.2} m/s"),
        }
    }

    /// Why this is no speed a zone may set, if it is not: a share from `slowest` to 99 per cent,
    /// or a speed in m/s that is that share of `top_mps`.
    pub(crate) fn check(self, slowest: f64, top_mps: Option<f64>) -> Result<(), String> {
        let Some(p) = self.percent(top_mps).filter(|p| p.is_finite()) else {
            return Err(
                "this robot's profile gives no top speed (`[geofence] top_speed_mps`), so a slow \
                 zone takes a share of it in per cent, not m/s"
                    .to_owned(),
            );
        };
        if p < slowest || p > 99.0 {
            return Err(format!(
                "a slow zone lets the robot go {slowest:.0}% to 99% of its top speed, not {}; to \
                 keep the robot out, add the zone with no speed (a keep-out zone)",
                self.words()
            ));
        }
        Ok(())
    }
}

/// One zone.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Zone {
    /// What the operator calls it.
    pub name: String,
    /// Its corners in the map frame, metres, in order around it.
    pub polygon: Vec<[f64; 2]>,
    /// What it does to the robot.
    #[serde(default)]
    pub rule: Rule,
    /// Set in the profile: only an edit of the profile removes it.
    #[serde(default, skip_serializing)]
    pub fixed: bool,
}

impl Zone {
    /// Whether it keeps the robot out.
    #[must_use]
    pub fn keeps_out(&self) -> bool {
        self.rule == Rule::KeepOut
    }

    /// Whether a point is inside, by ray casting.
    #[must_use]
    pub fn contains(&self, x: f64, y: f64) -> bool {
        let p = &self.polygon;
        let mut inside = false;
        let mut j = p.len().wrapping_sub(1);
        for i in 0..p.len() {
            let ([xi, yi], [xj, yj]) = (p[i], p[j]);
            if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
                inside = !inside;
            }
            j = i;
        }
        inside
    }

    /// How far along a straight move from `from` to `to` it first enters this zone, none when it
    /// never does. A move that starts inside is the way out: it enters only where it comes back
    /// in, or at 0 when it never leaves.
    #[must_use]
    pub fn entered(&self, from: (f64, f64), to: (f64, f64)) -> Option<f64> {
        let (dx, dy) = (to.0 - from.0, to.1 - from.1);
        let corners = self.polygon.len();
        let mut crossings: Vec<f64> = (0..corners)
            .filter_map(|i| {
                let ([x1, y1], [x2, y2]) = (self.polygon[i], self.polygon[(i + 1) % corners]);
                let (ex, ey) = (x2 - x1, y2 - y1);
                let denom = dx * ey - dy * ex;
                if denom.abs() < f64::EPSILON {
                    return None;
                }
                // Where the move meets the edge, as shares of the move and of the edge.
                let along = ((x1 - from.0) * ey - (y1 - from.1) * ex) / denom;
                let on_edge = ((x1 - from.0) * dy - (y1 - from.1) * dx) / denom;
                ((0.0..=1.0).contains(&along) && (0.0..=1.0).contains(&on_edge)).then_some(along)
            })
            .collect();
        crossings.sort_by(f64::total_cmp);
        // Through a corner the move meets both its edges at once.
        crossings.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
        let length = dx.hypot(dy);
        if self.contains(from.0, from.1) {
            return match crossings.get(1) {
                Some(back_in) => Some(back_in * length),
                None => self.contains(to.0, to.1).then_some(0.0),
            };
        }
        crossings.first().map(|share| share * length)
    }

    /// Its area, by the shoelace formula.
    #[must_use]
    pub fn area(&self) -> f64 {
        let n = self.polygon.len();
        let twice: f64 = (0..n)
            .map(|i| {
                let ([x1, y1], [x2, y2]) = (self.polygon[i], self.polygon[(i + 1) % n]);
                x1 * y2 - x2 * y1
            })
            .sum();
        twice.abs() / 2.0
    }

    /// Why this is not a usable zone, if it is not; the profile's are held to it too.
    pub(crate) fn check(&self) -> Result<(), String> {
        if self.name.trim().is_empty() {
            return Err("a zone needs a name".to_owned());
        }
        if self.polygon.len() < 3 {
            return Err("a zone needs at least three corners".to_owned());
        }
        if self.polygon.iter().flatten().any(|v| !v.is_finite()) {
            return Err("a zone's corners must be numbers".to_owned());
        }
        if self.area() < MIN_AREA_M2 {
            return Err(format!(
                "a zone of {:.2} m² is too small to keep anything out",
                self.area()
            ));
        }
        Ok(())
    }
}

/// A square around `(x, y)`, `half` metres out each way.
fn square(x: f64, y: f64, half: f64) -> Vec<[f64; 2]> {
    vec![
        [x - half, y - half],
        [x + half, y - half],
        [x + half, y + half],
        [x - half, y + half],
    ]
}

/// The profile's zones and the operator's, and a counter every change moves on.
#[derive(Debug)]
pub struct Zones {
    fixed: Vec<Zone>,
    added: Mutex<Vec<Zone>>,
    file: Option<PathBuf>,
    frame: String,
    /// The slowest a slow zone may make the robot, in per cent of its top speed.
    slowest: f64,
    /// The robot's top speed, which a slow zone in m/s is a share of.
    top_mps: Option<f64>,
    version: watch::Sender<u64>,
}

impl Zones {
    /// The profile's zones, and the operator's remembered in `file` when it exists.
    #[must_use]
    pub fn new(profile: &Profile, file: Option<PathBuf>) -> Arc<Self> {
        let fixed = profile
            .geofence
            .zones
            .iter()
            .map(|z| Zone {
                name: z.name.clone(),
                polygon: z.polygon.clone(),
                rule: z.slow.map_or(Rule::KeepOut, Rule::Slow),
                fixed: true,
            })
            .collect();
        let added = file
            .as_deref()
            .map(crate::persist::read_or_default)
            .unwrap_or_default();
        Arc::new(Self {
            fixed,
            added: Mutex::new(added),
            file,
            frame: profile.ros.map_frame.clone(),
            slowest: profile.geofence.slowest_percent,
            top_mps: profile.geofence.top_speed_mps,
            version: watch::Sender::new(0),
        })
    }

    /// The robot's top speed, when the profile gives it.
    #[must_use]
    pub fn top_speed_mps(&self) -> Option<f64> {
        self.top_mps
    }

    /// The keep-out zones: the profile's, then the operator's.
    #[must_use]
    pub fn keep_out(&self) -> Vec<Zone> {
        self.all().into_iter().filter(Zone::keeps_out).collect()
    }

    /// The slow zones a point is in, and the share of the top speed the slowest of them allows.
    #[must_use]
    pub fn slow_at(&self, x: f64, y: f64) -> Option<(Zone, f64)> {
        self.all()
            .into_iter()
            .filter(|z| z.contains(x, y))
            .filter_map(|z| match z.rule {
                Rule::Slow(limit) => limit.percent(self.top_mps).map(|p| (z, p)),
                Rule::KeepOut => None,
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
    }

    fn lock(&self) -> MutexGuard<'_, Vec<Zone>> {
        crate::lock(&self.added)
    }

    /// Every zone: the profile's, then the operator's.
    #[must_use]
    pub fn all(&self) -> Vec<Zone> {
        let mut all = self.fixed.clone();
        all.extend(self.lock().iter().cloned());
        all
    }

    /// The frame the zones are drawn in.
    #[must_use]
    pub fn frame(&self) -> &str {
        &self.frame
    }

    /// Moves on with every change, for what draws or publishes the zones.
    #[must_use]
    pub fn changes(&self) -> watch::Receiver<u64> {
        self.version.subscribe()
    }

    /// The keep-out zone a point is in, if any.
    #[must_use]
    pub fn at(&self, x: f64, y: f64) -> Option<Zone> {
        self.keep_out().into_iter().find(|z| z.contains(x, y))
    }

    /// The first keep-out zone a straight move from `a` to `b` enters, and how far along it does.
    #[must_use]
    pub fn crossed(&self, a: (f64, f64), b: (f64, f64)) -> Option<(Zone, f64)> {
        self.keep_out()
            .into_iter()
            .filter_map(|z| z.entered(a, b).map(|d| (z, d)))
            .min_by(|x, y| x.1.total_cmp(&y.1))
    }

    fn save(&self, added: &[Zone]) -> Result<(), String> {
        let Some(file) = &self.file else {
            return Ok(());
        };
        let text = serde_json::to_vec_pretty(added).map_err(|e| e.to_string())?;
        crate::persist::write_atomic(file, &text)
            .map_err(|e| format!("keeping the zones in {}: {e}", file.display()))
    }

    /// Adds a zone of the operator's.
    ///
    /// # Errors
    ///
    /// The zone is not a usable polygon, its name is taken, or it could not be kept.
    pub fn add(&self, zone: Zone) -> Result<(), String> {
        zone.check()?;
        if let Rule::Slow(limit) = zone.rule {
            limit.check(self.slowest, self.top_mps)?;
        }
        if self
            .all()
            .iter()
            .any(|z| z.name.eq_ignore_ascii_case(&zone.name))
        {
            return Err(format!("there is already a zone called `{}`", zone.name));
        }
        let mut added = self.lock();
        let mut next = added.clone();
        next.push(Zone {
            fixed: false,
            ..zone
        });
        // Kept first: a zone that could not be kept is not in force either.
        self.save(&next)?;
        *added = next;
        drop(added);
        self.version.send_modify(|v| *v += 1);
        Ok(())
    }

    /// Removes a zone of the operator's.
    ///
    /// # Errors
    ///
    /// No such zone, the profile sets it, or the change could not be kept.
    pub fn remove(&self, name: &str) -> Result<(), String> {
        self.removable(name)?;
        let mut added = self.lock();
        let mut next = added.clone();
        next.retain(|z| !z.name.eq_ignore_ascii_case(name));
        if next.len() == added.len() {
            return Err(format!("there is no zone called `{name}`"));
        }
        // Kept first: a removal that could not be kept leaves the zone in force.
        self.save(&next)?;
        *added = next;
        drop(added);
        self.version.send_modify(|v| *v += 1);
        Ok(())
    }

    /// Sets a slow zone of the operator's to `limit`; the speed it had.
    ///
    /// # Errors
    ///
    /// No such slow zone, the profile sets it, the speed is out of range, or the change could not
    /// be kept.
    pub fn set_speed(&self, name: &str, limit: SpeedLimit) -> Result<SpeedLimit, String> {
        self.removable(name)?;
        limit.check(self.slowest, self.top_mps)?;
        let mut added = self.lock();
        let mut next = added.clone();
        let Some(zone) = next.iter_mut().find(|z| z.name.eq_ignore_ascii_case(name)) else {
            return Err(format!("there is no zone called `{name}`"));
        };
        let Rule::Slow(was) = zone.rule else {
            return Err(format!(
                "`{name}` keeps the robot out; only a slow zone has a speed"
            ));
        };
        zone.rule = Rule::Slow(limit);
        self.save(&next)?;
        *added = next;
        drop(added);
        self.version.send_modify(|v| *v += 1);
        Ok(was)
    }

    /// The zone called `name`, if any.
    #[must_use]
    pub fn named(&self, name: &str) -> Option<Zone> {
        self.all()
            .into_iter()
            .find(|z| z.name.eq_ignore_ascii_case(name))
    }

    /// Why the zone `name` cannot be removed, if it cannot: none has that name, or the profile
    /// sets it.
    ///
    /// # Errors
    ///
    /// The reason, for the model.
    pub fn removable(&self, name: &str) -> Result<(), String> {
        if self.fixed.iter().any(|z| z.name.eq_ignore_ascii_case(name)) {
            return Err(format!(
                "`{name}` is set in the robot's profile; only an edit of the profile removes it"
            ));
        }
        if self
            .lock()
            .iter()
            .any(|z| z.name.eq_ignore_ascii_case(name))
        {
            Ok(())
        } else {
            let names: Vec<String> = self.all().into_iter().map(|z| z.name).collect();
            Err(format!(
                "there is no zone called `{name}`; the zones are: {}",
                if names.is_empty() {
                    "none".to_owned()
                } else {
                    names.join(", ")
                }
            ))
        }
    }
}

/// `geofence`: keep-out and slow zones, listed, added, slowed and removed in chat.
pub(crate) struct GeofenceTool {
    spec: ToolSpec,
    zones: Arc<Zones>,
    places: Arc<Places>,
    robot: Arc<dyn RobotPort>,
    profile: Profile,
}

impl GeofenceTool {
    /// The tool over the robot's zones.
    #[must_use]
    pub(crate) fn new(
        profile: &Profile,
        zones: Arc<Zones>,
        places: Arc<Places>,
        robot: Arc<dyn RobotPort>,
    ) -> Self {
        let spec = ToolSpec::new(
            "geofence",
            "Zones on the map. A keep-out zone the robot may not be sent into, and Nav2 keeps its \
             paths out of. A slow zone the robot may enter no faster than slow_percent of its top \
             speed (or slow_mps), and Nav2 slows down in. list them; add one around a room (id or \
             name), an object (id), a place (name) or a polygon of map corners [[x, y], ...]: a \
             keep-out zone, or a slow zone when slow_percent or slow_mps is given above 0; speed \
             changes a slow zone's speed; remove one the operator added. Removing a zone or \
             speeding one up needs the operator's approval; a zone the profile sets cannot be \
             changed here.",
            json!({
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["list", "add", "speed", "remove"]},
                    "name": {"type": "string", "description": "The zone's name; for add, defaults to what it is around"},
                    "room": {"type": "string"},
                    "object": {"type": "string"},
                    "place": {"type": "string"},
                    "polygon": {"type": "array", "items": {"type": "array", "items": {"type": "number"}}},
                    "margin_m": {"type": "number", "description": "Kept clear around an object or a place; default 0.3"},
                    "slow_percent": {"type": "number", "description": "A slow zone: how fast the robot may go in it, in per cent of its top speed"},
                    "slow_mps": {"type": "number", "description": "A slow zone: how fast the robot may go in it, m/s"},
                },
                "required": ["action"],
            }),
            Risk::Observe,
        );
        Self {
            spec,
            zones,
            places,
            robot,
            profile: profile.clone(),
        }
    }

    async fn latest(&self, topic: Option<&crate::profile::TopicRef>) -> Value {
        match topic {
            Some(t) => self
                .robot
                .latest(&t.topic, &t.msg_type, WORLD_WAIT)
                .await
                .unwrap_or(Value::Null),
            None => Value::Null,
        }
    }

    /// A room's outline from the world model, and its name.
    async fn room(&self, room: &str) -> Result<(String, Vec<[f64; 2]>), String> {
        let topic = self.profile.world.as_ref().and_then(|w| w.rooms.as_ref());
        let rooms = self.latest(topic).await;
        let rooms: Vec<&Value> = rooms["rooms"].as_array().into_iter().flatten().collect();
        // By id, else name, else type; a type several rooms share must not fence only one.
        let tier = ["id", "name", "type"].iter().find_map(|key| {
            let hits: Vec<&Value> = rooms
                .iter()
                .copied()
                .filter(|r| {
                    r[key]
                        .as_str()
                        .is_some_and(|v| crate::places::same_words(v, room))
                })
                .collect();
            (!hits.is_empty()).then_some(hits)
        });
        let found = match tier.as_deref() {
            Some([one]) => *one,
            Some(many) => {
                let ids: Vec<&str> = many.iter().filter_map(|r| r["id"].as_str()).collect();
                return Err(format!(
                    "`{room}` is {} rooms ({}): add each by its id, with a name of its own",
                    many.len(),
                    ids.join(", ")
                ));
            }
            None => {
                return Err(format!(
                    "the world model has no room `{room}`; list_places names them"
                ));
            }
        };
        let polygon = found["outline"]["points"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|p| Some([p["x"].as_f64()?, p["y"].as_f64()?]))
            .collect();
        let name = crate::places::room_label(found).unwrap_or(room);
        Ok((name.to_owned(), polygon))
    }

    /// A square about an object from the world model, `margin` clear of it, and its label.
    async fn object(&self, id: &str, margin: f64) -> Result<(String, Vec<[f64; 2]>), String> {
        let topic = self.profile.world.as_ref().and_then(|w| w.objects.as_ref());
        let objects = self.latest(topic).await;
        let found = objects["objects"]
            .as_array()
            .into_iter()
            .flatten()
            .find(|o| o["id"].as_str() == Some(id));
        let Some(found) = found else {
            return Err(format!(
                "the world model has no object `{id}`; find_objects gives ids"
            ));
        };
        let p = &found["pose"]["position"];
        let (Some(x), Some(y)) = (p["x"].as_f64(), p["y"].as_f64()) else {
            return Err(format!("`{id}` has no position in the world model"));
        };
        // Wide enough for any turn of the object's box.
        let size = &found["size"];
        let half = size["x"]
            .as_f64()
            .unwrap_or(0.0)
            .hypot(size["y"].as_f64().unwrap_or(0.0))
            / 2.0;
        let label = found["label"]
            .as_str()
            .filter(|l| !l.is_empty())
            .unwrap_or(id);
        Ok((label.to_owned(), square(x, y, half + margin)))
    }

    /// The speed `args` ask for, if they ask for one.
    fn speed_in(args: &Value) -> Option<SpeedLimit> {
        args["slow_percent"]
            .as_f64()
            .map(SpeedLimit::Percent)
            .or_else(|| args["slow_mps"].as_f64().map(SpeedLimit::Mps))
    }

    /// The zone `args` ask for, with its name, or why there is none.
    async fn zone_for(&self, args: &Value) -> Result<Zone, String> {
        let margin = args["margin_m"]
            .as_f64()
            .filter(|m| m.is_finite() && *m >= 0.0)
            .unwrap_or(MARGIN_M);
        let text = |key: &str| args[key].as_str().filter(|v| !v.is_empty());
        let (default, polygon) = if let Some(room) = text("room") {
            self.room(room).await?
        } else if let Some(id) = text("object") {
            self.object(id, margin).await?
        } else if let Some(place) = text("place") {
            let same = |a: &str| a.eq_ignore_ascii_case(place);
            let Some(p) = self
                .places
                .all()
                .into_iter()
                .find(|p| same(&p.name) || p.aliases.iter().any(|a| same(a)))
            else {
                return Err(format!(
                    "`{place}` is not a known place; list_places names them"
                ));
            };
            (p.name, square(p.pose.x, p.pose.y, PLACE_HALF_M + margin))
        } else if let Some(points) = args["polygon"].as_array() {
            let polygon: Option<Vec<[f64; 2]>> = points
                .iter()
                .map(|p| Some([p.get(0)?.as_f64()?, p.get(1)?.as_f64()?]))
                .collect();
            let Some(polygon) = polygon else {
                return Err("polygon is a list of [x, y] map corners, in metres".to_owned());
            };
            ("zone".to_owned(), polygon)
        } else {
            return Err("add needs a room, an object, a place or a polygon".to_owned());
        };
        let name = text("name").map_or(default, |n| n.trim().to_owned());
        Ok(Zone {
            name,
            polygon,
            // No speed at all is a zone the robot may not enter: a model asks for 0% to mean it.
            rule: Self::speed_in(args)
                .filter(|l| match l {
                    SpeedLimit::Percent(v) | SpeedLimit::Mps(v) => *v > 0.0,
                })
                .map_or(Rule::KeepOut, Rule::Slow),
            fixed: false,
        })
    }

    async fn add(&self, args: &Value) -> ToolOutcome {
        let zone = match self.zone_for(args).await {
            Ok(z) => z,
            Err(why) => return ToolOutcome::failed(why),
        };
        // A keep-out zone around the robot would leave it nowhere to plan a way out; a slow one
        // only slows it on its way.
        let (map, base) = (&self.profile.ros.map_frame, &self.profile.ros.base_frame);
        if zone.keeps_out()
            && let Ok(t) = self.robot.transform(map, base)
            && zone.contains(t.translation[0], t.translation[1])
        {
            return ToolOutcome::refused(format!(
                "the robot stands inside `{}`: walk it out first, or it could not move at all",
                zone.name
            ));
        }
        let area = zone.area();
        let name = zone.name.clone();
        let message = match zone.rule {
            Rule::KeepOut => format!("added the keep-out zone `{name}`"),
            Rule::Slow(limit) => format!(
                "added the slow zone `{name}`: the robot goes at most {} in it",
                limit.words()
            ),
        };
        match self.zones.add(zone) {
            Ok(()) => ToolOutcome {
                message,
                ..ToolOutcome::ok(json!({"added": name, "area_m2": (area * 100.0).round() / 100.0}))
            },
            Err(why) => ToolOutcome::failed(why),
        }
    }

    /// The speed a slow zone has, and the speed `args` would give it.
    fn speeds(&self, args: &Value) -> Result<(SpeedLimit, SpeedLimit), String> {
        let name = args["name"].as_str().unwrap_or_default();
        let Some(zone) = self.zones.named(name) else {
            return Err(format!("there is no zone called `{name}`"));
        };
        let Rule::Slow(now) = zone.rule else {
            return Err(format!(
                "`{name}` keeps the robot out; only a slow zone has a speed"
            ));
        };
        let Some(next) = Self::speed_in(args) else {
            return Err("speed needs slow_percent or slow_mps".to_owned());
        };
        Ok((now, next))
    }

    /// Whether `next` lets the robot go faster than `now`; a speed in m/s against one in per cent
    /// counts as faster when the top speed that would compare them is unknown.
    fn faster(&self, now: SpeedLimit, next: SpeedLimit) -> bool {
        let top = self.zones.top_speed_mps();
        match (now.percent(top), next.percent(top)) {
            (Some(a), Some(b)) => b > a,
            _ => true,
        }
    }
}

#[async_trait]
impl Tool for GeofenceTool {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn assess(&self, args: &Value) -> Option<Result<Assessment, ToolOutcome>> {
        let name = args["name"].as_str().unwrap_or_default();
        let reason = match args["action"].as_str() {
            Some("remove") => {
                // Nothing to ask the operator about a zone that is not there, or that only they can
                // remove.
                if let Err(why) = self.zones.removable(name) {
                    return Some(Err(ToolOutcome::failed(why)));
                }
                match self.zones.named(name).map(|z| z.rule) {
                    Some(Rule::Slow(limit)) => format!(
                        "removes the slow zone `{name}`: the robot may go faster than {} there \
                         again",
                        limit.words()
                    ),
                    _ => {
                        format!("removes the keep-out zone `{name}`: the robot may go there again")
                    }
                }
            }
            Some("speed") => {
                if let Err(why) = self.zones.removable(name) {
                    return Some(Err(ToolOutcome::failed(why)));
                }
                let (now, next) = match self.speeds(args) {
                    Ok(s) => s,
                    Err(why) => return Some(Err(ToolOutcome::failed(why))),
                };
                // Slowing a zone only makes the robot more careful: free.
                if !self.faster(now, next) {
                    return None;
                }
                format!(
                    "lets the robot go faster in the slow zone `{name}`: {} instead of {}",
                    next.words(),
                    now.words()
                )
            }
            _ => return None,
        };
        Some(Ok(Assessment {
            risk: Risk::Annotate,
            resources: Vec::new(),
            reason,
            args: None,
        }))
    }

    fn always_asks(&self, args: &Value) -> bool {
        // A keep-out zone's removal is asked every time; a slow zone's may be allowed for the
        // session.
        args["action"].as_str() == Some("remove")
            && self
                .zones
                .named(args["name"].as_str().unwrap_or_default())
                .is_none_or(|z| z.keeps_out())
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        match args["action"].as_str().unwrap_or("list") {
            "list" => {
                let zones: Vec<Value> = self
                    .zones
                    .all()
                    .iter()
                    .map(|z| {
                        let mut out = json!({
                            "name": z.name,
                            "kind": if z.keeps_out() { "keep_out" } else { "slow" },
                            "set_in": if z.fixed { "profile" } else { "chat" },
                            "area_m2": (z.area() * 100.0).round() / 100.0,
                            "polygon": z.polygon,
                        });
                        if let Rule::Slow(limit) = z.rule {
                            out["speed"] = json!(limit.words());
                        }
                        out
                    })
                    .collect();
                ToolOutcome::ok(json!({"frame": self.zones.frame(), "zones": zones}))
            }
            "add" => self.add(&args).await,
            "speed" => {
                let name = args["name"].as_str().unwrap_or_default();
                let next = match self.speeds(&args) {
                    Ok((_, next)) => next,
                    Err(why) => return ToolOutcome::failed(why),
                };
                match self.zones.set_speed(name, next) {
                    Ok(was) => ToolOutcome {
                        message: format!(
                            "the slow zone `{name}` now lets the robot go {} (was {})",
                            next.words(),
                            was.words()
                        ),
                        ..ToolOutcome::ok(json!({"zone": name, "speed": next.words()}))
                    },
                    Err(why) => ToolOutcome::failed(why),
                }
            }
            "remove" => {
                let name = args["name"].as_str().unwrap_or_default();
                let kind = match self.zones.named(name).map(|z| z.rule) {
                    Some(Rule::Slow(_)) => "slow",
                    _ => "keep-out",
                };
                match self.zones.remove(name) {
                    Ok(()) => ToolOutcome {
                        message: format!("removed the {kind} zone `{name}`"),
                        ..ToolOutcome::ok(json!({"removed": name}))
                    },
                    Err(why) => ToolOutcome::failed(why),
                }
            }
            other => ToolOutcome::failed(format!(
                "action `{other}` is not one of list, add, speed, remove"
            )),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn zone(name: &str, polygon: &[[f64; 2]]) -> Zone {
        Zone {
            name: name.into(),
            polygon: polygon.to_vec(),
            rule: Rule::KeepOut,
            fixed: false,
        }
    }

    fn l_shape() -> Zone {
        // An L: concave, as a room often is.
        zone(
            "l",
            &[
                [0.0, 0.0],
                [4.0, 0.0],
                [4.0, 1.0],
                [1.0, 1.0],
                [1.0, 4.0],
                [0.0, 4.0],
            ],
        )
    }

    #[test]
    fn a_point_is_inside_a_concave_zone_only_where_it_is() {
        let l = l_shape();
        assert!(l.contains(0.5, 3.0));
        assert!(l.contains(3.5, 0.5));
        assert!(!l.contains(2.0, 2.0), "the L's notch is outside");
        assert!((l.area() - 7.0).abs() < 1e-9);
    }

    #[test]
    fn a_walk_enters_a_zone_where_it_crosses_its_edge() {
        let box_ = zone("box", &[[2.0, -1.0], [3.0, -1.0], [3.0, 1.0], [2.0, 1.0]]);
        assert_eq!(box_.entered((0.0, 0.0), (1.5, 0.0)), None, "stops short");
        let d = box_.entered((0.0, 0.0), (5.0, 0.0)).unwrap();
        assert!((d - 2.0).abs() < 1e-9, "{d}");
        assert_eq!(box_.entered((2.5, 0.0), (5.0, 0.0)), None, "walks out");
        assert_eq!(box_.entered((2.5, 0.0), (2.8, 0.0)), Some(0.0), "stays in");
        // Out of one arm of the L, across its notch, and back in through the other.
        let back_in = l_shape().entered((0.5, 3.5), (3.5, 0.5)).unwrap();
        assert!((back_in - 2.5 * 2_f64.sqrt()).abs() < 1e-9, "{back_in}");
    }

    fn tool_on(robot: nervros_ros::fake::FakeRobot) -> (GeofenceTool, Arc<Zones>) {
        let profile: Profile = toml::from_str(
            "[robot]\nname = \"x\"\n[models]\nfile = \"m.toml\"\n\
             [world]\nrooms = { topic = \"/rooms\", type = \"canopy_msgs/msg/RoomArray\" }\n\
             objects = { topic = \"/objects\", type = \"canopy_msgs/msg/WorldObjectArray\" }\n\
             [[place]]\nname = \"dock\"\npose = { x = 8.0, y = 8.0 }\n",
        )
        .unwrap();
        let zones = Zones::new(&profile, None);
        let places = Places::new(&profile, None);
        let tool = GeofenceTool::new(&profile, Arc::clone(&zones), places, Arc::new(robot));
        (tool, zones)
    }

    fn flat() -> nervros_ros::fake::FakeRobot {
        let square = |x: f64, y: f64| json!([{"x": x, "y": y}, {"x": x + 4.0, "y": y}, {"x": x + 4.0, "y": y + 4.0}, {"x": x, "y": y + 4.0}]);
        nervros_ros::fake::FakeRobot::new()
            .with_topic(
                "/rooms",
                json!({"rooms": [
                    {"id": "R1", "name": "living room", "outline": {"points": square(0.0, 0.0)}},
                    {"id": "R2", "name": "kitchen", "outline": {"points": square(4.0, 0.0)}},
                    {"id": "R3", "name": "room C", "type": "bedroom", "outline": {"points": square(0.0, 10.0)}},
                ]}),
            )
            .with_topic(
                "/objects",
                json!({"objects": [{"id": "O17", "label": "red mug", "pose": {"position": {"x": 5.0, "y": 1.0}}, "size": {"x": 0.1, "y": 0.1}}]}),
            )
            .with_transform("map", "base_footprint", nervros_ros::Transform::IDENTITY)
    }

    #[tokio::test]
    async fn a_zone_is_added_around_a_room_an_object_or_a_place() {
        let (tool, zones) = tool_on(flat());
        let kitchen = tool.call(json!({"action": "add", "room": "kitchen"})).await;
        assert_eq!(kitchen.data["added"], "kitchen", "{}", kitchen.message);
        assert!(zones.at(6.0, 2.0).is_some());
        let mug = tool
            .call(json!({"action": "add", "object": "O17", "name": "mug", "margin_m": 0.2}))
            .await;
        assert_eq!(mug.data["added"], "mug", "{}", mug.message);
        // A slow zone at no speed is how a model asks to keep the robot out.
        let dock = tool
            .call(json!({"action": "add", "place": "dock", "slow_percent": 0}))
            .await;
        assert_eq!(dock.message, "added the keep-out zone `dock`");
        assert!(
            zones.at(8.6, 8.6).is_some(),
            "the place's square with its margin"
        );
        // canopy's "room C" says nothing; the zone takes what the room is.
        let bedroom = tool.call(json!({"action": "add", "room": "bedroom"})).await;
        assert_eq!(bedroom.data["added"], "bedroom", "{}", bedroom.message);
        let listed = tool.call(json!({"action": "list"})).await;
        assert_eq!(listed.data["zones"].as_array().unwrap().len(), 4);
        let removed = tool
            .call(json!({"action": "remove", "name": "bedroom"}))
            .await;
        assert_eq!(removed.message, "removed the keep-out zone `bedroom`");
    }

    #[tokio::test]
    async fn a_zone_around_the_robot_is_refused_and_removing_one_needs_approval() {
        let (tool, zones) = tool_on(flat());
        // The robot stands at the origin, in the living room.
        let refused = tool.call(json!({"action": "add", "room": "R1"})).await;
        assert!(
            refused.message.contains("stands inside"),
            "{}",
            refused.message
        );
        assert!(zones.all().is_empty());
        // A zone that is not there is refused before the operator is asked anything.
        let missing = tool
            .assess(&json!({"action": "remove", "name": "kitchen"}))
            .await
            .unwrap()
            .unwrap_err();
        assert!(
            missing.message.contains("no zone called `kitchen`"),
            "{}",
            missing.message
        );
        tool.call(json!({"action": "add", "room": "kitchen"})).await;
        let assessed = tool
            .assess(&json!({"action": "remove", "name": "kitchen"}))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(assessed.risk, Risk::Annotate);
        assert!(tool.always_asks(&json!({"action": "remove", "name": "kitchen"})));
        assert!(
            tool.assess(&json!({"action": "add", "room": "R2"}))
                .await
                .is_none()
        );
        assert!(!tool.always_asks(&json!({"action": "add", "room": "R2"})));
    }

    #[tokio::test]
    async fn a_type_several_rooms_share_is_refused_until_each_is_named() {
        let square = |x: f64| json!([{"x": x, "y": 10.0}, {"x": x + 4.0, "y": 10.0}, {"x": x + 4.0, "y": 14.0}, {"x": x, "y": 14.0}]);
        let robot = nervros_ros::fake::FakeRobot::new()
            .with_topic(
                "/rooms",
                json!({"rooms": [
                    {"id": "R3", "name": "room C", "type": "bedroom", "outline": {"points": square(0.0)}},
                    {"id": "R4", "name": "room D", "type": "bedroom", "outline": {"points": square(5.0)}},
                ]}),
            )
            .with_transform("map", "base_footprint", nervros_ros::Transform::IDENTITY);
        let (tool, zones) = tool_on(robot);
        let both = tool.call(json!({"action": "add", "room": "bedroom"})).await;
        assert!(
            both.message.contains("2 rooms (R3, R4)"),
            "{}",
            both.message
        );
        assert!(zones.all().is_empty());
        let one = tool
            .call(json!({"action": "add", "room": "R4", "name": "guest bedroom"}))
            .await;
        assert_eq!(one.data["added"], "guest bedroom", "{}", one.message);
    }

    #[test]
    fn zones_are_kept_and_only_the_operators_removed() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("zones.json");
        let profile: Profile = toml::from_str(
            "[robot]\nname = \"x\"\n[models]\nfile = \"m.toml\"\n\
             [[geofence.zone]]\nname = \"stairs\"\npolygon = [[0, 0], [1, 0], [1, 1], [0, 1]]\n",
        )
        .unwrap();
        let zones = Zones::new(&profile, Some(file.clone()));
        let seen = zones.changes();
        zones
            .add(zone("kitchen", &[[5.0, 5.0], [6.0, 5.0], [6.0, 6.0]]))
            .unwrap();
        assert!(seen.has_changed().unwrap());
        assert!(
            zones
                .add(zone("Kitchen", &[[7.0, 7.0], [8.0, 7.0], [8.0, 8.0]]))
                .is_err()
        );
        assert!(
            zones
                .add(zone("tiny", &[[0.0, 0.0], [0.1, 0.0], [0.1, 0.1]]))
                .is_err()
        );
        assert!(zones.remove("stairs").unwrap_err().contains("profile"));
        // A change that cannot be kept is not made.
        let unkept = Zones::new(
            &profile,
            Some(dir.path().join("missing").join("zones.json")),
        );
        std::fs::write(dir.path().join("missing"), b"a file, not a folder").unwrap();
        let before = unkept.changes();
        assert!(
            unkept
                .add(zone("hall", &[[5.0, 5.0], [6.0, 5.0], [6.0, 6.0]]))
                .is_err()
        );
        assert_eq!(unkept.all().len(), 1);
        assert!(!before.has_changed().unwrap());
        // A new session finds the operator's zone, and the profile's.
        let again = Zones::new(&profile, Some(file));
        let names: Vec<String> = again.all().into_iter().map(|z| z.name).collect();
        assert_eq!(names, ["stairs", "kitchen"]);
        again.remove("kitchen").unwrap();
        assert_eq!(again.all().len(), 1);
        assert_eq!(again.at(0.5, 0.5).map(|z| z.name), Some("stairs".into()));
        let (hit, d) = again.crossed((-1.0, 0.5), (3.0, 0.5)).unwrap();
        assert_eq!(hit.name, "stairs");
        assert!((d - 1.0).abs() < 1e-9);
    }
}

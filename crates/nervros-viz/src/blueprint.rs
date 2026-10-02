//! The viewer's layout, logged as blueprint data by hand: `re_sdk`'s layout builder cannot set a
//! view's properties from Rust, and the panes need theirs (the 3D view's eye, background and
//! grid, the camera views' background, the log's columns).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Mutex, PoisonError};

use rerun::blueprint::components::{
    ActiveTab, AutoLayout, AutoViews, BackgroundKind, ColumnShare, ContainerKind, Eye3DKind,
    IncludedContent, QueryExpression, RootContainer, RowShare, TextLogColumn, TimelineColumn,
    ViewClass,
};
use rerun::blueprint::encodings::{self, TextLogColumnKind};
use rerun::external::re_log_types::{BlueprintActivationCommand, RecordingId};
use rerun::external::re_sdk_types::SerializedComponentBatch;
use rerun::external::re_sdk_types::blueprint::archetypes::{
    Background, ContainerBlueprint, EyeControls3D, LineGrid3D, TextLogColumns, ViewBlueprint,
    ViewContents, ViewportBlueprint,
};
use rerun::{AsComponents, RecordingStream, RecordingStreamBuilder, RecordingStreamResult};

use crate::{CAMERA_PATH, FOLLOW_PATH};

/// Behind the 3D world: a shade above the app's panels and below the map's floor, so the
/// building stands out from what is around it.
const WORLD_BACKGROUND: [u8; 3] = [0x1c, 0x21, 0x2a];
/// Around a camera frame: the app's panel colour, so the bands beside a frame of another aspect
/// read as the panel.
const CAMERA_BACKGROUND: [u8; 3] = [0x12, 0x15, 0x1b];
/// How steeply the 3D view looks down on the map.
const PITCH_DEG: f32 = 52.0;
/// How steeply it looks down on the robot it follows: lower, so the robot reads as a body.
const FOLLOW_PITCH_DEG: f32 = 35.0;

/// A pane, or panes side by side, stacked or in tabs.
enum Pane {
    View {
        class: &'static str,
        name: String,
        origin: String,
        properties: Vec<(&'static str, Vec<SerializedComponentBatch>)>,
    },
    Split {
        kind: ContainerKind,
        panes: Vec<Self>,
        shares: Vec<f32>,
        /// The tab shown first, for tabs.
        active: Option<usize>,
    },
}

impl Pane {
    fn view(class: &'static str, name: &str, origin: &str) -> Self {
        Self::View {
            class,
            name: name.to_owned(),
            origin: origin.to_owned(),
            properties: Vec::new(),
        }
    }

    /// Sets a property of a view; the viewer reads each by its archetype's name.
    fn with(mut self, name: &'static str, property: &dyn AsComponents) -> Self {
        if let Self::View { properties, .. } = &mut self {
            properties.push((name, property.as_serialized_batches()));
        }
        self
    }

    fn split(kind: ContainerKind, panes: Vec<Self>, shares: &[f32]) -> Self {
        Self::Split {
            kind,
            panes,
            shares: shares.to_vec(),
            active: None,
        }
    }

    fn tabs(panes: Vec<Self>, active: usize) -> Self {
        Self::Split {
            kind: ContainerKind::Tabs,
            panes,
            shares: Vec::new(),
            active: Some(active),
        }
    }

    /// Logs the pane and what it holds; returns its blueprint path and id.
    fn log(&self, bp: &RecordingStream) -> RecordingStreamResult<(String, uuid::Uuid)> {
        let id = uuid::Uuid::new_v4();
        match self {
            Self::View {
                class,
                name,
                origin,
                properties,
            } => {
                let path = format!("view/{id}");
                bp.log(
                    format!("{path}/ViewContents"),
                    &ViewContents::new([QueryExpression("$origin/**".into())]),
                )?;
                let view = ViewBlueprint::new(ViewClass((*class).into()))
                    .with_display_name(name.as_str())
                    .with_space_origin(origin.as_str());
                bp.log(path.as_str(), &view)?;
                for (property, batches) in properties {
                    bp.log_serialized_batches(
                        format!("{path}/{property}"),
                        false,
                        batches.iter().cloned(),
                    )?;
                }
                Ok((path, id))
            }
            Self::Split {
                kind,
                panes,
                shares,
                active,
            } => {
                let paths = panes
                    .iter()
                    .map(|p| p.log(bp).map(|(path, _)| path))
                    .collect::<RecordingStreamResult<Vec<_>>>()?;
                let mut container = ContainerBlueprint::new(*kind)
                    .with_contents(paths.iter().map(|p| IncludedContent(p.as_str().into())));
                match kind {
                    ContainerKind::Horizontal => {
                        container = container
                            .with_col_shares(shares.iter().map(|&s| ColumnShare(s.into())));
                    }
                    ContainerKind::Vertical => {
                        container =
                            container.with_row_shares(shares.iter().map(|&s| RowShare(s.into())));
                    }
                    _ => {}
                }
                if let Some(tab) = active.and_then(|i| paths.get(i)) {
                    container = container.with_active_tab(ActiveTab(tab.as_str().into()));
                }
                let path = format!("container/{id}");
                bp.log(path.as_str(), &container)?;
                Ok((path, id))
            }
        }
    }
}

/// The layout NervROS gives the viewer, and what it frames the 3D view on.
pub(super) struct Layout {
    rec: RecordingStream,
    /// Each camera's name and entity path.
    cameras: Vec<(String, String)>,
    /// The map's known extent, `[x0, y0, x1, y1]` in the map frame, once a map arrived.
    bounds: Mutex<Option<[f32; 4]>>,
    /// The 3D view keeps to the robot.
    follow: AtomicBool,
}

impl Layout {
    pub(super) fn new(rec: RecordingStream, cameras: Vec<(String, String)>) -> Self {
        Self {
            rec,
            cameras,
            bounds: Mutex::new(None),
            follow: AtomicBool::new(false),
        }
    }

    /// Sends the layout and makes it active, not only the default: the viewer would otherwise
    /// restore the last session's, closed panes and an older version's layout included.
    pub(super) fn send(&self) {
        let bounds = *self.bounds.lock().unwrap_or_else(PoisonError::into_inner);
        let root = panes(&self.cameras, bounds, self.following());
        if let Err(e) = send(&self.rec, &root) {
            tracing::debug!(error = %e, "the viewer layout was not sent");
        }
    }

    pub(super) fn follow(&self, on: bool) {
        self.follow.store(on, Ordering::Relaxed);
        self.send();
    }

    pub(super) fn following(&self) -> bool {
        self.follow.load(Ordering::Relaxed)
    }

    /// Frames the 3D view on the first map; a later map leaves the operator's view alone until
    /// Reset layout.
    pub(super) fn map(&self, extent: [f32; 4]) {
        let first = self
            .bounds
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .replace(extent)
            .is_none();
        if first {
            self.send();
        }
    }
}

/// The world wide, as a building's plan is; under it the cameras and the agent's last marked
/// image side by side; below, the mission's steps, the agent's log and the plots, in tabs as wide
/// as the viewer so their columns read.
fn panes(cameras: &[(String, String)], bounds: Option<[f32; 4]>, follow: bool) -> Pane {
    let camera = |name: &str, path: &str| {
        let background =
            Background::new(BackgroundKind::SolidColor).with_color(rgb(CAMERA_BACKGROUND));
        Pane::view("2D", name, path).with("Background", &background)
    };
    let mut row: Vec<Pane> = match cameras {
        [] => vec![camera("Camera", CAMERA_PATH[0])],
        [(_, path)] => vec![camera("Camera", path)],
        many => many
            .iter()
            .map(|(name, path)| camera(&capitalised(name), path))
            .collect(),
    };
    row.push(camera("Last look", "/agent/look"));
    let shares = vec![1.0; row.len()];
    let mut world = Pane::view("3D", "World", "/world")
        .with(
            "Background",
            &Background::new(BackgroundKind::SolidColor).with_color(rgb(WORLD_BACKGROUND)),
        )
        .with(
            "LineGrid3D",
            // A metre grid a centimetre under the map, so the two never fight for a pixel.
            &LineGrid3D::new()
                .with_visible(true)
                .with_spacing(1.0)
                .with_plane(rerun::components::Plane3D::new([0.0, 0.0, 1.0], -0.01))
                .with_stroke_width(1.0)
                .with_color(rerun::Color::from_unmultiplied_rgba(255, 255, 255, 14)),
        );
    // Following, the viewer moves the eye with the robot and keeps only its direction: the
    // map's, or a default one before a map arrives.
    match (bounds, follow) {
        (Some(bounds), false) => world = world.with("EyeControls3D", &eye(bounds, PITCH_DEG)),
        (bounds, true) => {
            let bounds = bounds.unwrap_or([-1.0, -1.0, 1.0, 1.0]);
            let eye = eye(bounds, FOLLOW_PITCH_DEG).with_tracking_entity(format!("/{FOLLOW_PATH}"));
            world = world.with("EyeControls3D", &eye);
        }
        (None, false) => {}
    }
    let cameras = Pane::split(ContainerKind::Horizontal, row, &shares);
    let log_columns = TextLogColumns::new()
        .with_timeline_columns([TimelineColumn(encodings::TimelineColumn {
            visible: false.into(),
            timeline: "log_time".into(),
        })])
        .with_text_log_columns(
            [
                (TextLogColumnKind::EntityPath, false),
                (TextLogColumnKind::LogLevel, true),
                (TextLogColumnKind::Body, true),
            ]
            .map(|(kind, visible)| {
                TextLogColumn(encodings::TextLogColumn {
                    visible: visible.into(),
                    kind,
                })
            }),
        );
    let strip = Pane::tabs(
        vec![
            Pane::view("StateTimeline", "Mission", "/mission"),
            Pane::view("TextLog", "Agent", "/agent/log").with("TextLogColumns", &log_columns),
            Pane::view("TimeSeries", "Exploring", "/mapping"),
            Pane::view("TimeSeries", "Plots", "/plots"),
        ],
        0,
    );
    Pane::split(
        ContainerKind::Vertical,
        vec![world, cameras, strip],
        &[3.0, 1.1, 1.0],
    )
}

/// Where the eye stands and what it looks at, over the map's known extent: south of it and
/// steeply above, so the map reads as a plan with x to the right, and far enough back that all
/// of it is in view.
fn framing([x0, y0, x1, y1]: [f32; 4], pitch_deg: f32) -> ([f32; 3], [f32; 3]) {
    let target = [f32::midpoint(x0, x1), f32::midpoint(y0, y1), 0.0];
    let reach = 0.5 * (x1 - x0).hypot(y1 - y0);
    // The pane is wide: the map's width fits at a little over its half-diagonal.
    let distance = (1.3 * reach).max(4.0);
    let pitch = pitch_deg.to_radians();
    let position = [
        target[0],
        target[1] - distance * pitch.cos(),
        distance * pitch.sin(),
    ];
    (position, target)
}

fn eye(bounds: [f32; 4], pitch_deg: f32) -> EyeControls3D {
    let (position, target) = framing(bounds, pitch_deg);
    EyeControls3D::new()
        .with_kind(Eye3DKind::Orbital)
        .with_position(position)
        .with_look_target(target)
        .with_eye_up([0.0, 0.0, 1.0])
}

/// Logs `root` into a blueprint of its own and sends it, active.
fn send(rec: &RecordingStream, root: &Pane) -> RecordingStreamResult<()> {
    let app = rec
        .store_info()
        .map_or_else(|| "nervros".into(), |info| info.application_id().clone());
    let (bp, storage) = RecordingStreamBuilder::new(app)
        .recording_id(RecordingId::random())
        .blueprint()
        .memory()?;
    // The viewer reads blueprint data on this timeline.
    bp.set_time_sequence("blueprint", 0);
    let (_, root) = root.log(&bp)?;
    bp.log(
        "viewport",
        &ViewportBlueprint::new()
            .with_root_container(RootContainer(root.into()))
            .with_auto_layout(AutoLayout(false.into()))
            .with_auto_views(AutoViews(false.into())),
    )?;
    let msgs = storage.take();
    let Some(blueprint_id) = msgs.first().map(|m| m.store_id().clone()) else {
        return Ok(());
    };
    rec.send_blueprint(
        msgs,
        BlueprintActivationCommand {
            blueprint_id,
            make_active: true,
            make_default: true,
        },
    );
    Ok(())
}

fn rgb([r, g, b]: [u8; 3]) -> rerun::Color {
    rerun::Color::from_rgb(r, g, b)
}

fn capitalised(name: &str) -> String {
    let mut c = name.chars();
    c.next()
        .map_or_else(String::new, |f| f.to_uppercase().chain(c).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_eye_looks_down_on_the_middle_of_the_map_from_the_south() {
        let ([x, y, z], target) = framing([-2.0, -1.0, 10.0, 7.0], PITCH_DEG);
        let near = |a: f32, b: f32| (a - b).abs() < 1e-4;
        assert!(near(target[0], 4.0) && near(target[1], 3.0) && near(target[2], 0.0));
        assert!(near(x, 4.0), "straight from the south");
        assert!(y < -1.0, "standing beyond the map's south edge");
        let pitch = z.atan2(target[1] - y).to_degrees();
        assert!((pitch - PITCH_DEG).abs() < 0.01);
        let reach = 0.5 * 12.0f32.hypot(8.0);
        assert!(
            z.hypot(target[1] - y) >= reach,
            "far enough back to see all of it"
        );
        let ([.., z_small], _) = framing([0.0, 0.0, 1.0, 1.0], PITCH_DEG);
        assert!(
            z_small > 3.0,
            "a small map is not seen from a hand's breadth"
        );
    }

    #[test]
    fn following_tracks_the_robot_and_the_overview_waits_for_a_map() {
        // What the world view's eye says: none, a fixed eye, or one that tracks.
        let eye_of = |root: &Pane| {
            let Pane::Split { panes, .. } = root else {
                return None;
            };
            let Pane::View { properties, .. } = &panes[0] else {
                return None;
            };
            let (_, batches) = properties.iter().find(|(n, _)| *n == "EyeControls3D")?;
            let tracking = EyeControls3D::descriptor_tracking_entity().component;
            Some(batches.iter().any(|b| b.descriptor.component == tracking))
        };
        assert_eq!(
            eye_of(&panes(&[], None, false)),
            None,
            "no map: the viewer frames what it has"
        );
        assert_eq!(
            eye_of(&panes(&[], Some([0.0, 0.0, 4.0, 3.0]), false)),
            Some(false)
        );
        assert_eq!(
            eye_of(&panes(&[], None, true)),
            Some(true),
            "following needs no map"
        );
    }

    #[test]
    fn every_camera_gets_a_pane_and_the_strip_opens_on_the_mission() {
        let cameras = [
            ("chest".to_owned(), "/camera".to_owned()),
            ("head".to_owned(), "/cameras/head".to_owned()),
        ];
        let Pane::Split { panes, .. } = panes(&cameras, None, false) else {
            panic!("the root is a split");
        };
        let Pane::Split { panes: row, .. } = &panes[1] else {
            panic!("the cameras are side by side");
        };
        let names: Vec<&str> = row
            .iter()
            .map(|p| match p {
                Pane::View { name, .. } => name.as_str(),
                Pane::Split { .. } => "",
            })
            .collect();
        assert_eq!(names, ["Chest", "Head", "Last look"]);
        let Pane::Split { active, .. } = &panes[2] else {
            panic!("the strip is tabs");
        };
        assert_eq!(*active, Some(0));
    }
}

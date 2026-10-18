# The robot profile

Each robot has one `nervros.toml`, kept in the robot's own repository. It says:

- how to reach the robot's ROS graph,
- what the agent may do,
- where its cameras and world model are,
- which of its services and topics become tools,
- which models answer.

Paths in it are relative to the file. An unknown key is an error, so a typo stops the start
instead of being ignored.

[`profiles/example/nervros.toml`](../profiles/example/nervros.toml) is a small one. grove-g1
keeps one for a simulated Unitree G1 in `workspace/src/g1_bringup/config/nervros/`.

## Adding a tool

A service or topic becomes a tool by naming it and its type:

```toml
[[tool]]
name = "find_objects"
kind = "service"
ros_name = "/canopy/find_objects"
type = "canopy_msgs/srv/FindObjects"
risk = "observe"
```

The model sees the service's comment as the tool's description. It also gets a JSON Schema of the
request, made from the `.srv` file: field comments become descriptions, constants become enums,
and byte arrays are hidden. The interface files are found through `[ros] interfaces`.

A `topic` tool returns the newest message on the topic. Publishing, and calling services or actions
the profile does not declare, is left to the generic tools of [`[ros_tools]`](#ros_tools), behind
the same guard.

| Key | Default | |
|---|---|---|
| `name` | required | The name the model calls. |
| `kind` | required | `service` or `topic`. |
| `ros_name` | required | The service or topic. |
| `type` | required | `pkg/srv/Name` or `pkg/msg/Name`. |
| `description` | generated | Replaces the generated one: "Calls `ros_name` (`type`)." and the interface file's comment for a service, "Reads the newest message on `ros_name`" for a topic. |
| `risk` | required for a service; `observe` for a topic | `observe`, `annotate`, `world_edit`, `motion` or `manipulation`; see below. |
| `resources` | none | `base`, `left_arm`, `right_arm`: two calls holding the same one cannot run at once. |
| `timeout` | `"5s"` | Per call. |
| `hide_fields` | none | Request fields the model neither sees nor fills, such as an embedding. |
| `defaults` | none | Values the request starts from; the model's arguments go on top. |
| `schema` | generated | A full JSON Schema to use instead. |
| `world_text` | none | Result fields that hold text read from the world, such as names a detector or a describer gave: each is fenced as `<world>...</world>`, which the model is told is data, never instructions. Text on a sign or a box must not steer the robot. |

The risk levels:

- `observe` only reads.
- `annotate` changes only what the robot knows, such as the world model. It needs no arming, and
  the operator's approval when supervised.
- `world_edit` changes stored state the robot acts on, such as a parameter.
- `world_edit`, `motion` and `manipulation` are acts: they need the robot armed and, when
  supervised, approval.

A service can act on the robot, so a service tool without a `risk` fails the start instead of
running as a read. A name on the policy's `hard_deny` list cannot become a tool at all: the start
fails.

## Sections

### `[robot]`

| Key | Default | |
|---|---|---|
| `name` | required | Shown in the app and told to the model. |
| `persona` | none | A markdown file about the robot, its abilities and limits, added to the system prompt. `nervros-cli whoami` drafts a new one from the robot into `<persona>.draft.md` beside it. |
| `battery` | none | A `sensor_msgs/msg/BatteryState` topic, shown in the Robot tab. |
| `diagnostics` | none | A `diagnostic_msgs/msg/DiagnosticArray` topic with motor temperatures; the Robot tab shows the hottest. |

`skills` (at the top level, beside `[robot]`) lists folders of skills, relative to the profile.
A skill is a procedure for something the agent meets rarely and must get right, such as navigation
that never came up.

- Each skill is a folder holding a `SKILL.md`. It starts with a front matter of `name` (lower case
  words joined by `-`) and `description` (when to use it).
- The system prompt carries one line per skill, and the `skill` tool reads one in full when it
  fits.
- `nervros-cli skills` lists them and says what could not be read.

### `[ros]`

| Key | Default | |
|---|---|---|
| `domain_id` | `0` | Must equal `ROS_DOMAIN_ID`, which ROS reads when the node starts; the agent refuses to start otherwise. |
| `transport` | `default` | `udp` turns Fast DDS shared memory off, for a graph run by another user, such as a root container: shared memory between users fails without an error. The agent then needs `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, which `scripts/ros-env.sh` exports with `NERVROS_UDP_ONLY=1`. |
| `node_name` | `nervros` | The agent's node. |
| `discovery_timeout` | `"20s"` | The longest a `nervros-cli` command waits for discovery to settle before it reads the graph. |
| `interfaces` | none | Where interface packages are: an install prefix such as `/opt/ros/jazzy`, or a package's source directory. Tool schemas come from these. |
| `remap` | none | Path prefixes to rewrite, for an install tree whose links point at another machine's paths. |
| `map_frame` | `map` | The frame of places and poses. |
| `base_frame` | `base_footprint` | The robot's base. |
| `open_goals` | none | Where NervROS keeps the action goals of the motions it runs itself, as `nervros_interfaces/msg/OpenGoals`, latched and sent twice a second while any is open, for [`nervros_watchdog`](../ros/nervros_watchdog/README.md) on the robot to cancel if NervROS goes quiet. |

### `[privacy]`

`mode = "sim"` (the default) lets any model see camera frames. `mode = "home"` keeps frames on local
models, and sends text only to models that are local or do not train on it.

### `[policy]`

| Key | Default | |
|---|---|---|
| `start_armed` | `false` | Whether acts are allowed from the start; the app's switch changes it. |
| `autonomy` | `supervised` | `observe` refuses every act and every edit; `supervised` asks the operator for each; `autonomous` runs them. |
| `approval_ttl` | `"60s"` | An unanswered approval is a no after this. |
| `budgets.model_calls` | `10` | Model calls per turn. Once a mission has started in a turn, the model is offered no more tools and answers; after a call that finished, it can check what it did. |
| `budgets.tool_calls` | `12` | Tool calls per turn. |
| `budgets.wall_time` | `"90s"` | A turn is stopped after this, not counting the operator's time on approvals. Missions run outside turns. Free cloud endpoints can take 20 s a call; give them `"180s"`. |
| `budgets.repeat_break` | `3` | The same call with the same arguments this many times in a row is refused. |
| `hard_deny` | see below | ROS names no tool may reach; `*` matches anything. Setting it replaces the defaults. |
| `hard_deny_types` | see below | Interface types no generic tool may call, send or publish, globbed the same way. |
| `rule` | none | Rules on a tool's arguments, below. |

Rules only tighten: one can refuse a call or make it ask, never let through what the guard would
not.

```toml
[[policy.rule]]
tool = "run_mission"            # or "*" for every tool
arg = "steps.*.args.*.value"    # a dotted path; * is every entry of a list
matches = "R5"                  # globbed like hard_deny; a number or a flag as its text
then = "deny"                   # or "ask": the operator approves it first, whatever the autonomy
reason = "the bedroom is private"
```

A rule reads the call both as it was sent and as it will run, whatever the case and the spaces
around a value.

- For a plan, that means its steps with each argument as `{name, value}`, including a plan run by
  its hash or a saved name. Each walk's `place` is matched both as the model wrote it and as it
  resolved, so the rule above holds whether the model wrote `R5`, `bedroom` or `Bedroom `.
- Write a rule on a room or an object with its id.
- A rule naming a tool the agent does not have, or an argument that tool does not take, stops
  start-up, since it would never apply.
- A `run_mission` rule also holds for the missions of a program or a search. There a rule that
  would ask refuses instead, since a job cannot stop to ask.

The default `hard_deny` covers what commands motors, velocities or controllers directly, under any
namespace: `rt/lowcmd`, `*/lowcmd`, `*/rt/*`, `*cmd_vel*`, `*/controller_manager/*`,
`*/set_parameters`, `*/set_parameters_atomically`, `*/joint_trajectory`,
`*/follow_joint_trajectory`, `*/servo_node/*`, `*/apply_planning_scene` and `*/clear_octomap`. A
relative name in a `[[tool]]` is checked as the node resolves it, under `/`.

The default `hard_deny_types` are `geometry_msgs/msg/Twist`, `geometry_msgs/msg/TwistStamped`,
`trajectory_msgs/msg/*`, `control_msgs/action/*`, `control_msgs/msg/JointJog`,
`controller_manager_msgs/srv/*`, `lifecycle_msgs/srv/ChangeState` and
`rcl_interfaces/srv/SetParameters*` (only `param_set` sets parameters). A type written `pkg/Name`
is checked as every kind it could be.

A name the model gives an act must be absolute: `cmd_vel` would otherwise reach `/cmd_vel` past a
list written for absolute names. The read tools take `odom` as `/odom`, and the rules see it that
way too.

### `[ros_tools]`

Generic ROS tools: what `ros2 topic/service/action/param/node` and `tf2_echo` do, natively and
without a shell. With the table present, six read tools are always there:

| Tool | Does |
|---|---|
| `ros_graph` | Lists topics, services, actions or nodes (filtered by a substring or a glob of the name; a component container's entry lists the nodes it hosts), or describes one name: a topic's publishers and subscribers with their QoS, and which subscribers receive nothing because they ask for more than a publisher gives; a node's topics, services and clients; a service's or action's type and definition. |
| `topic_sample` | `echo` up to 10 messages, long arrays and strings shortened, `fields` to pick some; `hz` for the rate and its jitter; `bw` for bytes per second. Images, point clouds and maps are too large to echo. |
| `interface_show` | A message, service or action definition. |
| `tf` | Where one frame is in another, or every link with its parent, whether it is static and its age. |
| `params` | A node's parameters: names, values (all of them when none are named) and descriptions, at most 80 a call, narrowed by `prefix` or `filter`. |
| `log_tail` | Recent `/rosout` lines, warnings and errors by default. |

Four act tools exist only when their list names something.

- Each call needs the robot armed and, when supervised, the operator's approval of the resolved
  target, type and payload.
- It is checked against the interface, the hard deny lists and the profile's list before anyone
  is asked.
- Except for `param_set`, it holds every resource while it runs. A running mission holds them all
  until it ends, so the two cannot overlap: the second is refused as busy.

| Tool | Does |
|---|---|
| `service_call` | Calls a service; its type is looked up when not given. |
| `action_goal` | Sends a goal and waits for the result, up to 120 s, then cancels it; or cancels every goal of an action. A goal is cancelled too when its turn is stopped. |
| `param_set` | Sets one parameter, converted to its current type, and reads it back. Risk `world_edit`: it needs the robot armed and, when supervised, approval, and holds no resource. |
| `topic_publish` | Publishes a message up to 10 times, at up to 10 Hz, with the agent's own QoS. |

| Key | Default | |
|---|---|---|
| `hidden` | action internals, `/rosout`, `/parameter_events`, parameter services | Left out of listings unless `all` is asked; not a boundary. |
| `read_deny` | none | Topics never sampled, echoed, watched, plotted, read by a playbook or used to start a schedule. |
| `param_read_deny` | `*key*`, `*token*`, `*secret*`, `*password*` | Parameter values shown as hidden, matched on the name whatever its case: in `params`, `param_set`'s answer, a `service_call` reply, and the files `nervros-cli record` writes. |
| `service_observe` | `get_*`, `list_*`, `describe_*` | Services that only read: `service_call` runs them unarmed and unapproved. A pattern without `/` matches the last part of the name, so `get_*` covers `/map_server/get_map` but not `/arm/get_ready/execute`. |
| `service_call` | none | Services `service_call` may call. |
| `action_send` | none | Actions `action_goal` may reach. |
| `publish` | none | Topics `topic_publish` may publish on. |
| `param_set` | none | `node:parameter` globs `param_set` may change. |

`nervros-cli ros <tool> '<json>'` runs one read tool against the live graph, without a model.

### `[look]`

The `look` tool: a camera frame with the current detections drawn on it as numbered marks. The
operator sees the frame. The chat model gets the marks and the answer of the `vision_check` model,
which is shown the marked frame (768 px on its long side) with the chat model's `question`. With no
vision model available, `look` returns the marks alone and says why.

| Key | Default | |
|---|---|---|
| `name` | `"main"` | The camera's name, as `look` and `segment` take it in `camera`. |
| `image` | required | A `sensor_msgs/msg/Image` topic. |
| `detections` | required | `{ topic, type }`, where the type is `canopy_msgs/msg/InstanceMaskArray` or `vision_msgs/msg/Detection2DArray`. |
| `detection_image` | none | The image topic the detector reads, when it is not `image`, such as frames from a still base. Its frames are kept apart, so the marks are drawn on the frame they were cut from; while it sends none, as when the base moves, `look` and `point` wait up to `max_age` for new masks and say why there are none. |
| `max_marks` | `12` | |
| `max_age` | `"5s"` | Older detections are left out, and `look` and `point` wait this long for ones of a kept frame. A slow detector needs more. |
| `about` | none | What the vision model should know about the camera, such as where it points and how far it sees. |

More cameras go under `[look.cameras.<name>]` with `image`, `about` and, optionally, `detections`
and `detection_image` (without detections `look` shows that camera's frame unmarked). The tools
then take a `camera` argument, `[look]`'s own by default, and the chat model sees each camera's
`about` to choose one.

With `[look]` and `[mission]`, a mission that succeeds is checked through the cameras too, for
each goal that puts one thing in or on another, up to four. The detector's marks on the newest
frame come first. Goals they do not settle go to the `witness` model in one request, with every
camera's frame from the mission's start and from its end. Its "no" flags a success the world model
believed; its "yes" never clears a goal the world model found unmet.

```toml
[look]
name = "chest"
image = "/chest_camera/color/image_raw"
detections = { topic = "/detector_chest/instance_masks", type = "canopy_msgs/msg/InstanceMaskArray" }

[look.cameras.head]
image = "/camera/color/image_raw"
about = "It points down at the floor in front of the robot."
```

### `[keyframes]`

Each camera's frames kept on disk as its view changes, for `when`. It needs `[look]`. Every
`period_s` each camera's newest frame is compared with that camera's last kept one, by a small grey
thumbnail, and kept when it differs: a still robot in a still room keeps almost nothing. Frames go
under the state directory, `keyframes/<robot>/<camera>/<milliseconds>.jpg`.

| Key | Default | |
|---|---|---|
| `period_s` | `2.0` | How often each camera is looked at. |
| `keep_h` | `24.0` | How long frames are kept, and how far back `when` looks at most. |
| `max_mb` | `2000` | The most disk they take together; the oldest go first. |
| `edge_px` | `640` | The longest side of a kept frame. |

### `[segment]`

The `segment` tool: masks for whatever a prompt names in a camera's newest frame ("the floor",
"every mug on the table"), including things no detector marks. The operator sees the regions drawn,
as a cutout on a dimmed frame or tinted over it. The chat model gets each region's label, box and
share of the frame, as marks of the snapshot. It needs `[look]`'s cameras.

| Key | Default | |
|---|---|---|
| `backend` | `"model"` | `model` asks the models file's `segment` role for outlines, Gemini's segmentation format (`box_2d` and a polygon, 0 to 1000); `service` calls `service`. A call may name the other when both are set up. |
| `service` | none | A `canopy_msgs/srv/Segment` service: the camera, by the profile's name, and the prompt in; `success`, `message` and an `InstanceMaskArray` cut from one of that camera's frames out. canopy's `segmenter` is one. |
| `timeout` | `"40s"` | How long the service may take; its first call may load the models. |

`nervros-cli segment "the floor" --camera head --out floor.jpg` runs it once, without the chat
model.

### `[editor]`

canopy's world editor (`editor/canopy_editor.py <world_dir>`), which serves one saved world over
HTTP. With it:

- the agent gets `review_world` and `inspect_object` (reads), and `edit_world` and `world_edits`
  (edits, approved when supervised, no arming);
- the app gets **Edit world**: the saved world on its floor plan, where you pick, move, resize,
  turn, split, merge and add objects and type rooms, as on canopy's own editor page.

Both go through the editor, so each sees the other's edits.

| Key | Default | |
|---|---|---|
| `url` | `"http://127.0.0.1:8765"` | The editor. |
| `token` | none | `{ file = "..." }` or `{ env = "..." }`, when it serves off loopback. |
| `reload` | none | A `std_srvs/srv/Trigger` that makes the running world model read the saved world again, called after `world_edits` saves: canopy's `/canopy/reload`. |

### `[world]`

The world model's topics, each `{ topic, type }` and each optional:

| Key | Type |
|---|---|
| `map` | a `nav_msgs/msg/OccupancyGrid` |
| `rooms`, `objects` | canopy's `RoomArray` and `WorldObjectArray`, or messages with the same fields |
| `coverage` | an `OccupancyGrid` of what the camera has seen, canopy's `/canopy/coverage`: 0 seen, 90 still to see, 99 written off |
| `trail` | a `nav_msgs/msg/Path` of where the robot has been |

The viewer draws them all, with rooms coloured by how much of each the camera has seen, and plots
that share beside the room and object counts. `list_places`, `robot_state` and missions read rooms
and objects. The app's World tab lists the rooms with how much of their floor and walls was seen,
and its **Explore** button asks the agent to explore.

`history` names canopy's `canopy_msgs/srv/ObjectHistory` service: what happened to each object,
when it appeared, moved, went missing, was seen again or merged. `recall` reads it, and a failed
mission's report says where its object was last seen. An id the world model does not hold, such
as an executor's `mug_4`, names its object only when one object of that kind is known. With
several mugs, the robot's own missions with `mug_4` say where it went instead.

### `[viz]`

What the viewer draws beyond the world model, each optional:

| Key | |
|---|---|
| `plan` | `{ topic, type }`: the `nav_msgs/msg/Path` the navigation stack follows, Nav2's `/plan` |
| `camera_info` | `{ topic, type }`: the `[look]` camera's `sensor_msgs/msg/CameraInfo` |
| `urdf` | the robot's URDF file, relative to the profile |

- With `camera_info`, the camera frame is drawn in the world where the camera is, as a frustum
  placed by TF from `[ros] base_frame` to the image's frame. Without it, the frame is drawn on its
  own.
- With `urdf`, the robot is drawn as its model, posed by TF: the model's root from
  `[ros] base_frame`, and each joint that is not fixed from its parent link to its child link.
  `package://` meshes are found through `ROS_PACKAGE_PATH` or `AMENT_PREFIX_PATH`. The robot's
  heading arrow is drawn either way.
- Every camera in `[look]` gets its own view, with the detector's boxes on its frames.

`[[viz.layer]]` adds a topic the viewer draws in the world as RViz would:

```toml
[[viz.layer]]
name = "Next viewpoint"                   # its switch in the app's Layers tab
topic = "/canopy/markers"
type = "visualization_msgs/msg/MarkerArray"
namespaces = ["viewpoint", "visited"]     # a marker array's namespaces to draw; all when left out
hidden = false                            # starts hidden
group = "World model"                     # its heading in the Layers tab; one by its type otherwise
```

| Type | Drawn as |
|---|---|
| `nav_msgs/msg/OccupancyGrid` | Its occupied cells, over the map in the layer's colour. With `style = "costmap"`, a Nav2 costmap's costs: inflation from faint blue to indigo, the inscribed radius in violet, obstacles in coral, free cells clear. A grid in another frame, as a local costmap in `odom`, is placed by TF. |
| `visualization_msgs/msg/MarkerArray` | Each marker by its type: arrows, cubes, spheres, cylinders, lines, point lists, triangle lists and text. Mesh markers are left out. |
| `sensor_msgs/msg/LaserScan` | Points in the map, by TF from the scan's frame. |
| `geometry_msgs/msg/PolygonStamped` | A closed outline on the map, by TF from its frame: a collision monitor's polygon as it shows it (`visualize: true`). The Layers tab names it beside its colour. |

`color = [r, g, b]` gives a layer its colour; the viewer picks one otherwise.

The Layers tab lists the layers under headings (Robot, Navigation, Arms, World model, Sensors), each
with its colour, and also switches the viewer's own drawings: the map, coverage, rooms, objects,
trail, plan, zones (keep-out and slow), MoveIt's obstacles, the robot model and the detector's
boxes. Object names start hidden, since a
furnished room buries the map in them; hovering a box names it either way.

### `[mission]`

The robot's mission executor, which runs behaviour trees; see
[the executor contract](executor.md). A robot with `[nav2]` or `[moveit]` and no executor still
gets missions, of the motions NervROS runs itself ([integrations](integrations.md#native-steps)). With this section the agent gets `run_mission`. It checks a
plan, shows it, and runs it once the operator approves, so the operator approves once, in the app,
and the agent never asks first in the chat. `check_only` only checks a plan.

| Key | Default | |
|---|---|---|
| `execute` | required | The `ExecuteMission` action. |
| `validate` | required | The `ValidateMission` service. |
| `catalog` | required | The `GetCatalog` service. |
| `stop` | required | The `StopAll` service, which the Stop button and `stop` call. |
| `pause` | none | The `PauseMission` service: "pause" and "resume" in the chat while a mission runs, `/pause` and `/resume`, and the Pause buttons pause the mission and let it go on. |
| `state` | required | The `RobotState` topic. |
| `max_replans` | `2` | Plans after a failed mission for one request, counted until the operator speaks again: after `max_replans + 1` failed missions the agent hands back to the operator. |
| `preview` | none | The `PreviewMission` service: the viewer draws where a plan's walks end and the paths to them beside its approval card, and the card marks a step the preview cannot reach. |
| `heartbeat` | none | Where the agent publishes `nervros_interfaces/msg/Heartbeat` from the session's own loop, as long as the session runs; the executor listens while a mission runs and stops one whose agent is gone, crashed or hung. Without it a mission runs on alone. |
| `heartbeat_timeout_s` | `2.0` | How long a mission may go without a heartbeat; the executor clamps it to its own limit. |
| `teleop` | none | The `Teleop` service: Drive in the Robot tab hands the base to the operator while no mission runs and the robot is armed; disarming hands it back. W S walk, A D turn, Q E step sideways. |
| `teleop_cmd` | none | Where the hand-driving `geometry_msgs/msg/Twist` commands go; the executor caps their speed and stops the base when they pause. |
| `teleop_speed` | `[0.5, 0.3, 0.8]` | Full hand-driving speed forward and sideways (m/s) and turning (rad/s); Slow in the Robot tab halves it. |
| `define_skill`, `forget_skill` | none | The `DefineSkill` and `ForgetSkill` services: with both, the agent gets `teach_skill` and teaches the robot skills from the executor's learnable leaves; see [the executor contract](executor.md#taught-skills). |
| `checks` | none | The skills the plan checks know; see below. |
| `lend` | none | The `LendResources` service: an executor that holds the arms lends them for an arm move NervROS runs itself through MoveIt, and takes them back after; see [integrations](integrations.md#native-steps). |

Before a plan reaches the operator, it is checked against their words: a left turn planned as a
right one, a walk the wrong way or of another length, the other hand. Such a plan goes back to the
model once, then comes to the operator with its concerns and their fixes. Skill names, units and
signs are the executor's, so these rules check only the skills `[mission.checks]` names:

```toml
[mission.checks]
turn = { skill = "TurnInPlace", degrees = "degrees" }  # positive_left = false if + turns right
walk = { skill = "WalkStraight", metres = "distance_m", direction = "direction", max_m = 2.0 }
arm = "arm"                                            # the argument valued left or right
```

- `max_m` is the longest walk one step makes. A longer request needs more steps, so no single-step
  fix is offered.
- A request that names what to handle only as "it", said first in a session, is refused with a
  note to ask which.
- A request with a clock ("every 10 minutes") goes to `schedule`.
- With a `plan_check` model in the models file, a second model also judges the plan from the
  operator's words. With a `plan` model other than the routine one, it advises the planner once a
  request's plans have failed twice.
- With `[look]` and a `scene_check` model, a plan with a step that failed in at least half of its
  last runs is also checked against what the cameras show now, with the steps' track records. Its
  doubts are added to the card; they never send the plan back.

Missions are kept in a ledger beside the quota file. The card shows how each step went before, and
`recall`, the Library tab and `nervros-cli missions` read the ledger.

A run the robot or the executor ended, rather than the plan, is kept with its fault: the robot could
not move, the deadman stopped it, the executor shut down or restarted, a skill's server was not up,
someone drove the base by hand, or another mission ran. It counts in no track record, judges no
taught skill and uses none of the request's replans, and its report says what the fault needs
instead of asking for a new plan. A program that meets one stops there. A mission still kept as
running when NervROS starts was cut short by NervROS stopping: it is kept as interrupted, and the
window says so.

With a `plan` model, a step that fails again on fair runs (twice, and a third of its latest runs at
least) gets a proposed fix, one at a time per step: a call of `memory` (remember), `geofence` (add)
or `param_set`, drafted by the `plan` model and checked against the tool's own arguments. The Agent
tab lists it to accept, which runs the call as the operator's own (asked about first only where the
profile's rules say so), or to dismiss, which keeps that step from getting another. An accepted fix
shows the step's record before and since. `nervros-cli inbox` lists them and `--dismiss N` turns one
down. Proposals live in the state directory, `inbox/<robot>.json`.

### `[[place]]`

Named places, beyond the world model's rooms. The operator can add more from where the robot
stands ("remember this spot as the reading corner"). `tag_place` keeps them beside the quota
ledger, one JSON file per robot, and `forget_place` drops them. Neither changes this file, and a
remembered place cannot take a name the profile already uses.

```toml
[[place]]
name = "dock"
aliases = ["the charging dock"]
pose = { x = 0.0, y = 0.0, yaw = 0.0 }   # in frame, default "map"
near = ["charger_1"]                     # reached from here; see below
```

`near` lists what the robot reaches from the place that the world model may not know, such as a
detector's name for an object. A plan step that needs the robot near one of them walks to the
place first, as it walks to any object the world model knows.

### `[geofence]`

Zones: polygons on the map. A keep-out zone the robot may not be sent into; a slow zone it may
enter no faster than a speed.

- The profile's zones are fixed. The operator adds more in chat with `geofence` ("keep the robot
  out of the kitchen", "keep it a metre from the vase", "make the corridor a slow zone at 30%"),
  around a room, an object, a place or corners on the map. These are kept per robot in the state
  directory, beside the remembered places.
- Adding a zone, or slowing a slow zone down, only makes the robot more careful, so it is free.
- Removing a keep-out zone asks the operator every time, supervised or autonomous, and is never
  allowed for the session. Removing a slow zone or letting the robot go faster in one asks too,
  but may be allowed for the session. An `observe` profile refuses them. Only an edit of this
  file changes the profile's zones.

```toml
[geofence]
slowest_percent = 30   # the slowest a slow zone may make the robot; default 5
top_speed_mps = 0.6    # optional: then a slow zone may give its speed in m/s

[[geofence.zone]]
name = "stairs"
polygon = [[4.0, 0.0], [5.0, 0.0], [5.0, 2.5], [4.0, 2.5]]   # map frame, metres, in order

[[geofence.zone]]
name = "corridor"
polygon = [[0.0, 3.0], [6.0, 3.0], [6.0, 4.2], [0.0, 4.2]]
slow = { percent = 50 }   # or { mps = 0.3 }
```

A slow zone's speed is a share of the robot's top speed, as Nav2's speed filter takes it. Where
slow zones overlap, the slowest holds.

What the guard refuses:

- A Nav2 goal inside a zone (`NavigateToPose`, `NavigateThroughPoses`, `FollowWaypoints`,
  `FollowPath` and the path planners' actions), and a `PoseStamped` published inside one. A pose
  in another frame is taken through TF, and refused when TF cannot place it.
- While zones are set, Nav2's straight moves (`DriveOnHeading`, `BackUp`), which only a mission
  step checks. `action_goal` never starts a mission past `run_mission`.

What the plan check refuses:

- A mission whose walk to a place, pose or target ends in a zone (for a label several objects
  share, any of them ending in one).
- A mission whose straight walk passes through a zone, followed from where the robot stands. A
  walk out of a zone the robot stands in is allowed. The straight-walk check covers the skill
  `[mission.checks] walk` names.
- A plan run later, by its hash or a schedule, is checked again as it starts.

Nav2 learns the zones through [`[nav2]`](#nav2): keep-out zones through `keepout_mask`, slow
zones through `speed_zones`. Without them, Nav2 knows nothing of the zones and only the agent's own
checks hold the keep-out zones: a walk whose route crosses one, driving by hand, and skills that
move the base themselves are not held back, and nothing slows the robot in a slow zone.

The viewer draws keep-out zones in red and slow zones in amber with their speed; the World tab
lists them, and its buttons add and remove them through the agent.

- A zone needs a name, at least three corners and an area of 0.05 m² or more. Names are unique,
  whatever their case, across the profile's zones and the chat's. A bad zone in the profile stops
  the start.
- In chat, a room is found by its id, then its name, then its type. A type several rooms share is
  refused until one is named by id.
- An object or a place gets a margin of 0.3 m unless the operator says otherwise. A place is a
  square reaching half a metre out from it, plus the margin.

### `[nav2]`

Nav2, when the robot runs it. The section gives the agent `nav_status`, `nav_check`,
`collision_monitor` and `nav_admin` (see [Builtin tools](#builtin-tools)), a line on what Nav2 is
doing in every turn, and the zones in Nav2's costmaps. Names without a leading `/` are under
`namespace`.

```toml
[nav2]
keepout_mask = "/keepout_filter_mask"   # optional; needs [world] map

[nav2.speed_zones]                      # optional; needs [world] map
mask = "/speed_filter_mask"
info = "/speed_filter_info"             # optional: NervROS publishes the filter info itself
```

| Key | Default | |
|---|---|---|
| `namespace` | none | Where Nav2's nodes and servers run, such as `/robot1`. |
| `keepout_mask` | none | Where the keep-out zones go out for Nav2's `KeepoutFilter`: an `OccupancyGrid` the size of `[world] map`, the zones' cells at 100, latched (transient local), published again whenever the zones or the map's grid change. A `costmap_filter_info_server` names it to the filter. |
| `speed_zones.mask` | none | Where the slow zones go out for Nav2's `SpeedFilter`: the same grid, each cell the per cent of the top speed the slowest zone over it allows, 0 (no limit) elsewhere. |
| `speed_zones.info` | none | Where NervROS publishes the speed filter's `CostmapFilterInfo` (in per cent, base 0, multiplier 1), latched, so the robot needs no filter info server for it. Leave it out when the robot runs its own. |
| `lifecycle_manager` | `lifecycle_manager_navigation` | The lifecycle manager of the navigation servers: `is_active` for the status, `manage_nodes` for `nav_admin`. |
| `collision_monitor` | `collision_monitor` | The collision monitor node. |
| `costmaps` | `["global_costmap/global_costmap", "local_costmap/local_costmap"]` | The costmap nodes, whose clear and cost services `nav_admin` and `nav_check` call. |
| `drive_speed_mps` | `0.25` | How fast a straight walk NervROS runs itself goes. |
| `docking` | `false` | Whether the robot runs Nav2's docking server: then missions may dock and undock. |

Missions also get Nav2's motions as steps NervROS runs itself, where the robot's executor has no
skill of the name: `GoToPose`, `GoToTarget` (with a world model), `WalkRoute`, `TurnInPlace`,
`WalkStraight`, `Wait`, and `Dock` and `Undock` with `docking`; see
[integrations](integrations.md#native-steps).

`nervros-cli doctor` says whether Nav2's navigation server is up and whether a costmap filter reads
each mask.

What Nav2 1.3 lets NervROS change while it runs, and so what `collision_monitor` offers: the whole
monitor on or off, each polygon on or off, and the shape of a polygon that takes its shape from a
topic (`polygon_sub_topic`, with no `points`; set `polygon_subscribe_transient_local: true` so the
shape NervROS keeps reaches it after a restart). A polygon's action, slowdown ratio, limits and
timing are read when the node starts. Keep the robot's own stop polygon fixed: a polygon that takes
its shape from a topic stops nothing until a shape arrives.

### `[moveit]`

MoveIt, when the robot runs it. The section gives the agent `arm_status`, `arm_check`, `scene` and
`arm_settings` (see [Builtin tools](#builtin-tools)), a line on where each arm is and what MoveIt's
planning scene holds in every turn, and an arm preview drawn in the 3D view, beside the planning
scene's boxes, spheres and cylinders (the Layers tab's "Arm obstacles"). Stop also tells MoveIt
to stop any trajectory it runs; the hands keep their grip. NervROS reads the groups, end effectors
and named poses from `move_group`'s `robot_description_semantic`. Its tools ask MoveIt's services,
never its move action, so nothing they ask disturbs a motion the robot's executor runs; only a
mission step moves an arm.

Missions also get MoveIt's motions as steps NervROS runs itself, where the robot's executor has no
skill of the name: `ArmTo` (a named or remembered pose), `HandTo` (just above an object, or a
point), `HandLine` (a short straight hand move) and `Grip` (a hand's named state; never opening a
hand that holds something). See [integrations](integrations.md#native-steps).

| Key | Default | |
|---|---|---|
| `move_group` | `/move_group` | The `move_group` node; its services sit beside it, as MoveIt names them. |
| `planning_frame` | the SRDF's virtual joint | The frame MoveIt plans in, its robot model's root, such as `pelvis`. |
| `joint_states` | `/joint_states` | The `sensor_msgs/msg/JointState` topic with the arms' joints. |
| `scaling` | `0.2` | The share of the arms' top speed NervROS's own arm moves start at; `arm_settings` changes it. |
| `max_scaling` | `0.5` | The most `arm_settings` may raise it to. |

`nervros-cli doctor` says whether it runs. The host needs `ros-jazzy-moveit-msgs` to build NervROS
with MoveIt's messages.

The obstacles NervROS adds are kept on the map, per robot in the state directory, and sent again
in the planning frame whenever it moves on the map: a robot whose model is rooted at its body
would otherwise carry them along as it walks.

### `[incidents]`

The incident investigator: what its flight recorder watches, and this robot's playbooks. It also
runs without the section, recording `/rosout`, the nodes and the missions, with the playbooks
NervROS carries (the robot not placed on the map, a node running twice).

```toml
[incidents]
playbooks = ["playbooks"]   # folders of *.toml, relative to this file
window_s = 300              # how far back the recorder keeps
watch = [                   # going quiet, after keeping its rate, opens an incident
    { name = "head camera", topic = "/camera/color/camera_info", type = "sensor_msgs/msg/CameraInfo", min_hz = 1.0 },
]
nodes = ["detector_head", "nervros_executor"]   # stopping opens one; full name or last part
```

A playbook names:

- the incidents it may explain (`mission_failed`, `topic_silent`, `node_gone`, `error_burst`,
  `asked`; none for all),
- whether `all` its checks must hold or `any`,
- the verdict and the fix,
- the agent skill with the long form, if any.

For example:

```toml
id = "robot-lost"
title = "The robot is lost on the map"
explains = ["mission_failed", "asked"]
verdict = "Localization settled far from where the simulator has the robot."
fix = "Set the robot's pose again before any walk."
procedure = "lost-on-the-map"

[[check]]
name = "map to pelvis, against the simulator"
pose_gap = ["map", "pelvis"]
topic = "/g1_sensor_relay/base_state"
type = "nav_msgs/msg/Odometry"
field = "pose.pose"
metres = 1.0
degrees = 30
```

Each check has one test, and may have a `name` for the report and `not = true`:

| Test | Holds when |
|---|---|
| `topic`, `type`, `condition` | The topic, read now, meets a `watch` condition: `rate_below` with `hz`, `value` with `field`, `op` and `value`, or `text` with `contains`. |
| `log` | The recording has a line holding these words, case aside; `node` narrows it to nodes whose name holds that, `level` (`warn` unless given) and `at_least` (1) set the least. |
| `node_missing`, `node_duplicated` | The node is not running, or runs twice under one name; `*` for any node. |
| `tf_missing` | `[target, source]`: no transform links them. |
| `pose_gap` | `[target, source]` differs from the pose at `field` of `topic`'s newest message by more than `metres` or `degrees`. |
| `failure_has` | The failed mission's reason holds these words. |

- A check that cannot run leaves its playbook unchecked rather than ruled out.
- `{map_frame}` and `{base_frame}` in a playbook are filled from `[ros]`.
- A playbook with the same id as an earlier one replaces it. The profile's folders come after
  NervROS's, and the ones the agent was asked to remember, kept per robot in the state directory,
  come after those.
- Incidents are kept in the state directory too, the newest fifty.

### `[models]`

`file`: the [models file](models.md).

### `[[mcp_server]]`

Tools from an MCP server, behind the same guard as every other tool.

- Only the tools listed exist for the agent, and each only while its definition is the one
  approved. `nervros-cli mcp pin` keeps each tool's definition, by a hash of its canonical form and
  its text, in `mcp.lock.json` beside the profile. A tool whose definition changed is left out
  until it is pinned again.
- A tool acts, with arming and approval, unless the profile marks it `observe`. The server's own
  hints are not believed.
- What a tool returns is cut to a size and fenced with the server and tool it came from.

```toml
[[mcp_server]]
id = "docs"                          # tools appear as docs__<name>
transport = "stdio"                  # or "http" with url and, optionally, bearer_file
command = "/usr/local/bin/docs-mcp"  # absolute; nothing is fetched to run it
args = ["--root", "/srv/docs"]
env = { LANG = "C.UTF-8" }           # its whole environment: no model key reaches it
open_world = false                   # true when it reaches the internet: off in home mode
[mcp_server.tools.search]
observe = true
description = "Searches the robot's manuals."
```

| Key | Default | |
|---|---|---|
| `max_result_bytes` | `8192` | A result past this is cut. |
| `timeout` | `"20s"` | How long a call may take. |

`nervros-cli mcp` lists each server's tools and whether they are pinned; the doctor and the app's
Doctor tab show a server that did not connect.

## Builtin tools

Every robot gets `list_places`, `robot_state`, `geofence` and `stop`. `stop` is always allowed,
armed or not, because it only makes the robot do less; it cancels the goals NervROS sent itself
too. `look` comes with `[look]`, `segment` with
`[segment]`, and `run_mission` with `[mission]`.

The rest are the checks you would run yourself before blaming the model:

| Tool | What it does |
|---|---|
| `health_check` | The connection check, plus each camera's frame rate, whether the robot knows where it is on the map, the executor's state, and the known problems the playbooks check for. Returns what is wrong and what is fine. |
| `why` | Explains the newest incident (within ten minutes), one by its id or its mission's, or checks the robot now; `list` lists the incidents. See [`[incidents]`](#incidents). |
| `remember_finding` | Keeps a playbook the agent writes from a debugging session, checked as the profile's are; an `annotate` call. |
| `watch` | Watches a topic in the background and posts to the chat when a condition holds: its rate drops below a floor, a field crosses a value, or a text field matches. With `[look]`, `sees` watches what a camera shows instead ("a person at the door"), asking the `watch` model every 15 s and posting on two yes in a row; at most 2 of those. Once, or each time it comes back; at most 8 at a time, all ended with the session. |
| `when` | With `[keyframes]`: finds when something happened ("when did the mug leave the table?") from the frames kept, sampled to a dozen, narrowed to the moments the world model saw the thing change when it did, and asked of the `moment` model; a second look between the chosen frame and the one before closes a wide gap, and the `vision_check` model confirms the frame alone or finds its neighbour that shows it. Returns the time and the frame as a snapshot. |
| `digest` | With `[mission]`: what the robot did over the last hours, in sections code writes: missions by how they ended, the runs cut short by the robot apart, each skill's record on fair runs, what moved or went missing in the world model, incidents and goals. `nervros-cli digest` prints the same from the robot's files, without the world model. |
| `patrol` | With `[mission]`: walks a round of places as a job, looking around at each with a full turn on the spot (the `[mission.checks]` `turn` skill), then reports what it saw in each room and what moved or went missing meanwhile. Approved once with its round and time limit. |
| `watches` | Lists the running watches, or cancels one or all. |
| `plot` | Draws a number from a topic's messages over time in the app's Plots tab, as `rqt_plot` does, for two minutes unless told longer. A joint state's joint is named, as `position.right_elbow_joint`; a topic quiet now, as a velocity command between walks, is checked against its type and drawn once it publishes. |
| `memory` | Keeps what the operator asks it to remember across sessions ("the kitchen door sticks"), in the state directory, one file per robot; lists and forgets notes. The notes join the system prompt on every turn, and the Agent tab lists them. Remembering and forgetting are approved like an edit. |
| `goal` | With `[mission]`: keeps a goal the operator wants to hold across missions and sessions ("keep the mug on the tray"), with the predicates that hold once it is met (`at`, `holding`, `inside`, `on`), a budget of missions and an optional time limit, in the state directory, one file per robot. NervROS judges it after every mission the agent runs and reports, and at start, and the agent works toward it with missions that name it in `run_mission`'s `goal`. It ends met, exhausted, or impossible when the agent gives it up with the reason. Setting, pausing and giving up are approved like an edit. |
| `schedule` | With `[mission]`: runs a plan again and again, such as a patrol every 30 minutes, a set number of times, or each time something happens: an object of a kind appears, in a room or anywhere, or a condition on a topic as `watch` takes it becomes true. The operator approves it once for all its runs; a run is skipped while the robot is disarmed or busy, and stopping the robot cancels every schedule. The Mission tab lists them. |
| `recall` | With `[mission]`: what the robot did lately, from the mission ledger, what happened to an object, from `[world] history` and the robot's own missions with it, and the operator's notes. |
| `plans` | With `[mission]`: saves a plan that worked by name ("evening check"), lists them, and runs one again. |
| `skill_gap` | With `[mission]`: logs a request no skill can do, and why; planning that gives up logs one itself. The Library tab and `nervros-cli gaps` list them. |
| `program` | With `[mission]`: runs a short [Rhai](https://rhai.rs) program for what takes loops, counting or measuring, sandboxed (no files, network or `eval`, bounded in operations and size), on the robot's functions: the world model, topic reads and rates, TF, waits, and skills, each run as a mission. One that moves is approved once, with the skills its text names and its time limit, and runs as a job with Stop; one that only reads runs at once. Programs are kept by name per robot in the state directory, and the Library tab lists them. |
| `search` | With `[mission]`: finds something, ranking where to look from the world model's sightings and history and where such things usually are, then walking to each place and looking until the detector sees it; approved once, with the places it will look in order. `plan` lists the places without moving. |
| `teach_skill` | With `[mission]`'s `define_skill` and `forget_skill`: teaches the robot a skill from the executor's learnable leaves, as a draft kept once a mission with it succeeds; lists and forgets taught skills. |
| `point` | With a `segment` model in the models file: points at what the operator names in the frame a camera's detector marks, found as `look` finds it, or the newest frame when there are no marks, including things no detector marks, and says which mark a point lands on. |
| `skill` | With `skills`: reads one skill's procedure. |
| `tag_place`, `forget_place` | Remembers where the robot stands, and which way it faces, as a named place; forgets one. Each is an `annotate` call: approval when supervised, no arming. |
| `geofence` | Lists, adds, slows and removes zones (see `[geofence]`): keep-out zones and slow zones, around a room, an object (its size and a margin), a place, or corners on the map. Adding is free, and a keep-out zone is refused while the robot stands inside it or when a type names several rooms; removing a keep-out zone always asks the operator, removing or speeding up a slow zone asks too. |
| `nav_status` | With `[nav2]`: whether Nav2's servers are up, how sure localization is, what moves the base and how far it has to go, what the collision monitor is doing, the speed limit in force, and the zones. |
| `nav_check` | With `[nav2]`: whether the robot can walk to a place, a room, an object or a point, how far and about how long, and through which zones, from Nav2's planner without moving (refused while Nav2 walks, since a request would cut into its own planning); and what Nav2's costmap holds at a spot. |
| `collision_monitor` | With `[nav2]`: the collision monitor's polygons and what each does; the monitor on or off, a polygon on or off, and a polygon's shape where Nav2 allows it. Turning on and growing are free; turning off and shrinking ask the operator. |
| `nav_admin` | With `[nav2]`: clears costmaps, sets localization's pose or makes it search the map, pauses, resumes or resets the navigation servers, saves and loads maps. Each asks the operator when supervised, and none runs during a mission. |
| `arm_status` | With `[moveit]`: each arm group, its named poses and the one it is at, each hand and its poses, MoveIt's planners, and the obstacles in its planning scene and what each hand holds. |
| `arm_check` | With `[moveit]`: whether an arm's hand reaches an object or a point, held as it is now (IK); whether a named pose is clear of collisions now; and the path an arm would take to a named pose, an object or a point, and how long it takes, drawn in the 3D view. Nothing moves. |
| `scene` | With `[moveit]`: lists MoveIt's planning scene; adds an obstacle (a box, a cylinder or a sphere on the map, or an object of the world model by its box), free; removes one NervROS added, attaches an object to a hand or detaches it, and clears the octomap, each asking the operator. |
| `arm_settings` | With `[moveit]`: how the arm moves NervROS runs itself go: their speed (slower is free, faster asks, up to `max_scaling`), the planning pipeline, planner and time (asks), and arm poses remembered by name from where an arm is now, for `ArmTo` and `arm_check`. |

`look` can also answer about an earlier snapshot, and look closer at one mark: it crops the mark
from the full frame at a higher resolution for the vision model, for a label or a small part.

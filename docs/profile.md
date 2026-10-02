# The robot profile

Each robot has one `nervros.toml`, kept in the robot's own repository. It says how to reach the
robot's ROS graph, what the agent may do, where its camera and world model are, which of its
services and topics become tools, and which models answer. Paths in it are relative to the file.
An unknown key is an error, so a typo stops the start instead of being ignored.

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

The model sees the service's comment as the tool's description and gets a JSON Schema of the
request made from the `.srv` file: field comments become descriptions, constants become enums, and
byte arrays are hidden. The interface files are found through `[ros] interfaces`. A `topic` tool
returns the newest message on the topic. Publishing, and calling services or actions the profile
does not declare, is for the generic tools of [`[ros_tools]`](#ros_tools), behind the same guard.

| Key | Default | |
|---|---|---|
| `name` | required | The name the model calls. |
| `kind` | required | `service` or `topic`. |
| `ros_name` | required | The service or topic. |
| `type` | required | `pkg/srv/Name` or `pkg/msg/Name`. |
| `description` | from the interface | Replaces the interface file's comment. |
| `risk` | required for a service; `observe` for a topic | `observe`, `annotate`, `world_edit`, `motion` or `manipulation`. `annotate` changes only what the robot knows, such as the world model: no arming, the operator's approval when supervised. The rest are acts: they need the robot armed and, when supervised, approval. A service can act on the robot, so a service tool without it fails the start instead of running as a read. |
| `resources` | none | `base`, `left_arm`, `right_arm`: two calls holding the same one cannot run at once. |
| `timeout` | `"5s"` | Per call. |
| `hide_fields` | none | Request fields the model neither sees nor fills, such as an embedding. |
| `defaults` | none | Values the request starts from; the model's arguments go on top. |
| `schema` | generated | A full JSON Schema to use instead. |
| `world_text` | none | Result fields that hold text read from the world, such as names a detector or a describer gave: each is fenced as `<world>...</world>`, which the model is told is data, never instructions. Text on a sign or a box must not steer the robot. |

A name on the policy's `hard_deny` list cannot become a tool at all: the start fails.

## Sections

### `[robot]`

| Key | Default | |
|---|---|---|
| `name` | required | Shown in the app and told to the model. |
| `persona` | none | A markdown file about the robot, its abilities and limits, added to the system prompt. |
| `battery` | none | A `sensor_msgs/msg/BatteryState` topic, shown in the Robot tab. |
| `diagnostics` | none | A `diagnostic_msgs/msg/DiagnosticArray` topic with motor temperatures; the Robot tab shows the hottest. |

`skills` (at the top level, beside `[robot]`) lists folders of skills, relative to the profile:
procedures for what the agent meets rarely and must get right, such as navigation that never came
up. Each skill is a folder holding a `SKILL.md` that starts with a front matter of `name` (lower
case words joined by `-`) and `description` (when to use it). The system prompt carries one line
per skill; the `skill` tool reads one in full when it fits. `nervros-cli skills` lists them and
says what could not be read.

### `[ros]`

| Key | Default | |
|---|---|---|
| `domain_id` | `0` | Must equal `ROS_DOMAIN_ID`, which ROS reads when the node starts; the agent refuses to start otherwise. |
| `transport` | `default` | `udp` turns Fast DDS shared memory off, for a graph run by another user, such as a root container: shared memory between users fails without an error. The agent then needs `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, which `scripts/ros-env.sh` exports with `NERVROS_UDP_ONLY=1`. |
| `node_name` | `nervros` | The agent's node. |
| `discovery_timeout` | `"20s"` | How long start-up checks wait for the graph. |
| `interfaces` | none | Where interface packages are: an install prefix such as `/opt/ros/jazzy`, or a package's source directory. Tool schemas come from these. |
| `remap` | none | Path prefixes to rewrite, for an install tree whose links point at another machine's paths. |
| `map_frame` | `map` | The frame of places and poses. |
| `base_frame` | `base_footprint` | The robot's base. |

### `[privacy]`

`mode = "sim"` (the default) lets any model see camera frames. `mode = "home"` keeps frames on local
models, and sends text only to models that are local or do not train on it.

### `[policy]`

| Key | Default | |
|---|---|---|
| `start_armed` | `false` | Whether acts are allowed from the start; the app's switch changes it. |
| `autonomy` | `supervised` | `observe` refuses every act; `supervised` asks the operator for each; `autonomous` runs them. |
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
then = "deny"                   # or "ask": the operator approves it first, armed or not
reason = "the bedroom is private"
```

A rule reads the call as it was sent and as it will run, matching whatever the case and the
spaces around a value. For a plan that means its steps with each argument as `{name, value}`, a
plan run by its hash or a saved name included, and each walk's `place` both as the model wrote it
and as it resolved: the rule above holds whether the model wrote `R5`, `bedroom` or `Bedroom `.
Write a rule on a room or an object with its id. A rule naming a tool the agent does not have, or
an argument that tool does not take, stops start-up, since it would never apply.

The default `hard_deny` covers what commands motors, velocities or controllers directly, under any
namespace: `rt/lowcmd`, `*/lowcmd`, `*/rt/*`, `*cmd_vel*`, `*/controller_manager/*`,
`*/set_parameters`, `*/set_parameters_atomically`, `*/joint_trajectory`,
`*/follow_joint_trajectory`, `*/servo_node/*`, `*/apply_planning_scene` and `*/clear_octomap`. A
relative name in a `[[tool]]` is checked as the node resolves it, under `/`. The default
`hard_deny_types` are
`geometry_msgs/msg/Twist`, `geometry_msgs/msg/TwistStamped`, `trajectory_msgs/msg/*`,
`control_msgs/action/*`, `control_msgs/msg/JointJog`, `controller_manager_msgs/srv/*`,
`lifecycle_msgs/srv/ChangeState` and `rcl_interfaces/srv/SetParameters*` (only `param_set` sets
parameters); a type written `pkg/Name` is checked as every kind it could be. A name from the
model must be absolute: `cmd_vel` would otherwise reach `/cmd_vel` past a list written for
absolute names.

### `[ros_tools]`

Generic ROS tools, what `ros2 topic/service/action/param/node` and `tf2_echo` do, natively and
without a shell. With the table present, six read tools are always there:

| Tool | Does |
|---|---|
| `ros_graph` | Lists topics, services, actions or nodes (filtered by a substring or a glob), or describes one name: a topic's publishers and subscribers with their QoS, and which subscribers receive nothing because they ask for more than a publisher gives; a node's topics, services and clients; a service's or action's type and definition. |
| `topic_sample` | `echo` up to 10 messages, long arrays and strings shortened, `fields` to pick some; `hz` for the rate and its jitter; `bw` for bytes per second. Images, point clouds and maps are too large to echo. |
| `interface_show` | A message, service or action definition. |
| `tf` | Where one frame is in another, or every link with its parent, whether it is static and its age. |
| `params` | A node's parameters: names, values (all of them when none are named) and descriptions. |
| `log_tail` | Recent `/rosout` lines, warnings and errors by default. |

Four act tools exist only when their list names something. Each call needs the robot armed and,
when supervised, the operator's approval of the resolved target, type and payload. It is checked
against the interface, the hard deny lists and the profile's list before anyone is asked, and it
holds every resource while it runs. A running mission holds them all until it ends, so neither
can overlap the other: the second is refused as busy.

| Tool | Does |
|---|---|
| `service_call` | Calls a service; its type is looked up when not given. |
| `action_goal` | Sends a goal and waits for the result, up to 120 s, then cancels it; or cancels every goal of an action. A goal is cancelled too when its turn is stopped. |
| `param_set` | Sets one parameter, converted to its current type, and reads it back. A world edit: approval, no arming. |
| `topic_publish` | Publishes a message up to 10 times, at up to 10 Hz, with the agent's own QoS. |

| Key | Default | |
|---|---|---|
| `hidden` | action internals, `/rosout`, `/parameter_events`, parameter services | Left out of listings unless `all` is asked; not a boundary. |
| `read_deny` | none | Topics never sampled or echoed. |
| `param_read_deny` | `*key*`, `*token*`, `*secret*`, `*password*` | Parameter values shown as hidden. |
| `service_observe` | `get_*`, `list_*`, `describe_*` | Services that only read: `service_call` runs them unarmed and unapproved. A pattern without `/` matches the last part of the name, so `get_*` covers `/map_server/get_map` but not `/arm/get_ready/execute`. |
| `service_call` | none | Services `service_call` may call. |
| `action_send` | none | Actions `action_goal` may reach. |
| `publish` | none | Topics `topic_publish` may publish on. |
| `param_set` | none | `node:parameter` globs `param_set` may change. |

`nervros-cli ros <tool> '<json>'` runs one read tool against the live graph, without a model.

### `[look]`

The `look` tool: a camera frame with the current detections drawn on it as numbered marks. The
operator sees the frame; the chat model gets the marks and the answer of the `vision_check` model,
which is shown the marked frame (768 px on its long side) with the chat model's `question`. With no
vision model available, `look` returns the marks alone and says why.

| Key | Default | |
|---|---|---|
| `name` | `"main"` | The camera's name, as `look` and `segment` take it in `camera`. |
| `image` | required | A `sensor_msgs/msg/Image` topic. |
| `detections` | required | `{ topic, type }`, where the type is `canopy_msgs/msg/InstanceMaskArray` or `vision_msgs/msg/Detection2DArray`. |
| `max_marks` | `12` | |
| `max_age` | `"5s"` | Older detections are left out. |
| `about` | none | What the vision model should know about the camera, such as where it points and how far it sees. |

More cameras go under `[look.cameras.<name>]` with `image`, `about` and, optionally, `detections`
(without them `look` shows that camera's frame unmarked). The tools then take a `camera` argument,
`[look]`'s own by default, and the chat model sees each camera's `about` to choose one.

```toml
[look]
name = "chest"
image = "/chest_camera/color/image_raw"
detections = { topic = "/detector_chest/instance_masks", type = "canopy_msgs/msg/InstanceMaskArray" }

[look.cameras.head]
image = "/camera/color/image_raw"
about = "It points down at the floor in front of the robot."
```

### `[segment]`

The `segment` tool: masks for whatever a prompt names in a camera's newest frame ("the floor",
"every mug on the table"), including things no detector marks. The operator sees the regions drawn,
as a cutout on a dimmed frame or tinted over it; the chat model gets each region's label, box and
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
HTTP. With it the agent gets `review_world` and `inspect_object` (reads), and `edit_world` and
`world_edits` (edits, approved when supervised, no arming); and the app gets Edit world, the saved
world on its floor plan, where objects are picked, moved, resized, turned, split, merged and added
and rooms typed, as on canopy's own editor page. Both go through the editor, so each sees the
other's edits.

| Key | Default | |
|---|---|---|
| `url` | `"http://127.0.0.1:8765"` | The editor. |
| `token` | none | `{ file = "..." }` or `{ env = "..." }`, when it serves off loopback. |
| `reload` | none | A `std_srvs/srv/Trigger` that makes the running world model read the saved world again, called after `world_edits` saves: canopy's `/canopy/reload`. |

### `[world]`

The world model's topics, each `{ topic, type }` and each optional: `map` (a
`nav_msgs/msg/OccupancyGrid`), `rooms` and `objects` (canopy's `RoomArray` and
`WorldObjectArray`, or messages with the same fields), `coverage` (an `OccupancyGrid` of what the
camera has seen, canopy's `/canopy/coverage`: 0 seen, 90 still to see, 99 written off) and `trail`
(a `nav_msgs/msg/Path` of where the robot has been). The viewer draws them all, rooms coloured by
how much of each the camera has seen, and plots that share with the room and object counts;
`list_places`, `robot_state` and missions read rooms and objects. The app's World tab lists the
rooms with their floor and walls seen, and its Explore button asks the agent to explore.

`history` names canopy's `canopy_msgs/srv/ObjectHistory` service: what happened to each object,
when it appeared, moved, went missing, was seen again or merged. `recall` reads it, and a failed
mission's report says where its object was last seen.

### `[viz]`

What the viewer draws beyond the world model, each optional: `plan` (`{ topic, type }`, the
`nav_msgs/msg/Path` the navigation stack follows, Nav2's `/plan`), `camera_info` (`{ topic, type }`,
the `[look]` camera's `sensor_msgs/msg/CameraInfo`) and `urdf` (the robot's URDF file, relative to
the profile). With `camera_info`, the camera frame is drawn in the world where the camera is, as a
frustum placed by TF from `[ros] base_frame` to the image's frame; without it, the frame is drawn
on its own. With `urdf`, the robot is drawn as its model, posed by TF: the model's root from
`[ros] base_frame`, and each joint that is not fixed from its parent link to its child link.
`package://` meshes are found through `ROS_PACKAGE_PATH` or `AMENT_PREFIX_PATH`. The robot's
heading arrow is drawn either way. Every camera in `[look]` gets its own view, with the detector's
boxes on its frames.

`[[viz.layer]]` adds a topic the viewer draws in the world as RViz would:

```toml
[[viz.layer]]
name = "Next viewpoint"                   # its switch in the app's Layers tab
topic = "/canopy/markers"
type = "visualization_msgs/msg/MarkerArray"
namespaces = ["viewpoint", "visited"]     # a marker array's namespaces to draw; all when left out
hidden = false                            # starts hidden
```

| Type | Drawn as |
|---|---|
| `nav_msgs/msg/OccupancyGrid` | Its occupied cells, over the map in the layer's colour. |
| `visualization_msgs/msg/MarkerArray` | Each marker by its type: arrows, cubes, spheres, cylinders, lines, point lists, triangle lists and text. Mesh markers are left out. |
| `sensor_msgs/msg/LaserScan` | Points in the map, by TF from the scan's frame. |

The Layers tab also switches the viewer's own drawings: the map, coverage, rooms, objects, trail,
plan, the robot model and the detector's boxes. Object names start hidden, since a furnished room
buries the map in them; hovering a box names it either way.

### `[mission]`

The robot's mission executor, which runs behaviour trees; see
[the executor contract](executor.md). With this section the agent gets `run_mission`: it checks
a plan, shows it, and runs it once the operator approves, so the operator approves once, in the
app, and the agent never asks first in the chat. `check_only` only checks a plan.

| Key | Default | |
|---|---|---|
| `execute` | required | The `ExecuteMission` action. |
| `validate` | required | The `ValidateMission` service. |
| `catalog` | required | The `GetCatalog` service. |
| `stop` | required | The `StopAll` service, which the Stop button and `stop` call. |
| `state` | required | The `RobotState` topic. |
| `max_replans` | `2` | Failed missions for one request, counted until the operator speaks again, before the agent must hand back to the operator. |
| `preview` | none | The `PreviewMission` service: the viewer draws where a plan's walks end and the paths to them beside its approval card, and the card marks a step the preview cannot reach. |
| `heartbeat` | none | Where the agent publishes `nervros_interfaces/msg/Heartbeat` while a mission runs, from the session's own loop: the executor stops a mission whose agent is gone, crashed or hung. Without it a mission runs on alone. |
| `heartbeat_timeout_s` | `2.0` | How long a mission may go without a heartbeat; the executor clamps it to its own limit. |
| `teleop` | none | The `Teleop` service: Drive in the Robot tab hands the base to the operator while no mission runs and the robot is armed; disarming hands it back. W S walk, A D turn, Q E step sideways. |
| `teleop_cmd` | none | Where the hand-driving `geometry_msgs/msg/Twist` commands go; the executor caps their speed and stops the base when they pause. |

Before a plan reaches the operator it is checked against their words: a left turn planned as a
right one, a walk the wrong way or of another length, the other hand. Such a plan goes back to the
model once, and comes to the operator with its concerns and their fixes. A request that names what
to handle only as "it", said first in a session, is refused with word to ask; a request with a
clock ("every 10 minutes") goes to `schedule`. With a `plan_check` model in the models file, a
second model judges the plan from the operator's words too; with a `plan` model other than the
routine one, it advises the planner once a request's plans have failed twice. Missions are kept in
a ledger beside the quota file: the card shows how each step went before, and `recall`, the
Mission tab and `nervros-cli missions` read it.

### `[[place]]`

Named places, beyond the world model's rooms. The operator can add more from where the robot
stands ("remember this spot as the reading corner"): `tag_place` keeps them beside the quota ledger,
one JSON file per robot, and `forget_place` drops them. Neither changes this file, and a remembered
place cannot take a name the profile already uses.

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

### `[models]`

`file`: the [models file](models.md).

### `[[mcp_server]]`

Tools from an MCP server, behind the same guard as every other tool. Only the tools listed exist
for the agent, and each only while its definition is the one approved: `nervros-cli mcp pin` keeps
each tool's definition, by a hash of its canonical form and its text, in `mcp.lock.json` beside the
profile, and a tool whose definition changed is left out until it is pinned again. A tool acts,
with arming and approval, unless the profile marks it `observe`; the server's own hints are not
believed. What a tool returns is cut to a size and fenced with the server and tool it came from.

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

Every robot gets `list_places`, `robot_state` and `stop`. `stop` is always allowed, armed or not,
because it only makes the robot do less. `look` comes with `[look]`, `segment` with `[segment]`,
and `run_mission` with `[mission]`.

The rest are the checks you would run yourself before blaming the model:

| Tool | What it does |
|---|---|
| `health_check` | The connection check, plus each camera's frame rate, whether the robot knows where it is on the map, and the executor's state. Returns what is wrong and what is fine. |
| `watch` | Watches a topic in the background and posts to the chat when a condition holds: its rate drops below a floor, a field crosses a value, or a text field matches. Once, or each time it comes back; at most 8 at a time, all ended with the session. |
| `watches` | Lists the running watches, or cancels one or all. |
| `plot` | Draws a number from a topic's messages over time in the app's Plots tab, as `rqt_plot` does, for two minutes unless told longer. |
| `memory` | Keeps what the operator asks it to remember across sessions ("the kitchen door sticks"), in the state directory, one file per robot; lists and forgets notes. The notes join the system prompt on every turn, and the Agent tab lists them. Remembering and forgetting are approved like an edit. |
| `schedule` | With `[mission]`: runs a plan again and again, such as a patrol every 30 minutes, a set number of times, or each time something happens: an object of a kind appears, in a room or anywhere, or a condition on a topic as `watch` takes it becomes true. The operator approves it once for all its runs; a run is skipped while the robot is disarmed or busy, and stopping the robot cancels every schedule. The Mission tab lists them. |
| `recall` | With `[mission]`: what the robot did lately, from the mission ledger, what happened to an object, from `[world] history`, and the operator's notes. |
| `plans` | With `[mission]`: saves a plan that worked by name ("evening check"), lists them, and runs one again. |
| `skill_gap` | With `[mission]`: logs a request no skill can do, and why; planning that gives up logs one itself. The Mission tab and `nervros-cli gaps` list them. |
| `point` | With a `segment` model in the models file: points at what the operator names in a camera's newest frame, including things no detector marks, and says which mark a point lands on. |
| `skill` | With `skills`: reads one skill's procedure. |

`look` can also answer about an earlier snapshot, and look closer at one mark: it crops the mark
from the full frame at a higher resolution for the vision model, for a label or a small part.
| `tag_place`, `forget_place` | Remembers where the robot stands, and which way it faces, as a named place; forgets one. Each asks for approval when supervised, like a world edit. |

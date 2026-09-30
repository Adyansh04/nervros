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
```

The model sees the service's comment as the tool's description and gets a JSON Schema of the
request made from the `.srv` file: field comments become descriptions, constants become enums, and
byte arrays are hidden. The interface files are found through `[ros] interfaces`. A `topic` tool
returns the newest message on the topic. No tool can publish.

| Key | Default | |
|---|---|---|
| `name` | required | The name the model calls. |
| `kind` | required | `service` or `topic`. |
| `ros_name` | required | The service or topic. |
| `type` | required | `pkg/srv/Name` or `pkg/msg/Name`. |
| `description` | from the interface | Replaces the interface file's comment. |
| `risk` | `observe` | `observe`, `world_edit`, `motion` or `manipulation`. Anything but `observe` is an act: it needs the robot armed and, when supervised, the operator's approval. |
| `resources` | none | `base`, `left_arm`, `right_arm`: two calls holding the same one cannot run at once. |
| `timeout` | `"5s"` | Per call. |
| `hide_fields` | none | Request fields the model neither sees nor fills, such as an embedding. |
| `defaults` | none | Values the request starts from; the model's arguments go on top. |
| `schema` | generated | A full JSON Schema to use instead. |

A name on the policy's `hard_deny` list cannot become a tool at all: the start fails.

## Sections

### `[robot]`

| Key | Default | |
|---|---|---|
| `name` | required | Shown in the app and told to the model. |
| `persona` | none | A markdown file about the robot, its abilities and limits, added to the system prompt. |

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
| `budgets.model_calls` | `10` | Model calls per turn. Once the robot has acted in a turn, the model is offered no more tools and answers. |
| `budgets.tool_calls` | `12` | Tool calls per turn. |
| `budgets.wall_time` | `"90s"` | A turn is stopped after this, not counting the operator's time on approvals. Missions run outside turns. Free cloud endpoints can take 20 s a call; give them `"180s"`. |
| `budgets.repeat_break` | `3` | The same call with the same arguments this many times in a row is refused. |
| `hard_deny` | see below | ROS names no tool may reach; `*` matches anything. Setting it replaces the defaults. |

The default `hard_deny` covers what commands motors, velocities or controllers directly:
`rt/lowcmd`, `/lowcmd`, `/cmd_vel`, `*/cmd_vel`, `/controller_manager/*`, `*/set_parameters`,
`*/set_parameters_atomically`, `*/joint_trajectory`, `*/follow_joint_trajectory`, `/servo_node/*`,
`/apply_planning_scene` and `/clear_octomap`.

### `[look]`

The `look` tool: a camera frame with the current detections drawn on it as numbered marks, which
the model and the operator both see.

| Key | Default | |
|---|---|---|
| `image` | required | A `sensor_msgs/msg/Image` topic. |
| `detections` | required | `{ topic, type }`, where the type is `canopy_msgs/msg/InstanceMaskArray` or `vision_msgs/msg/Detection2DArray`. |
| `max_marks` | `12` | |
| `max_age` | `"5s"` | Older detections are left out. |

### `[world]`

The world model's topics, each `{ topic, type }` and each optional: `map` (a
`nav_msgs/msg/OccupancyGrid`), `rooms` and `objects` (canopy's `RoomArray` and
`WorldObjectArray`, or messages with the same fields), `coverage` (an `OccupancyGrid` of what the
camera has seen, canopy's `/canopy/coverage`: 0 seen, 90 still to see, 99 written off) and `trail`
(a `nav_msgs/msg/Path` of where the robot has been). The viewer draws them all, rooms coloured by
how much of each the camera has seen, and plots that share with the room and object counts;
`list_places`, `robot_state` and missions read rooms and objects. The app's World tab lists the
rooms with their floor and walls seen, and its Explore button asks the agent to explore.

### `[mission]`

The robot's mission executor, which runs behaviour trees; see
[the executor contract](executor.md). With this section the agent gets `plan_mission` and
`run_mission`.

| Key | Default | |
|---|---|---|
| `execute` | required | The `ExecuteMission` action. |
| `validate` | required | The `ValidateMission` service. |
| `catalog` | required | The `GetCatalog` service. |
| `stop` | required | The `StopAll` service, which the Stop button and `stop` call. |
| `state` | required | The `RobotState` topic. |
| `max_replans` | `2` | Failed missions for one request before the agent must hand back to the operator. |

### `[[place]]`

Named places, beyond the world model's rooms.

```toml
[[place]]
name = "dock"
aliases = ["the charging dock"]
pose = { x = 0.0, y = 0.0, yaw = 0.0 }   # in frame, default "map"
```

### `[models]`

`file`: the [models file](models.md).

## Builtin tools

Every robot gets `list_places`, `robot_state` and `stop`. `stop` is always allowed, armed or not,
because it only makes the robot do less. `look` comes with `[look]`, and `plan_mission` and
`run_mission` come with `[mission]`.

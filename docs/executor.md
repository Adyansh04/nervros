# The mission executor contract

NervROS never moves a robot directly. Motion goes through missions: behaviour trees that the
agent compiles, the robot's executor checks, the operator approves by hash, and the executor
runs. The executor is the robot's own node, in C++ with BehaviorTree.CPP 4 in the reference
implementation (grove-g1's `nervros_executor`). This page is what an executor must provide.
The interfaces are in [`ros/nervros_interfaces`](../ros/nervros_interfaces), and the profile's
`[mission]` section names them.

## The executor is the safety boundary

The agent and its model are not trusted. The executor checks every tree itself before it runs
it, and turns it away when:

- its SHA-256 is not the hash sent with it. `execute` always needs the hash, so an empty one fails;
- it uses `<include>`, or defines any tree other than `Mission`: a document could otherwise
  redefine a library macro;
- its `<TreeNodesModel>`, if it has one, is anything but exact copies of the library's SubTree
  entries, since a model entry can give a macro's port a default;
- it uses a node outside the allowlist: the control set (`Sequence`, `Fallback`,
  `RetryUntilSuccessful`, `Timeout`, `ForceSuccess`) and `SubTree` references to catalog macros.
  The leaves inside the macros are the executor's own and never appear in a mission;
- a node carries an attribute starting with `_` (pre- and post-conditions, `_autoremap`), or an
  argument is missing, unknown, longer than 200 characters, not one of its `enum` values, or holds
  a `{blackboard}` reference;
- a `Timeout` is not a positive whole number of milliseconds, or a retry is outside 1 to 3
  attempts;
- it is too big, or its worst case (the sum of its steps' limits times their attempts) is over
  the executor's cap. The reference executor allows 131072 bytes, 10 levels, 200 nodes, 32 steps
  and 1800 s; a longer task is sent as more than one mission.

## Interfaces

| Interface | Type | What it does |
|---|---|---|
| `execute` | `ExecuteMission` action | Runs one mission at a time. `mode` also allows validating without running, and a dry run in which every skill is a timed stand-in that moves nothing. |
| `validate` | `ValidateMission` service | Everything `execute` checks, without running: diagnostics written for a model to act on, the tree's hash and its worst-case duration. |
| `catalog` | `GetCatalog` service | The skill catalog as JSON (below), the TreeNodesModel XML of the leaves and the macros, and the BehaviorTree.CPP version. |
| `stop` | `StopAll` service | Halts the tree, cancels every goal on every action server the skills use, whoever sent it, and holds posture while the hands keep their grip. The reference executor also cancels the arm trajectory controller's goal, which holds the arm where it is, and leaves the hand controllers alone. Safe to call at any time, with or without a mission. |
| `state` | `RobotState` topic | Transient local, on change and at 1 Hz: the running mission and step, the resources held, what each hand holds, whether a stop is in force, and whether the robot can walk (its tilt, and why not when it cannot). |
| `preview` | `PreviewMission` service | Optional. Where each step of a tree would end the robot and the path Nav2's planner finds there, chained from the robot's pose now, with a note when there is none; only the planner runs. The app draws it beside the plan's approval card. |
| `heartbeat` | `Heartbeat` topic | Optional. The agent's heartbeats while a mission runs; see the deadman below. |
| `teleop` | `Teleop` service | Optional. Hands the base to the operator: while on, the executor refuses missions and forwards each `geometry_msgs/Twist` on its `~/teleop_cmd` within its speed limits, stops the base when they pause, and `StopAll` ends it. |

## Running a mission

Every goal is accepted, so that a refusal can say why: a second mission while one runs ends at
once as `REJECTED` with the diagnostic `BUSY`, and so does any mission once the executor is
shutting down (`SHUTTING_DOWN`). A tree that fails its checks ends as `REJECTED` too, with the
diagnostics in `diagnostics_json`.

| Outcome | When |
|---|---|
| `SUCCESS` | The tree succeeded. |
| `FAILURE` | The tree failed. `failed_node` is the path of the leaf that failed last before the tree gave up; `failure_reason` is the skill server's own words. |
| `TIMEOUT` | A step's `Timeout` ran out (`failed_node` is its path), or the mission's watchdog fired. |
| `CANCELED` | The client cancelled the goal, `StopAll` stopped it (`failure_reason` starts with "stopped:"), or the executor shut down. `failed_node` names the step that was running. |
| `REJECTED` | The tree failed its checks, or the executor was busy or shutting down. |
| `ERROR` | The executor could not run it, for instance because the arms could not be taken. |

`failed_step_id` is the step the failed node belongs to, such as `s2`. The goal's own status is
`CANCELED` only when the client cancelled it; any other outcome but success arrives as aborted.
The watchdog is the goal's `max_duration_s`, or the tree's worst case times 1.2, and never more
than the executor's cap. Feedback carries the running nodes and the transitions since the last
message, for the mission's steps and the skills' leaves.

## Deadman and health

A goal may carry `heartbeat_timeout_s` and `heartbeat_client`: the executor then stops the mission,
as `StopAll` would, when no `Heartbeat` from that client has arrived for that long. It clamps the
timeout to its own limit, and counts only that client's beats, so another agent cannot keep a
mission alive. NervROS beats from its session's own loop, the one that serves Stop, so an agent
that crashed or hung stops beating too.

The executor refuses to start a mission that moves the base when the robot cannot walk, and stops
one that is running when that changes: the reference executor reads the body's tilt from the IMU
and whether the balance controller is active, with some hysteresis, and says why in `RobotState`.

## Arms and hands

- The executor takes the arms before the first tick when a skill needs them, and hands them back
  when the mission ends: after success, failure, a cancel, a stop or a shutdown.
- Except when a hand holds something. A released hand goes limp and drops what it holds, so the
  arms stay held, and `RobotState` says what each hand keeps. A stop never opens a hand.
- A pick that is cut off may have closed the hand before it stopped. The executor then does not
  know what that hand holds: `holding_<arm>` stays empty, `message` says "<arm> hand unknown",
  and the hand counts as full until a place with it succeeds. The planner treats it as full.

## A mission

```xml
<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission">
    <Timeout msec="1296000">
      <Sequence name="mission">
        <Timeout msec="240000"><SubTree ID="GoToTarget" name="s1_GoToPlace" target="O17"/></Timeout>
        <RetryUntilSuccessful num_attempts="2"><Timeout msec="420000"><SubTree ID="PickObject" name="s2_PickObject" object_id="O17" phrase="red mug" arm="right"/></Timeout></RetryUntilSuccessful>
      </Sequence>
    </Timeout>
  </BehaviorTree>
</root>
```

- Each step is a `SubTree` reference to a macro in the executor's own library, which it registers
  in every mission's factory. The agent never sends macro bodies.
- Every argument is a literal attribute.
- A step is wrapped in its `Timeout`, and in `RetryUntilSuccessful` and `ForceSuccess` when the
  plan asks for them. The whole mission is wrapped in a `Timeout` of its worst case times 1.2:
  here (240 + 2 × 420) s × 1.2.
- Step nodes are named `s<N>_<Skill>`. Feedback paths start with that name, which is how the
  agent maps transitions onto steps. Transitions back to idle are resets, which it ignores.

## The catalog

`GetCatalog.catalog_json`:

```json
{"catalog_version": "b2ab798877fc",
 "skills": [{
   "name": "PickObject",
   "description": "Pick an object up with one hand ... (for a language model)",
   "args": [
     {"name": "object_id", "type": "string", "description": "...", "enum": []},
     {"name": "phrase", "type": "string", "description": "...", "enum": [], "default_from": "label(object_id)"},
     {"name": "arm", "type": "string", "description": "Which hand", "enum": ["left", "right"]}],
   "requires": ["near(object_id)", "hand_empty(arm)"],
   "effects": ["holding(arm, object_id)"],
   "resources": ["base", "left_arm", "right_arm"],
   "idempotent": false,
   "risk": "manipulation",
   "max_duration_s": 420,
   "template": "PickObject"}]}
```

- **Arguments.** `type` is `world_id` when the value must be an id in the world model, which the
  planner checks against the live one, and `string` otherwise. `enum` lists the only values
  allowed. `default_from: "label(<arg>)"` lets the planner fill the value with the world model's
  label for what the other argument names, or with the words of an id it does not know
  (`red_block` gives "red block"). The model never sees such an argument, but the mission still
  carries it.
- **Needs and effects.** The planner understands these predicates in `requires` and `effects`:
  - `near(x)`: within about half a metre of object or room `x`, judged only for ids the world
    model knows;
  - `holding(arm, x)`: a hand holds `x`, where a capital letter means any object;
  - `hand_empty(arm)`.

  It follows the plan from the robot's current state. It adds a walk before a step that needs
  the robot near a known object, forgets where the robot is after a skill that takes the base,
  and rejects a step whose hand is wrong, with the fix spelled out. Other predicates are the
  executor's to check.
- **Navigation.** An executor that has `GoToPose(station)`, where the station is `x;y;yaw` in
  the map frame, or `GoToTarget(target)`, a room or object id, lets plans write
  `GoToPlace(place)`. The planner picks the macro from what the place is. The model never names
  these two.
- **`catalog_version`** changes with the catalog and with the node model, so an approval refers
  to a known catalog.

## Diagnostics

`diagnostics_json` is an array of `{code, line, node, message}`, the message written for a model
to fix the tree by. The codes: `TOO_LARGE`, `HASH_MISMATCH`, `INCLUDE_FORBIDDEN`, `XML_SYNTAX`,
`BAD_ROOT`, `BAD_STRUCTURE`, `NO_MISSION_TREE`, `DUPLICATE_TREE`, `EXTRA_TREE`, `MODEL_MISMATCH`,
`FORBIDDEN_NODE`, `INTERNAL_LEAF`, `UNKNOWN_NODE`, `FORBIDDEN_ATTRIBUTE`, `UNKNOWN_ATTRIBUTE`,
`MISSING_ATTRIBUTE`, `BAD_NAME`, `DUPLICATE_NAME`, `BAD_TIMEOUT`, `BAD_RETRIES`, `BAD_CHILDREN`,
`UNKNOWN_MACRO`, `UNKNOWN_ARG`, `MISSING_ARG`, `BAD_ARG`, `TOO_DEEP`, `TOO_MANY_NODES`,
`TOO_MANY_STEPS`, `TOO_LONG`, `TOO_MANY_PROBLEMS`, `LOAD_ERROR`, `BUSY`, `SHUTTING_DOWN` and
`BAD_MODE`.

## Goal checks

After a mission succeeds, the agent checks the plan's `goal` against the robot and the world
model, and tells the model which predicates hold:

- `at(place)`;
- `holding(arm, object)`, from `RobotState`;
- `inside(object, container)` and `on(object, surface)`, from the world model's `support_id` or
  the container's footprint.

# The mission executor contract

NervROS never commands motors or velocities. The robot moves through missions: behaviour trees
that the agent compiles, the robot's executor checks, the operator approves by hash, and the
executor runs. Any other act tool a profile lists, such as a Nav2 goal through `action_goal`, needs
the robot armed and the operator's approval.

The executor is the robot's own node. The reference implementation, grove-g1's
`nervros_executor`, is C++ with BehaviorTree.CPP 4. This page is what an executor must provide. The
interfaces are in [`ros/nervros_interfaces`](../ros/nervros_interfaces), and the profile's
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
  a `{blackboard}` reference that is not an earlier step's output of a fitting kind (below);
- a `Timeout` is not a positive whole number of milliseconds, or a retry is outside 1 to 3
  attempts;
- it is too big, or its worst case (the sum of its steps' limits times their attempts) is over
  the executor's cap. The reference executor allows 131072 bytes, 10 levels, 200 nodes, 32 steps
  and 1800 s; a longer task is sent as more than one mission.

## Interfaces

| Interface | Type | What it does |
|---|---|---|
| `execute` | `ExecuteMission` action | Runs one mission at a time. `mode` also allows validating without running, and a dry run in which every skill is a timed stand-in that moves nothing. |
| `validate` | `ValidateMission` service | The tree checks `execute` makes, without running it and without the hash, busy, hand-driving and health checks: diagnostics written for a model to act on, the tree's hash and its worst-case duration. |
| `catalog` | `GetCatalog` service | The skill catalog as JSON (below), the TreeNodesModel XML of the leaves and the macros, and the BehaviorTree.CPP version. |
| `stop` | `StopAll` service | Halts the tree, cancels every goal on every action server the skills use, whoever sent it, and holds posture while the hands keep their grip. The reference executor also cancels the arm trajectory controller's goal, which holds the arm where it is, and leaves the hand controllers alone. Safe to call at any time, with or without a mission. |
| `state` | `RobotState` topic | Transient local, on change and at 1 Hz: the running mission and step, whether it is paused, the resources held, what each hand holds, whether a stop is in force, whether the base is driven by hand, and whether the robot can walk (its tilt, and why not when it cannot). |
| `preview` | `PreviewMission` service | Optional. Where each step of a tree would end the robot and the path Nav2's planner finds there, chained from the robot's pose now, with a note when there is none; only the planner runs. The app draws it beside the plan's approval card. |
| `pause` | `PauseMission` service | Optional. Pauses the running mission, or resumes it; see below. |
| `heartbeat` | `Heartbeat` topic | Optional. The agent's heartbeats while a mission runs; see the deadman below. |
| `teleop` | `Teleop` service | Optional. Hands the base to the operator: while on, the executor refuses missions and forwards each `geometry_msgs/Twist` on its `~/teleop_cmd` within its speed limits, stops the base when they pause, and `StopAll` ends it. |
| `define_skill` | `DefineSkill` service | Optional. Takes a skill an agent teaches; see below. |
| `forget_skill` | `ForgetSkill` service | Optional. Forgets a taught skill. |

## Running a mission

Every goal is accepted, so that a refusal can say why. A mission ends at once as `REJECTED` with a
diagnostic when:

- another mission is running (`BUSY`),
- the executor is shutting down (`SHUTTING_DOWN`),
- the base is driven by hand (`TELEOP_ACTIVE`),
- the robot cannot walk or stand for what the mission needs (`ROBOT_CANNOT_MOVE`).

A tree that fails its checks ends as `REJECTED` too, with the diagnostics in `diagnostics_json`.

| Outcome | When |
|---|---|
| `SUCCESS` | The tree succeeded. |
| `FAILURE` | The tree failed. `failed_node` is the path of the leaf that failed last before the tree gave up; `failure_reason` is the skill server's own words. |
| `TIMEOUT` | A step's `Timeout` ran out (`failed_node` is its path), or the mission's watchdog fired. |
| `CANCELED` | The client cancelled the goal, `StopAll` stopped it (`failure_reason` starts with "stopped:"), a pause ran past the executor's limit ("paused for longer than ..."), or the executor shut down. `failed_node` names the step that was running. |
| `REJECTED` | The tree failed its checks, or the executor was busy, shutting down or driven by hand, or the robot could not move. |
| `ERROR` | The executor could not run it, for instance because the arms could not be taken. |

- `failed_step_id` is the step the failed node belongs to, such as `s2`.
- The goal's own status is `CANCELED` only when the client cancelled it. Any other outcome but
  success arrives as aborted.
- The watchdog is the goal's `max_duration_s`, or the tree's worst case times 1.2, and never more
  than the executor's cap. Time paused does not count.
- Feedback carries the running nodes and the transitions since the last message, for the
  mission's steps and the skills' leaves, and whether the mission is paused.

## Pause and resume

`PauseMission` with `pause: true` halts the running step. Its goals are cancelled, and a goal
accepted after the halt is asked again. The reference executor also cancels the arm trajectory
controller's goal, as a stop does, so the base stops, the arms hold where they are, and the hands
keep their grip.

- A step that handles an object (`risk: manipulation`, a pick or a place) is allowed to finish
  first, and the pause takes the next step before it sends anything.
- While paused, the executor ticks nothing and keeps the authority it holds. It still stops for
  `StopAll`, a cancel, the deadman and the robot's health.
- `RobotState` and the feedback say `paused` once the robot has stopped. While a pick or a place
  finishes they do not, and `RobotState`'s `message` says what it waits for.

`pause: false` resumes. The halted step starts again from its beginning, so a walk to a place sets
out again, while the reference executor's straight walk and turn do only what was left of them.
The watchdog does not count the pause, and gives back the time the halted step had run, since that
step runs again. A pause longer than the executor's limit (600 s in the reference executor) ends
the mission.

NervROS pauses on a plain "pause", "wait" or "hold on" while a mission runs, and on the window's
Pause button. It resumes on "resume", "continue", "carry on" or "go on".

## Deadman and health

A goal may carry `heartbeat_timeout_s` and `heartbeat_client`. The executor then stops the mission,
as `StopAll` would, when no `Heartbeat` from that client has arrived for that long.

- It clamps the timeout to its own limit, and 0 turns the deadman off.
- It counts only that client's beats (or every client's when none is named), so another agent
  cannot keep a mission alive.
- NervROS beats from its session's own loop, the one that serves Stop, so an agent that crashed or
  hung stops beating too.

The executor refuses to start a mission that moves the base when the robot cannot walk, and stops a
running one when that changes. The reference executor reads the body's tilt from the IMU and
whether the balance controller is active, with some hysteresis, and says why in `RobotState`.

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
    <Sequence name="mission">
      <Timeout msec="240000"><SubTree ID="GoToTarget" name="s1_GoToPlace" target="red mug" target_id="{s1_target_id}"/></Timeout>
      <RetryUntilSuccessful num_attempts="2"><Timeout msec="420000"><SubTree ID="PickObject" name="s2_PickObject" object_id="{s1_target_id}" phrase="red mug" arm="right"/></Timeout></RetryUntilSuccessful>
    </Sequence>
  </BehaviorTree>
</root>
```

- Each step is a `SubTree` reference to a macro in the executor's own library, which it registers
  in every mission's factory. The agent never sends macro bodies.
- Every argument is a literal attribute, or what an earlier step hands on. A step gives an output
  under its own step's key, `target_id="{s1_target_id}"`, and a later step takes that key as an
  argument. Here the walk goes to the nearest red mug and the pick takes the one it went to. The
  executor checks that:
  - the key is written once, by a step before it in the document that must run to its end (not
    under a `ForceSuccess`, nor in a `Fallback` branch the taking step is not in);
  - the output fits the argument: an id from the world model for a `world_id`, an id for an `id`,
    the same format otherwise, and never a catalog `enum`.

  The planner likewise refuses an answer from an `optional` step.
- A step is wrapped in its `Timeout`, and in `RetryUntilSuccessful` and `ForceSuccess` when the
  plan asks for them. The mission itself has no `Timeout`: the executor's watchdog, its worst case
  times 1.2, is its limit, and that does not run down while it is paused.
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
     {"name": "object_id", "type": "string", "format": "id", "description": "...", "enum": []},
     {"name": "phrase", "type": "string", "format": "phrase", "description": "...", "enum": [], "default_from": "label(object_id)"},
     {"name": "arm", "type": "string", "description": "Which hand", "enum": ["left", "right"]}],
   "outputs": [],
   "requires": ["near(object_id)", "hand_empty(arm)"],
   "effects": ["holding(arm, object_id)"],
   "resources": ["base", "left_arm", "right_arm"],
   "idempotent": false,
   "risk": "manipulation",
   "max_duration_s": 420,
   "template": "PickObject"}]}
```

- **Arguments.**
  - `type` is `world_id` when the value must be an id in the world model, which the planner checks
    against the live one, and `string` otherwise. A name ending in `_id` with no `type` counts as a
    `world_id`.
  - `enum` lists the only values allowed. A single value fills the argument when a plan leaves it
    out, ahead of any `default_from`.
  - `default_from: "label(<arg>)"` lets the planner fill the value with the world model's label for
    what the other argument names, or with the words of an id it does not know (`red_block` gives
    "red block"). The model never sees such an argument, but the mission still carries it.
  - `format` (`id`, `phrase`, `station`, `number`, the last with `min` and `max`) is how the
    executor checks the value. A `world_id` with `phrase` also takes plain words the world model
    searches for.
  - `narrowed: true` marks an argument that had no list and that the robot narrowed to what it can
    do where it is, such as the only names a simulated detector knows. What an earlier step found
    on the robot may stand in for one, though not a world model's id.
  - `max_duration_s` must be positive, and `template` names the macro.
- **Outputs.** `outputs` lists what a skill hands on, typed like an argument; plans give the
  step an `id` and a later argument the whole value `{id.output}`, which the planner compiles to
  the step's key. The reference executor's `GoToTarget` hands on `target_id`, and `FindInView`
  the `object_id` the detector reports what it found under. `ExecuteMission`'s result carries
  what the steps handed on as `outputs_json`, `{"s2_object_id": "mug"}`, which programs and
  searches read.
- **Needs and effects.** The planner understands these predicates in `requires` and `effects`:
  - `near(x)`: within about 0.6 m of object or room `x`, judged only for ids the world model
    knows;
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

## Taught skills

An executor may let an agent teach it skills. `GetCatalog`'s JSON then carries `learnable`: the
leaves a taught skill may be built from, each with the authority it takes (`base`, `arms` or
none) and its inputs, and the most such a skill may take and hold:

```json
"learnable": {"leaves": {"MoveHand": {"holds": "arms", "ports": {"arm": "'left' or 'right'.", "target": "...", "frame": "..."},
                                      "one_of": [["target", "frame"]], "world_ids": ["target"]},
                         "Wait": {"holds": "", "ports": {"seconds": "How long, 0 to 60 s."}, "world_ids": []}},
              "max_duration_s": 300.0, "max_nodes": 40}
```

Two optional rules on a leaf's inputs:

- `one_of`: groups of inputs of which a body sets exactly one, such as a target in the world model
  or a frame of the robot.
- `world_ids`: the inputs that take a world-model id. When present, a skill's `world_id` argument
  may feed only these, so an object's id never goes in as a frame.

NervROS checks both before the operator is asked to approve the teaching; the executor checks them
again.

The leaves are the executor's choice, and the boundary: each must check every goal it sends
itself, since a planner's arrangement of them is only as safe as they are. The reference executor
offers Nav2's walk, its own bounded walk and turn, the detector's look, named arm postures,
`MoveHand` (one hand to a point, or pointing at one, planned by MoveIt against the scene) and
`Wait`.

`DefineSkill` takes a name, a description, the arguments as the catalog writes them, the body as
one behaviour-tree element, the most it may take, and the request it was taught for.

- The executor accepts only the plain control nodes and the learnable leaves, with each attribute
  a literal or `{arg}` (one of the skill's own arguments), within its size limits.
- It puts the body under a Timeout of the skill's time, and gives the skill the authority its
  leaves take. The diagnostics come back as `ValidateMission`'s do.
- `keep: false` serves the skill as a draft until the executor restarts. `keep: true` keeps it on
  disk, where it loads with the executor and is checked again each time.
- Taught skills carry `learned` (`draft` or `kept`) and `taught_for` in the catalog, and the
  catalog's version changes with them.
- A mission is checked and built against the skills as they were when it started, whatever is
  taught meanwhile. `ForgetSkill` removes a skill.

NervROS teaches a draft with `teach_skill`, and keeps it when the first mission that runs it
succeeds; a kept skill that fails more often than not over its last four runs it forgets.

## Diagnostics

`diagnostics_json` is an array of `{code, line, node, message}`, the message written for a model
to fix the tree by. The codes: `TOO_LARGE`, `HASH_MISMATCH`, `INCLUDE_FORBIDDEN`, `XML_SYNTAX`,
`BAD_ROOT`, `BAD_STRUCTURE`, `NO_MISSION_TREE`, `DUPLICATE_TREE`, `EXTRA_TREE`, `MODEL_MISMATCH`,
`FORBIDDEN_NODE`, `INTERNAL_LEAF`, `UNKNOWN_NODE`, `FORBIDDEN_ATTRIBUTE`, `UNKNOWN_ATTRIBUTE`,
`MISSING_ATTRIBUTE`, `BAD_NAME`, `DUPLICATE_NAME`, `BAD_TIMEOUT`, `BAD_RETRIES`, `BAD_CHILDREN`,
`UNKNOWN_MACRO`, `UNKNOWN_ARG`, `MISSING_ARG`, `BAD_ARG`, `TOO_DEEP`, `TOO_MANY_NODES`,
`TOO_MANY_STEPS`, `TOO_LONG`, `TOO_MANY_PROBLEMS`, `LOAD_ERROR`, `BUSY`, `SHUTTING_DOWN`,
`TELEOP_ACTIVE`, `ROBOT_CANNOT_MOVE` and `BAD_MODE`.

## Goal checks

After a mission succeeds, the agent checks the plan's `goal` against the robot and the world
model, and tells the model which predicates hold:

- `at(place)`;
- `holding(arm, object)`, from `RobotState`;
- `inside(object, container)` and `on(object, surface)`, from the world model's `support_id` or
  the container's footprint.

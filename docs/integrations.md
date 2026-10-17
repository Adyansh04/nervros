# Integrations

NervROS works with the ROS stacks a robot runs, Nav2 and MoveIt today, through integrations: one
module each under `crates/nervros-core/src/integrations/`, switched on by its profile section
(`[nav2]`, `[moveit]`; see [the profile](profile.md)). Nothing outside that directory names a
stack, so a robot that runs Nav2 and MoveIt needs a profile, not code, and another stack can be
added the same way.

An integration gives the agent four things:

- **Tools** for reads and for changes to how the stack behaves, called over ROS directly: Nav2's
  `nav_status`, `nav_check`, `collision_monitor` and `nav_admin`, the zones' `geofence`; MoveIt's
  `arm_status`, `arm_check`, `scene` and `arm_settings`.
- **A status line** in every turn, so the model knows what the stack is doing before it is asked.
- **Native steps**: standard motions NervROS runs itself as mission steps (below).
- **Stop**: the Stop button and `stop` cancel every goal the integration sent; MoveIt is also told
  to stop any trajectory it runs. Hands keep their grip.

## What changes the robot's behaviour asks first

| Free | Asks the operator (may be allowed for the session) | Always asks |
|---|---|---|
| Adding a slow zone or a keep-out zone, slowing one down | Removing a slow zone, raising its speed | Removing a keep-out zone |
| The collision monitor or a polygon on, a polygon grown | The monitor or a polygon off, a polygon shrunk | |
| Adding a scene obstacle, slowing the arms | Removing an obstacle, attaching or detaching an object, clearing the octomap or the costmaps, raising the arms' speed, another planner | |

Localization, the navigation servers' lifecycle and maps (`nav_admin`) ask, and none runs during a
mission. No read disturbs a motion: a path check is refused while Nav2 walks, since a
`ComputePathToPose` request cuts into the navigator's own, and arm previews use MoveIt's plan
service, never its move action. NervROS reshapes only the collision polygons the robot hands it,
beside the robot's own fixed stop polygon.

## Native steps

A mission may hold steps NervROS runs itself beside the robot executor's skills:

| Stack | Steps |
|---|---|
| Nav2 | `GoToPose`, `GoToTarget` (with a world model), `WalkRoute` (through places without stopping), `TurnInPlace`, `WalkStraight` (forward or back), `Wait`, `Dock` and `Undock` (with `[nav2] docking`) |
| MoveIt | `ArmTo` (a named or remembered pose), `HandTo` (just above an object, or a point), `HandLine` (a short straight hand move), `Grip` (a hand's named state) |

They take the names an executor's skills would. A skill of the same name in the executor's catalog
is used instead: on the G1 the executor walks, turns and moves the arms itself, and NervROS runs
only routes, waits and straight hand moves. A robot with no executor at all plans with native
steps only.

A composite executor (`mission/composite.rs`) stands between the missions and the robot's
executor, so planning, approval, the plan checks, the ledger, track records and goal checks work as
they do for executor skills:

- The catalog is the executor's with the native steps it lacks. A plan of executor steps only goes
  to the executor unchanged, tree and hash.
- A mixed plan runs stretch by stretch: each run of executor steps is a mission of its own on the
  executor, with what earlier steps handed on filled in; each native step runs here with its
  timeout and retries. The operator approves the whole plan once.
- Stop ends a native step as cancelled and its goal with it, and calls the executor's `StopAll`.
  Pause stops a walk, a route, a turn or a straight walk and sends what was left once resumed; an
  arm move finishes first, as a pick does; a wait does not count down while paused.
- The plan check holds native steps to their catalog entries (enums, number ranges, `x;y;yaw`) and
  refuses one whose action server is not running. Routes are checked against keep-out zones.

On a robot whose executor holds the arms, an arm step borrows them through the executor's
`LendResources` (`[mission] lend`) and gives them back after it, also when the step is cut short.
The executor refuses while a mission runs or the base is driven by hand, and takes the arms back on
`StopAll`, when NervROS's heartbeat stops, or when the time it lent them for runs out. A hand that
holds something keeps its arm held.

If NervROS dies with a native goal open, [`nervros_watchdog`](../ros/nervros_watchdog/README.md)
on the robot cancels it: NervROS sends its open goals on `[ros] open_goals` twice a second while
any is open, and the watchdog cancels each by id once they stop coming.

## Adding a stack

1. A module beside `nav2.rs` and `moveit.rs` with a config struct for its profile section, added
   to `Profile` and documented in [the profile](profile.md).
2. A type implementing `Integration`: `id`, `tools`, `status`, `stop`, and if it draws or runs
   motions, `attach` and `natives`. Start it in `Integrations::start` when its section is there,
   and add what it finds on the graph to `discover` for `nervros-cli doctor`.
3. Its tools follow the rules above: `assess` says what a call changes, with `Risk::WorldEdit` for
   a change that loosens what keeps the robot safe and `always_asks` for one never allowed for the
   session.
4. Its native steps implement `NativeSkill`: a catalog entry, the actions it needs on the graph,
   and `run`, which sends goals through `run_goal` so Stop, Pause, the watchdog and timeouts reach
   them.

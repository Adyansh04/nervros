# nervros_interfaces

The ROS 2 contract between the NervROS agent and a robot's mission executor. A robot that wants
NervROS to drive it implements these on its executor node. The G1 implementation lives in
grove-g1's `g1_orchestration`.

| Interface | Kind | What it does |
|---|---|---|
| `ExecuteMission` | action | Run one compiled, validated and approved behaviour tree. The hash must match the approved XML. Feedback carries node status changes, and the result names the failed step, the skill's own reason and what the steps handed on |
| `ValidateMission` | service | Load and check a tree as `ExecuteMission` would, without running it and without the hash, busy and health checks |
| `GetCatalog` | service | The robot's skills, with args, requires, effects, resources, risk and durations, which ones were learned, and the leaves a learned skill may use, plus BehaviorTree.CPP's node model |
| `DefineSkill` | service | Teach the robot a skill built from the learnable leaves: checked, then served as a draft or kept on disk |
| `ForgetSkill` | service | Forget a learned skill |
| `StopAll` | service | Halt the mission, cancel outstanding goals, wait for them, hold posture. A hand that holds an object keeps its grip |
| `PauseMission` | service | Pause the running mission (the base stops, the arms hold, the hands keep their grip; a pick or place finishes first) or resume it, re-running the halted step |
| `RobotState` | message | What the robot holds and does: mission, step, whether it is paused, resources held, object in each hand |
| `NodeEvent` | message | One behaviour-tree node's status change |
| `PreviewMission` | service | Where each step of a tree would leave the robot and the path there, without running it; one `StepPreview` per step |
| `Teleop` | service | Hand the base to the operator for driving by hand, or take it back |
| `Heartbeat` | message | The agent's heartbeat while a mission runs, for the executor's deadman |
| `LendResources` | service | Lend the arms (or the base) the executor holds to the agent for a motion it runs itself through MoveIt or Nav2, and take them back; refused while busy, and taken back on a heartbeat loss, StopAll or the time limit |
| `OpenGoals` | message | The action goals the agent's own motions have open, for `nervros_watchdog` to cancel if the agent goes quiet |

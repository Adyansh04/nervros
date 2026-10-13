# nervros_interfaces

The ROS 2 contract between the NervROS agent and a robot's mission executor. A robot that wants
NervROS to drive it implements these on its executor node. The G1 implementation lives in
grove-g1's `g1_orchestration`.

| Interface | Kind | What it does |
|---|---|---|
| `ExecuteMission` | action | Run one compiled, validated and approved behaviour tree. The hash must match the approved XML. Feedback carries node status changes, and the result names the failed step and the skill's own reason |
| `ValidateMission` | service | Load and check a tree as `ExecuteMission` would, without running it and without the hash, busy and health checks |
| `GetCatalog` | service | The robot's skills, with args, requires, effects, resources, risk and durations, plus BehaviorTree.CPP's node model |
| `StopAll` | service | Halt the mission, cancel outstanding goals, wait for them, hold posture. A hand that holds an object keeps its grip |
| `PauseMission` | service | Pause the running mission (the base stops, the arms hold, the hands keep their grip; a pick or place finishes first) or resume it, re-running the halted step |
| `RobotState` | message | What the robot holds and does: mission, step, whether it is paused, resources held, object in each hand |
| `NodeEvent` | message | One behaviour-tree node's status change |
| `PreviewMission` | service | Where each step of a tree would leave the robot and the path there, without running it; one `StepPreview` per step |
| `Teleop` | service | Hand the base to the operator for driving by hand, or take it back |
| `Heartbeat` | message | The agent's heartbeat while a mission runs, for the executor's deadman |

# nervros_interfaces

The ROS 2 contract between the NervROS agent and a robot's mission executor. A robot that wants
NervROS to drive it implements these on its executor node. The G1 implementation lives in
grove-g1's `g1_orchestration`.

| Interface | Kind | What it does |
|---|---|---|
| `ExecuteMission` | action | Run one compiled, validated and approved behaviour tree. The hash must match the approved XML. Feedback carries node status changes, and the result names the failed step and the skill's own reason |
| `ValidateMission` | service | Load and check a tree exactly as `ExecuteMission` would, without running it |
| `GetCatalog` | service | The robot's skills, with args, requires, effects, resources, risk and durations, plus BehaviorTree.CPP's node model |
| `StopAll` | service | Halt the mission, cancel outstanding goals, wait for them, hold posture. A hand that holds an object keeps its grip |
| `RobotState` | message | What the robot holds and does: mission, step, resources held, object in each hand |
| `NodeEvent` | message | One behaviour-tree node's status change |

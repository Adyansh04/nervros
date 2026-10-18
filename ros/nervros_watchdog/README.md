# nervros_watchdog

Cancels the action goals NervROS's own motions have open on the robot when NervROS goes quiet.
NervROS runs some motions itself, such as a route it sends Nav2 or an arm move it sends MoveIt,
rather than through the robot's executor; this node makes sure such a motion does not run on
after NervROS has stopped, crashed or lost the network.

NervROS sends the goals it has open on `[ros] open_goals` (latched), again twice a second while
any is open. When nothing arrives for `timeout_s` while goals are open, the watchdog cancels each
by its id through the action's cancel service and, if `moveit_stop_topic` is set, tells MoveIt to
stop.

| | Name | Type |
|---|---|---|
| In | `open_goals_topic` | `nervros_interfaces/msg/OpenGoals`, reliable, transient local |
| Out | `<action>/_action/cancel_goal` | `action_msgs/srv/CancelGoal` |
| Out | `moveit_stop_topic` | `std_msgs/msg/String` |

Parameters (`config/nervros_watchdog.yaml`): `open_goals_topic`, `timeout_s`, `moveit_stop_topic`.

```bash
ros2 launch nervros_watchdog nervros_watchdog.launch.py params_file:=<the robot's parameters>
```

Tests: `colcon test --packages-select nervros_watchdog`.

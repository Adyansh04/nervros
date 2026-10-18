# The action goals an agent's own motions have open on the robot, such as a walk NervROS sends
# Nav2 itself. Latched, and sent again and again while any is open: a watchdog on the robot
# cancels them when these stop arriving.

builtin_interfaces/Time stamp
# Who sent the goals, as its heartbeat names it, such as "nervros 41234".
string client
# Each goal's action, such as /navigate_to_pose, beside its id in goal_ids.
string[] actions
unique_identifier_msgs/UUID[] goal_ids

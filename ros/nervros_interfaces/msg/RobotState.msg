# What the robot is doing and holding, as the mission executor knows it.
# Published on change and at 1 Hz, transient local, so a late subscriber gets the current state.

builtin_interfaces/Time stamp
# Id of the running mission, empty when idle.
string mission_id
# Name of the running step, such as "s2_GoToPlace", empty when idle.
string mission_step
# Resources the executor holds control authority over, such as "base", "left_arm", "right_arm".
string[] resources_held
# Object id held in the left hand, empty when the hand is free.
string holding_left
# Object id held in the right hand, empty when the hand is free.
string holding_right
# True after a stop until the next mission starts.
bool stopped
# Short human-readable state, such as "holding posture; right hand keeps O17".
string message

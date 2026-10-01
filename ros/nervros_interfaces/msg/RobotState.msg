# What the robot is doing and holding, as the mission executor knows it.
# Published on change and at 1 Hz, transient local, so a late subscriber gets the current state.

builtin_interfaces/Time stamp
# Id of the running mission, empty when idle.
string mission_id
# Name of the running step, such as "s2_GoToPlace", empty when idle.
string mission_step
# Resources the executor holds control authority over, such as "base", "left_arm", "right_arm".
string[] resources_held
# Object id held in the left hand, empty when the hand is free. Also empty when the executor
# cannot tell, after a pick that was cut off: message then says "left hand unknown", and the
# hand counts as full until a place with it succeeds.
string holding_left
# Object id held in the right hand; as holding_left.
string holding_right
# True after a stop until the next mission starts.
bool stopped
# Short human-readable state, such as "holding posture; right hand keeps O17".
string message
# Whether the robot can move now. False while it is down, or the controller that keeps it
# standing or drives it is not running; a mission that moves the robot is then refused.
bool can_move
# Why it cannot, for people and models, such as "fallen: tilted 104 degrees"; empty when it can.
string cannot_move_reason
# Tilt of the body from upright in degrees; NaN when the robot does not report one.
float32 tilt_deg
# True while the executor drives the base from Teleop commands.
bool teleop

# Where one mission step is expected to take the robot.

# The step id, such as "s1", and the SubTree's name, such as "s1_GoToTarget".
string step_id
string name
# The pose the base should end the step at. An empty frame_id: the step does not move the base,
# or where it ends cannot be known before it runs (an approach that closes on what it sees).
geometry_msgs/PoseStamped goal
# The path Nav2's planner finds to goal from where the previous step ends; empty when none was
# planned or none was found.
nav_msgs/Path path
# For people and models, such as "no path: the goal is inside an obstacle".
string note

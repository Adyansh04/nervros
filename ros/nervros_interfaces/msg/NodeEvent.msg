# One behaviour-tree node changed status. Modelled on nav2_msgs/BehaviorTreeStatusChange.

builtin_interfaces/Time stamp
# BehaviorTree.CPP's per-tree unique node id.
uint16 uid
# Full path of the node in the tree.
string path
# Instance name; mission steps are named "s<N>_<Skill>".
string name
# The registered node type, such as "SubTree" or "NavigateToPose".
string registration_id

uint8 STATUS_IDLE=0
uint8 STATUS_RUNNING=1
uint8 STATUS_SUCCESS=2
uint8 STATUS_FAILURE=3
uint8 STATUS_SKIPPED=4
uint8 prev_status
uint8 status

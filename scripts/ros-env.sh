# Source this before building or running anything that links ROS 2 (bash only):
#
#   source scripts/ros-env.sh
#
# It sources ROS and the overlay from build-overlay.sh, limits r2r's generated types to the
# packages NervROS and the robot need, and, with NERVROS_UDP_ONLY=1, turns off Fast DDS shared
# memory, which fails silently when the ROS graph runs as another user (a root container).

ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"
NERVROS_OVERLAY="${NERVROS_OVERLAY:-${HOME}/.cache/nervros/overlay}"
# shellcheck source=/dev/null
source "${ROS_SETUP}"
if [ -f "${NERVROS_OVERLAY}/install/setup.bash" ]; then
    # shellcheck source=/dev/null
    source "${NERVROS_OVERLAY}/install/setup.bash"
fi

# r2r generates Rust types for every interface package it is given; the full distro takes minutes
# and gigabytes, so list what is used. Add the robot's packages with NERVROS_EXTRA_IDL_PACKAGES.
_nervros_idl="std_msgs;builtin_interfaces;geometry_msgs;sensor_msgs;nav_msgs;nav2_msgs;tf2_msgs"
_nervros_idl="${_nervros_idl};action_msgs;unique_identifier_msgs;rcl_interfaces;lifecycle_msgs"
_nervros_idl="${_nervros_idl};std_srvs;vision_msgs;trajectory_msgs;service_msgs"
_nervros_idl="${_nervros_idl};type_description_interfaces;visualization_msgs;geographic_msgs;nervros_interfaces"
# For the generic ROS tools: controllers, diagnostics, logs, maps, SLAM, localization.
_nervros_idl="${_nervros_idl};control_msgs;controller_manager_msgs;diagnostic_msgs;rosgraph_msgs"
_nervros_idl="${_nervros_idl};statistics_msgs;shape_msgs;map_msgs;octomap_msgs;slam_toolbox"
_nervros_idl="${_nervros_idl};robot_localization;composition_interfaces;rosbag2_interfaces;example_interfaces"
export IDL_PACKAGE_FILTER="${_nervros_idl}${NERVROS_EXTRA_IDL_PACKAGES:+;${NERVROS_EXTRA_IDL_PACKAGES}}"
unset _nervros_idl

if [ "${NERVROS_UDP_ONLY:-0}" = 1 ]; then
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
fi

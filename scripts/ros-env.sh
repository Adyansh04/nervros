# Source this before building or running anything that links ROS 2 (bash only):
#
#   source scripts/ros-env.sh
#
# It sources ROS and the overlay from build-overlay.sh, limits r2r's generated types to the
# packages NervROS and the robot need, and, with NERVROS_UDP_ONLY=1, turns off Fast DDS shared
# memory, which fails silently when the ROS graph runs as another user (a root container).

_nervros_setup="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"
_nervros_overlay="${NERVROS_OVERLAY:-${HOME}/.cache/nervros/overlay}"
# Sourced, so a missing ROS returns rather than goes on to build against nothing.
if [[ ! -f "${_nervros_setup}" ]]; then
    echo "ros-env.sh: no ROS setup at ${_nervros_setup}; set ROS_SETUP or ROS_DISTRO" >&2
    unset _nervros_setup _nervros_overlay
    return 1
fi
# shellcheck source=/dev/null
source "${_nervros_setup}"
if [[ -f "${_nervros_overlay}/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${_nervros_overlay}/install/setup.bash"
fi
unset _nervros_setup _nervros_overlay

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
# For the MoveIt integration: its messages and the one package they need beside those above.
_nervros_idl="${_nervros_idl};moveit_msgs;object_recognition_msgs"
export IDL_PACKAGE_FILTER="${_nervros_idl}${NERVROS_EXTRA_IDL_PACKAGES:+;${NERVROS_EXTRA_IDL_PACKAGES}}"
unset _nervros_idl

if [[ "${NERVROS_UDP_ONLY:-0}" == 1 ]]; then
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
fi

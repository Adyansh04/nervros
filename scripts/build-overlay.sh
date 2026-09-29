#!/usr/bin/env bash
#
# Builds ROS 2 interface packages into a host overlay that r2r compiles against: always
# nervros_interfaces, plus the robot's own interface packages given as source directories.
#
#   ./scripts/build-overlay.sh ~/grove-g1/workspace/src/g1_msgs ~/grove-g1/workspace/src/canopy/canopy_msgs
#
# The overlay lives in ${NERVROS_OVERLAY:-~/.cache/nervros/overlay}; scripts/ros-env.sh sources it.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OVERLAY="${NERVROS_OVERLAY:-${HOME}/.cache/nervros/overlay}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"

# ROS setup files read unset variables, so nounset is off while they run.
set +u
# shellcheck source=/dev/null
source "${ROS_SETUP}"
set -u
colcon --log-base "${OVERLAY}/log" build \
    --base-paths "${ROOT}/ros" "$@" \
    --build-base "${OVERLAY}/build" --install-base "${OVERLAY}/install" \
    --parallel-workers 2 --event-handlers console_cohesion- summary+

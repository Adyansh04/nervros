#!/usr/bin/env bash
#
# Builds ROS 2 interface packages into a host overlay that r2r compiles against: always
# nervros_interfaces, plus the robot's own interface packages given as source directories.
#
#   ./scripts/build-overlay.sh ~/grove-g1/workspace/src/g1_msgs ~/grove-g1/workspace/src/canopy/canopy_msgs
#
# The overlay lives in ${NERVROS_OVERLAY:-~/.cache/nervros/overlay}; scripts/ros-env.sh sources it.
set -euo pipefail

if [[ "${1:-}" == -h || "${1:-}" == --help ]]; then
    echo "usage: $0 [interface package directory...]"
    exit 0
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
readonly ROOT
readonly OVERLAY="${NERVROS_OVERLAY:-${HOME}/.cache/nervros/overlay}"
readonly ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"

# ROS setup files read unset variables, so nounset is off while they run.
set +u
# shellcheck source=/dev/null
source "${ROS_SETUP}"
set -u
# A mistyped path to a directory would build nervros_interfaces alone and succeed, and the
# robot's types would show up missing only later.
for path in "$@"; do
    if [[ ! -f "${path}/package.xml" ]]; then
        echo "build-overlay.sh: ${path} is not a ROS package (no package.xml)" >&2
        exit 2
    fi
done
# The watchdog runs on the robot, not beside the agent: only the interfaces are built here.
colcon --log-base "${OVERLAY}/log" build \
    --base-paths "${ROOT}/ros" "$@" --packages-skip nervros_watchdog \
    --build-base "${OVERLAY}/build" --install-base "${OVERLAY}/install" \
    --parallel-workers 2 --event-handlers console_cohesion- summary+

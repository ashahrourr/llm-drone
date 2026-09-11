#!/usr/bin/env bash
# Integration check for the ROS 2 graph.
#
# Brings up the vehicle and the guard, pushes a mix of good and hostile
# commands at /drone/command, and asserts the guard's verdicts. Run it inside
# the container:
#
#   docker run --rm -v "$PWD":/work -w /work/ros2_ws -e DRONE_REPO=/work \
#       ros:jazzy-ros-base bash ros2_ws/test_graph.sh
# ROS's setup.bash references unbound vars, so -u is off deliberately.
set -eo pipefail

apt-get -qq update >/dev/null 2>&1
apt-get -qq install -y python3-numpy >/dev/null 2>&1
source /opt/ros/jazzy/setup.bash
colcon build --packages-select drone_agent >/dev/null 2>&1
source install/setup.bash

ros2 run drone_agent sim_node   >/tmp/sim.log   2>&1 &
ros2 run drone_agent guard_node >/tmp/guard.log 2>&1 &
trap 'kill $(jobs -p) 2>/dev/null || true' EXIT
sleep 6

pub() {
    ros2 topic pub --once /drone/command std_msgs/msg/String "{data: \"$1\"}" >/dev/null 2>&1
    sleep 1.2
}

pub "{\\\"verb\\\": \\\"goto\\\", \\\"north\\\": 15, \\\"east\\\": 10, \\\"altitude\\\": 8}"
pub "{\\\"verb\\\": \\\"goto\\\", \\\"north\\\": 5000, \\\"east\\\": 0, \\\"altitude\\\": 900}"
pub "{\\\"verb\\\": \\\"descend_below_ground\\\"}"
pub "{\\\"verb\\\": \\\"goto\\\", \\\"north\\\": null}"
pub "totally not json"
sleep 1

log=/tmp/guard.log
fail=0
expect() {   # expect <description> <grep pattern>
    if grep -q "$2" "$log"; then
        echo "  ok    $1"
    else
        echo "  FAIL  $1"
        fail=1
    fi
}

echo "guard verdicts:"
expect "valid command accepted"      "goto → N=15.0 E=10.0 alt=8.0"
expect "out-of-range clamped"        "clamped altitude→30.0m ceiling, waypoint→60.0m geofence"
expect "unknown verb rejected"       "unknown verb 'descend_below_ground'"
expect "null field rejected"         "north must be a number"
expect "non-JSON rejected"           "no JSON object"

# A rejected command must never become a setpoint.
if grep -qE "→ N=(5000|60\.0).*alt=900" "$log"; then
    echo "  FAIL  a rejected command reached the vehicle"
    fail=1
fi

exit $fail

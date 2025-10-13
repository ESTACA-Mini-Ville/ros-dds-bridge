#!/usr/bin/env bash
#set -euo pipefail
# avoid mesg warning on non-interactive shells
mesg n 2>/dev/null || true

# Usage: replay_rosbag_entrypoint.sh <rosbag-file> [rosbag-play-args...]
ROSBAG_FILE=${1:-}
shift || true

# Source ROS and Fast-DDS setup if present
if [ -f /opt/ros/melodic/setup.bash ]; then
  # shellcheck disable=SC1091
  . /opt/ros/melodic/setup.bash
else
  echo "/opt/ros/melodic/setup.bash not found"
fi
if [ -f /Fast-DDS/install/setup.bash ]; then
  # shellcheck disable=SC1091
  . /Fast-DDS/install/setup.bash
else
  echo "/Fast-DDS/install/setup.bash not found"
fi

# start roscore in background if available
if command -v roscore >/dev/null 2>&1; then
  echo "Starting roscore..."
  roscore >/dev/null 2>&1 &
else
  echo "roscore not found in PATH" >&2
  exit 1
fi

# Wait dynamically for ROS master to answer (timeout configurable via WAIT_ROS_TIMEOUT seconds)
WAIT_ROS_TIMEOUT=${WAIT_ROS_TIMEOUT:-30}
echo "Waiting up to ${WAIT_ROS_TIMEOUT}s for ROS master to become available..."
end_time=$((SECONDS + WAIT_ROS_TIMEOUT))
while true; do
  if rosparam list >/dev/null 2>&1; then
    echo "ROS master is available"
    break
  fi
  if [ $SECONDS -ge $end_time ]; then
    echo "Timed out waiting for ROS master after ${WAIT_ROS_TIMEOUT}s" >&2
    exit 1
  fi
  sleep 0.2
done

# If a rosbag file was provided, play it in the background
if [ -n "$ROSBAG_FILE" ]; then
  if [ -f "$ROSBAG_FILE" ]; then
    echo "Playing rosbag: $ROSBAG_FILE"
    rosbag play "$ROSBAG_FILE" "$@" >/dev/null 2>&1 &
  else
    echo "Rosbag file not found: $ROSBAG_FILE" >&2
    exit 1
  fi
else
  echo "No rosbag file provided; skipping rosbag play"
fi

# exec Bridge in foreground (installed by catkin to devel space)
if [ -f /catkin_ws/devel/setup.bash ]; then
  . /catkin_ws/devel/setup.bash
fi
# Prefer launching via roslaunch so we pick up parameters and launchfile configuration
if command -v roslaunch >/dev/null 2>&1; then
  # make sure workspace environment is sourced so roslaunch can find the package
  if [ -f /catkin_ws/devel/setup.bash ]; then
    . /catkin_ws/devel/setup.bash
  fi
  ROBOT_ARG="robot_id:=${ROBOT_ID:-robot_1}"
  exec roslaunch ros_dds_bridge ros_dds_bridge.launch ${ROBOT_ARG}
else
  # Fallback: try executing the built binary directly
  if command -v RosDdsBridge >/dev/null 2>&1; then
    exec RosDdsBridge
  elif [ -x /catkin_ws/devel/lib/ros_dds_bridge/RosDdsBridge ]; then
    exec /catkin_ws/devel/lib/ros_dds_bridge/RosDdsBridge
  else
    echo "roslaunch and RosDdsBridge binary not found; cannot start bridge" >&2
    exit 1
  fi
fi

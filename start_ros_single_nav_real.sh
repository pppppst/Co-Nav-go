#!/usr/bin/env bash
set -e

# Start the real-robot single-agent stack from one terminal:
# 1) Unitree ROS2/DDS environment
# 2) odom -> tf bridge
# 3) /cmd_vel_bridge -> Go2 sport bridge
# 4) Co-Nav single navigation node

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITREE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONAV_VENV="${CONAV_VENV:-${HOME}/venv/co-nav-real/bin/activate}"
ODOM_TOPIC="${ODOM_TOPIC:-/utlidar/robot_odom}"
PARENT_FRAME="${PARENT_FRAME:-odom}"
CHILD_FRAME="${CHILD_FRAME:-base_link}"
CMD_BRIDGE="${CMD_BRIDGE:-${UNITREE_ROOT}/install/unitree_ros2_example/bin/twist_to_go2_sport_bridge}"
ODOM_TO_TF="${ODOM_TO_TF:-${UNITREE_ROOT}/odom_to_tf.py}"

cleanup() {
    echo
    echo "Stopping ros_single_nav real-robot stack..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill "${pid}" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
}

trap cleanup EXIT INT TERM

PIDS=()

if [ -n "${CONDA_PREFIX:-}" ] && command -v conda >/dev/null 2>&1; then
    conda deactivate || true
fi

cd "${UNITREE_ROOT}"
if [ -f "${UNITREE_ROOT}/setup_connect.sh" ]; then
    # shellcheck disable=SC1091
    source "${UNITREE_ROOT}/setup_connect.sh"
else
    echo "Missing ${UNITREE_ROOT}/setup_connect.sh"
    exit 1
fi

if [ -f "/opt/ros/humble/setup.bash" ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
else
    echo "Missing /opt/ros/humble/setup.bash"
    exit 1
fi

if [ -f "${CONAV_VENV}" ]; then
    # shellcheck disable=SC1090
    source "${CONAV_VENV}"
else
    echo "Missing co-nav-real venv activate script: ${CONAV_VENV}"
    exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
unset ROS_LOCALHOST_ONLY

if [ ! -f "${ODOM_TO_TF}" ]; then
    echo "Missing odom_to_tf script: ${ODOM_TO_TF}"
    exit 1
fi

if [ ! -x "${CMD_BRIDGE}" ]; then
    echo "Missing or non-executable Go2 bridge: ${CMD_BRIDGE}"
    exit 1
fi

echo "Starting odom_to_tf: ${ODOM_TOPIC} -> ${PARENT_FRAME}/${CHILD_FRAME}"
python3 "${ODOM_TO_TF}" --ros-args \
    -p odom_topic:="${ODOM_TOPIC}" \
    -p parent_frame:="${PARENT_FRAME}" \
    -p child_frame:="${CHILD_FRAME}" &
PIDS+=("$!")

echo "Starting Twist -> Go2 Sport bridge"
"${CMD_BRIDGE}" &
PIDS+=("$!")

sleep 2

cd "${SCRIPT_DIR}"
echo "Starting ros_single_nav.py"
python ros_single_nav.py -v 1 --num_agents 1 --cmd_transport ros "$@"

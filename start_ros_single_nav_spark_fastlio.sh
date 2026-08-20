#!/usr/bin/env bash
set -e

# Start the real-robot single-agent stack with SPARK-FAST-LIO as the pose source:
# 1) Unitree ROS2/DDS environment
# 2) SPARK-FAST-LIO2 on /utlidar/cloud + /utlidar/imu, publishing TF odom -> base_link
# 3) /cmd_vel_bridge -> Go2 sport bridge
# 4) Co-Nav single navigation node

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITREE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONAV_VENV="${CONAV_VENV:-${HOME}/venv/co-nav-real/bin/activate}"
FASTLIO2_WS="${FASTLIO2_WS:-${HOME}/Documents/liang/fastlio2_ws}"
FASTLIO2_LAUNCH="${FASTLIO2_LAUNCH:-${SCRIPT_DIR}/launch/spark_fast_lio_go2.launch.yaml}"
FASTLIO2_CONFIG="${FASTLIO2_CONFIG:-${SCRIPT_DIR}/configs/spark_fast_lio_go2.yaml}"
CMD_BRIDGE="${CMD_BRIDGE:-${UNITREE_ROOT}/install/unitree_ros2_example/bin/twist_to_go2_sport_bridge}"

cleanup() {
    echo
    echo "Stopping SPARK-FAST-LIO Co-Nav real-robot stack..."
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

if [ -f "${FASTLIO2_WS}/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${FASTLIO2_WS}/install/setup.bash"
else
    echo "Missing SPARK-FAST-LIO workspace setup: ${FASTLIO2_WS}/install/setup.bash"
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

if [ ! -f "${FASTLIO2_LAUNCH}" ]; then
    echo "Missing SPARK-FAST-LIO launch file: ${FASTLIO2_LAUNCH}"
    exit 1
fi

if [ ! -f "${FASTLIO2_CONFIG}" ]; then
    echo "Missing SPARK-FAST-LIO config file: ${FASTLIO2_CONFIG}"
    exit 1
fi

if [ ! -x "${CMD_BRIDGE}" ]; then
    echo "Missing or non-executable Go2 bridge: ${CMD_BRIDGE}"
    exit 1
fi

echo "Starting SPARK-FAST-LIO2: /utlidar/cloud + /utlidar/imu -> odom/base_link"
ros2 launch "${FASTLIO2_LAUNCH}" \
    start_rviz:=false \
    config_path:="${FASTLIO2_CONFIG}" &
PIDS+=("$!")

echo "Starting Twist -> Go2 Sport bridge"
"${CMD_BRIDGE}" &
PIDS+=("$!")

sleep 10

cd "${SCRIPT_DIR}"
echo "Starting ros_single_nav.py"
python ros_single_nav.py -v 1 --num_agents 1 --cmd_transport ros "$@"

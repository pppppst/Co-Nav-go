#!/usr/bin/env bash
set -e

# Real-robot Co-Nav stack using Hesai XT16 raw points + SPARK-FAST-LIO as pose source.
# Assumes Hesai config uses host receive time:
#   /home/isee/Documents/liang/hesai_xt16_ws/src/HesaiLidar_ROS_2.0/config/config.yaml
#   driver.use_timestamp_type: 1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITREE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONAV_VENV="${CONAV_VENV:-${HOME}/venv/co-nav-real/bin/activate}"
HESAI_WS="${HESAI_WS:-${HOME}/Documents/liang/hesai_xt16_ws}"
HESAI_CONFIG="${HESAI_CONFIG:-${HESAI_WS}/src/HesaiLidar_ROS_2.0/config/config.yaml}"
FASTLIO2_WS="${FASTLIO2_WS:-${HOME}/Documents/liang/fastlio2_ws}"
FASTLIO2_LAUNCH="${FASTLIO2_LAUNCH:-${SCRIPT_DIR}/launch/spark_fast_lio_go2_xt16.launch.yaml}"
FASTLIO2_CONFIG="${FASTLIO2_CONFIG:-${SCRIPT_DIR}/configs/spark_fast_lio_go2_xt16.yaml}"
CMD_BRIDGE="${CMD_BRIDGE:-${UNITREE_ROOT}/install/unitree_ros2_example/bin/twist_to_go2_sport_bridge}"

cleanup() {
    echo
    echo "Stopping Hesai XT16 SPARK-FAST-LIO Co-Nav stack..."
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

# Keep all robot/hesai/fastlio/conav nodes on the same DDS backend.
unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset RMW_FASTRTPS_USE_QOS_FROM_XML
unset ROS_LOCALHOST_ONLY

if [ -f "${HESAI_WS}/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${HESAI_WS}/install/setup.bash"
else
    echo "Missing Hesai workspace setup: ${HESAI_WS}/install/setup.bash"
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

for required in "${HESAI_CONFIG}" "${FASTLIO2_LAUNCH}" "${FASTLIO2_CONFIG}"; do
    if [ ! -f "${required}" ]; then
        echo "Missing required file: ${required}"
        exit 1
    fi
done

if [ ! -x "${CMD_BRIDGE}" ]; then
    echo "Missing or non-executable Go2 bridge: ${CMD_BRIDGE}"
    exit 1
fi

cd "${SCRIPT_DIR}"

echo "Starting Hesai XT16 driver: 192.168.123.20 -> /lidar_points"
ros2 run hesai_ros_driver hesai_ros_driver_node \
    --ros-args -p config_path:="${HESAI_CONFIG}" &
PIDS+=("$!")

sleep 2

echo "Starting Hesai -> FAST-LIO point cloud converter: /lidar_points -> /lidar_points_fastlio"
python tools/hesai_to_fastlio.py \
    --input /lidar_points \
    --output /lidar_points_fastlio \
    --reliability reliable &
PIDS+=("$!")

sleep 2

echo "Starting SPARK-FAST-LIO2: /lidar_points_fastlio + /utlidar/imu -> odom/base_link"
ros2 launch "${FASTLIO2_LAUNCH}" \
    start_rviz:=false \
    config_path:="${FASTLIO2_CONFIG}" &
PIDS+=("$!")

echo "Starting Twist -> Go2 Sport bridge"
"${CMD_BRIDGE}" &
PIDS+=("$!")

sleep 10

echo "Starting ros_single_nav.py"
python ros_single_nav.py -v 1 --num_agents 1 --cmd_transport ros "$@"

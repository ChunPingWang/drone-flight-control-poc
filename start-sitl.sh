#!/bin/bash
# PX4 SITL + Gazebo Harmonic 固定翼模擬快速啟動
#
# 用法:
#   ./start-sitl.sh                  # 預設啟動 RC Cessna
#   ./start-sitl.sh advanced_plane   # 啟動進階固定翼
#   HEADLESS=1 ./start-sitl.sh       # 不開啟 Gazebo GUI
#
# 可用的固定翼 vehicle:
#   rc_cessna       - RC Cessna 小型飛機
#   advanced_plane  - 進階固定翼飛機

VEHICLE=${1:-rc_cessna}
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="${SCRIPT_DIR}/PX4-Autopilot"

# 排除 ESP-IDF Python venv，避免 CMake 使用錯誤的 Python
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v espressif | grep -v 'esp/esp-idf' | tr '\n' ':' | sed 's/:$//')

# 啟用 PX4 專用 Python venv
if [ -d "${PX4_DIR}/.venv" ]; then
    source "${PX4_DIR}/.venv/bin/activate"
fi

echo "========================================="
echo " PX4 SITL - Gazebo Harmonic"
echo " Vehicle: ${VEHICLE}"
echo " Python:  $(which python3)"
echo "========================================="

cd "${PX4_DIR}" || exit 1
make px4_sitl gz_${VEHICLE}

#!/usr/bin/env bash
# 手先空間テレオペ（ikpy 版 teleop_cartesian）を起動する。W/S:x A/D:y R/F:z Q/E:刻み O/C:グリッパ X:リセット
# 前提: pip3 install --user ikpy 済み、/home/ubuntu/crane_plus.urdf が生成済み。
# 使い方: bash run_teleop_task_ikpy.sh
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash
exec ros2 run physicalai_teleop teleop_cartesian "$@"

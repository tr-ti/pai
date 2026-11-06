#!/usr/bin/env bash
# 講義オリジナルの関節空間テレオペ（teleop_keyboard）を起動する。
# 使い方: bash run_teleop_joint_only.sh
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash
exec ros2 run physicalai_teleop teleop_keyboard "$@"

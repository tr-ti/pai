#!/usr/bin/env bash
# 手先空間テレオペ（MoveIt /compute_ik 版）を起動する。W/S:x A/D:y R/F:z T/G:ピッチ O/C:グリッパ X:リセット
# 前提: move_group が起動していて /compute_ik が出ていること。
# BASE / TIP は SRDF の arm グループで確認した名前に合わせる（環境変数で上書き可）。
# 使い方: BASE=crane_plus_base TIP=crane_plus_link_endeffector bash run_teleop_task_moveit.sh
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash
exec ros2 run physicalai_teleop teleop_cartesian_moveit --ros-args \
  -p base_frame:="${BASE:-crane_plus_base}" \
  -p tip_link:="${TIP:-crane_plus_link_endeffector}" "$@"

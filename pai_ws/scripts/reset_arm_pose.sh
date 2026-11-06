#!/usr/bin/env bash
# アームを直立姿勢（全関節 0）に戻し、グリッパを閉じる（テレオペの初期状態と同じ）。
# 使い方: bash reset_arm_pose.sh [所要秒数(既定2)]
#         GRIP=-0.5236 bash reset_arm_pose.sh   # グリッパを開いて戻したいとき
# 注意: テレオペが起動中だと 20 Hz で指令を上書きするため、戻らない。
#       テレオペ側のリセットは X キー。このスクリプトはテレオペを止めた状態（episode 前など）で使う。
SEC="${1:-2}"
GRIP="${GRIP:-0.1745}"
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash

ros2 topic pub --once /crane_plus_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [crane_plus_joint1, crane_plus_joint2, crane_plus_joint3, crane_plus_joint4], points: [{positions: [0.0, 0.0, 0.0, 0.0], time_from_start: {sec: ${SEC}}}]}"
ros2 topic pub --once /crane_plus_gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [crane_plus_joint_hand], points: [{positions: [${GRIP}], time_from_start: {sec: ${SEC}}}]}"

sleep "$((SEC + 1))"
echo "reset: arm=直立(全関節0) gripper=${GRIP}"

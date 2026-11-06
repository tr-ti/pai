#!/usr/bin/env bash
# Gazebo の立方体位置を /cube_pose に流す（データ収集・推論の間は起動したままにする）。
# 使い方: Gazebo 起動後、別ターミナルで bash run_cube_pose_relay.sh   （停止は Ctrl+C）
# 前提: 起動時の立方体が初期位置(0, -0.1)付近にあること（cube_pose_relay.py の SEED）。
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash
exec python3 ~/pai_training/scripts/cube_pose_relay.py "$@"

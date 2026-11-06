#!/usr/bin/env bash
# Gazebo(机+CRANE+ V2)を起動する。Gazebo / RViz2 / move_group がまとめて立ち上がる。
# 使い方: bash launch_gazebo.sh   （起動に1〜2分。ログが落ち着くまで待つ）
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash
exec ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py "$@"

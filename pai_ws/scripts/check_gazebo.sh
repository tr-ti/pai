#!/usr/bin/env bash
# Gazebo 起動後の状態確認（コントローラが active か、joint_states が出ているか）。
# 使い方: 別ターミナルで bash check_gazebo.sh
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash
echo "=== controllers ==="
ros2 control list_controllers
echo
echo "=== /joint_states (1回) ==="
timeout 5 ros2 topic echo /joint_states --once
echo
echo "=== 重いプロセス上位 ==="
top -b -n 1 -o %CPU | head -12

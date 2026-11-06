#!/usr/bin/env bash
# 関節空間 + 手先空間 テレオペ（teleop_keyboard_task）を起動する。
#   関節: Q/E W/S R/F O/C X ESC    手先: 矢印キー(前後左右) I/K(上下)
# 使い方: Gazebo の実時間比が安定してから bash run_teleop_joint_and_task.sh
# 注意: teleop_keyboard など同じトピックに出すノードは止めておくこと。
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash
exec ros2 run physicalai_teleop teleop_keyboard_task "$@"

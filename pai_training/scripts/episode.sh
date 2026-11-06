#!/bin/bash
# 1 エピソード分のデータ収集: 初期姿勢へ -> 立方体リセット -> bag 記録開始 -> push_demo 実行 -> 記録停止
# 使い方: bash episode.sh <エピソード番号> [push_demo の追加パラメータ  例: -p x_end:=0.02]
# 前提:
#   - Gazebo 起動済み（~/pai_ws/scripts/launch_gazebo.sh）
#   - 別ターミナルで cube_pose_relay.py 起動済み（~/pai_ws/scripts/run_cube_pose_relay.sh）
#   - テレオペは停止（同じトピックに出すと競合する）
#   - ~/pai_ws/scripts/startp0.txt, startp.txt が用意済み（show_arm_pose.sh の出力）
N=${1:?エピソード番号を指定}
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash
BAG=$HOME/dataset/rosbags/push/ep$N
[ -e "$BAG" ] && { echo "already exists: $BAG"; exit 1; }
for f in startp0.txt startp.txt; do
  [ -f ~/pai_ws/scripts/$f ] || { echo "姿勢ファイルがありません: ~/pai_ws/scripts/$f"; exit 1; }
done
timeout 5 ros2 topic echo /cube_pose --once >/dev/null 2>&1 \
  || { echo "/cube_pose が出ていません。run_cube_pose_relay.sh を起動してください"; exit 1; }
mkdir -p $HOME/dataset/rosbags/push
bash ~/pai_ws/scripts/reset_arm_pose.sh 2          # 初期姿勢（全関節0・グリッパ閉）
bash ~/pai_training/scripts/reset_cube.sh
ros2 bag record -o $BAG /joint_states /crane_plus_arm_controller/joint_trajectory /crane_plus_gripper_controller/joint_trajectory /cube_pose &
BAGPID=$!
sleep 2
python3 ~/pai_training/scripts/push_demo.py --ros-args -p use_topic:=true "${@:2}"
sleep 1
kill -INT $BAGPID
wait $BAGPID
echo "done: $BAG"

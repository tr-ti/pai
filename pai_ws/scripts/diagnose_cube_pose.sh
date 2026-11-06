#!/usr/bin/env bash
# /cube_pose が出ないときの切り分け。Gazebo 起動中に実行し、出力を貼ってもらえば原因を絞れる。
# 使い方: bash diagnose_cube_pose.sh
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash

echo "=== 1. プロセス（橋渡し / 中継）==="
ps aux | grep -E "parameter_bridge|cube_pose_relay" | grep -v grep | cut -c1-160 || true
pgrep -f "parameter_bridge.*pose/info" >/dev/null && echo "OK   pose/info の橋渡しあり" || echo "NG   pose/info の橋渡しが動いていない"
pgrep -f cube_pose_relay.py >/dev/null && echo "OK   cube_pose_relay あり" || echo "NG   cube_pose_relay が動いていない"

echo
echo "=== 2. Gazebo 側の姿勢トピック（ign）==="
if timeout 5 ign topic -e -t /world/default/pose/info -n 1 >/dev/null 2>&1; then echo "OK   Gazebo は /world/default/pose/info を出している"
else echo "NG   Gazebo から pose/info が取れない（Gazebo 未起動 / 一時停止中？）"; fi

echo
echo "=== 3. 立方体の現在位置（Gazebo 世界座標。ロボット原点は z=1.015）==="
bash ~/pai_training/scripts/cube_xy.sh 2>&1 | head -3
echo "（中継の期待位置: 世界座標 x=0 y=-0.1 z=1.015 から 3 cm 以内）"

echo
echo "=== 4. ROS 側トピック ==="
ros2 topic list 2>/dev/null | grep -E "pose|cube" || echo "NG   pose/cube 関連トピックが ROS に見えない"
echo "--- /world/default/pose/info の受信（5秒）"
timeout 6 ros2 topic hz /world/default/pose/info 2>&1 | head -3 || true
echo "--- /cube_pose の受信（5秒）"
timeout 6 ros2 topic hz /cube_pose 2>&1 | head -3 || true

echo
echo "=== 5. 橋渡しのログ（run_cube_pose_relay.sh が起動した場合）==="
tail -5 /tmp/pose_bridge.log 2>/dev/null || echo "（ログなし）"
echo
echo "ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'  （各ターミナルで同じ値であること）"

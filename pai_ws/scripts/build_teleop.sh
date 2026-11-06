#!/usr/bin/env bash
# physicalai_teleop パッケージをビルドする（teleop ノードを追加・変更したあとに実行）。
# 使い方: bash build_teleop.sh
set -e
source /opt/ros/humble/setup.bash
cd ~/pai_ws
colcon build --symlink-install --packages-select physicalai_teleop
echo "ビルド完了。新しいターミナルで: source ~/pai_ws/install/setup.bash"

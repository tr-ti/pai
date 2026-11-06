#!/usr/bin/env bash
# Gazebo 関連の残プロセスを確認して止める（再起動前の掃除）。
# 先に launch したターミナルで Ctrl+C を押してから実行する。
# 使い方: bash stop_gazebo.sh
PATTERN="ruby|rviz2|move_group|ign gazebo|gz sim|teleop_"
echo "=== 残プロセス ==="
ps aux | grep -E "$PATTERN" | grep -v grep
echo
read -r -p "これらを kill しますか? [y/N] " ans
if [ "$ans" = "y" ]; then
  pkill -f "$PATTERN"
  sleep 2
  echo "=== kill 後 ==="
  ps aux | grep -E "$PATTERN" | grep -v grep || echo "残りなし"
fi

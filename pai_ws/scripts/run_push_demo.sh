#!/usr/bin/env bash
# 押し出しデモを実行する（初期姿勢 -> startp0 -> startp -> +x へ押し出し -> 戻る）。
# 使い方:
#   bash run_push_demo.sh dry            # 軌道の検証だけ（動かさない）。最初に必ずこれ
#   bash run_push_demo.sh                # 実行（/cube_pose があれば押した量も表示）
#   bash run_push_demo.sh run -p x_end:=0.0 -p j4_widen:=0.1   # パラメータ変更
# テレオペは止めておくこと。
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash
MODE="${1:-run}"; shift 2>/dev/null
if [ "$MODE" = "dry" ]; then
  exec python3 ~/pai_training/scripts/push_demo.py --ros-args -p dry_run:=true "$@"
fi
exec python3 ~/pai_training/scripts/push_demo.py --ros-args -p use_topic:=true "$@"

#!/usr/bin/env bash
# 立方体 (wood_cube_5cm) を初期位置に戻す。既定は pai_training/scripts/reset_cube.sh と同じ
# （x=0±0.01, y=-0.1±0.01 の範囲でランダム）。
# 使い方:
#   bash reset_cube_pose.sh                    # ランダム（既定）
#   FIXED=1 bash reset_cube_pose.sh            # ランダムなしで固定位置
#   CUBE_X=0.02 CUBE_Y=-0.12 RANGE=0 bash reset_cube_pose.sh   # 位置・散らし幅[m]を指定
set -e
source /opt/ros/humble/setup.bash
CUBE_X="${CUBE_X:-0.0}"
CUBE_Y="${CUBE_Y:--0.1}"
RANGE="${RANGE:-0.01}"
[ -n "$FIXED" ] && RANGE=0
CUBE_Z="${CUBE_Z:-1.016}"     # 机上の高さ（reset_cube.sh と同じ値）

read -r DX DY < <(python3 -c "
import random
r = $RANGE
print(round($CUBE_X + random.uniform(-r, r), 4), round($CUBE_Y + random.uniform(-r, r), 4))")

REQ="name: \"wood_cube_5cm\", position: {x: $DX, y: $DY, z: $CUBE_Z}, orientation: {x: 0, y: 0, z: 0, w: 1}"
ign service -s /world/default/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 2000 --req "$REQ"
echo "reset cube: x=$DX y=$DY"
sleep 1.5

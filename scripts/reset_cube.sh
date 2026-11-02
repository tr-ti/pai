#!/bin/bash
DX=$(python3 -c "import random;print(round(random.uniform(-0.01,0.01),4))")
DY=$(python3 -c "import random;print(round(-0.1+random.uniform(-0.01,0.01),4))")
REQ="name: \"wood_cube_5cm\", position: {x: $DX, y: $DY, z: 1.016}, orientation: {x: 0, y: 0, z: 0, w: 1}"
ign service -s /world/default/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 2000 --req "$REQ"
echo "reset: x=$DX y=$DY"
sleep 1.5

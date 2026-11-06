#!/usr/bin/env bash
# 現在のアーム姿勢を表示する（関節角[rad/deg]・グリッパ・手先位置と向き）。
# 手先位置は teleop_keyboard_task と同じ運動学で計算（tool_length は TOOL_LEN で変更可、既定 0.05 m）。
# 使い方: Gazebo 起動中に bash show_arm_pose.sh
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash
TOOL_LEN="${TOOL_LEN:-0.05}" python3 - <<'EOF'
import math, os, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from physicalai_teleop.teleop_keyboard_task import fk, ARM, GRIP

rclpy.init()
n = Node('show_arm_pose')
got = {}
n.create_subscription(JointState, '/joint_states', lambda m: got.update(zip(m.name, m.position)), 10)
for _ in range(100):
    rclpy.spin_once(n, timeout_sec=0.1)
    if all(j in got for j in ARM):
        break
else:
    print('/joint_states を受信できません（Gazebo 起動中か確認）')
    raise SystemExit(1)
q = [got[j] for j in ARM]
for j, v in zip(ARM, q):
    print(f'{j}: {v:+.4f} rad ({math.degrees(v):+7.1f} deg)')
if GRIP in got:
    print(f'{GRIP}: {got[GRIP]:+.4f} rad ({math.degrees(got[GRIP]):+7.1f} deg)')
x, y, z, phi = fk(q, float(os.environ['TOOL_LEN']))
print(f'手先: x={x:+.3f} y={y:+.3f} z={z:+.3f} m  pitch={math.degrees(phi):+.0f} deg')
n.destroy_node(); rclpy.shutdown()
EOF

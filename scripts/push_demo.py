#!/usr/bin/env python3
"""立方体を +x 方向へ押す自動デモ。関節軌道を 20 Hz で publish する（未検証。dry_run で先に確認）"""
import math, sys
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HZ = 20
ARM = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint3', 'crane_plus_joint4']
GRIP = 'crane_plus_joint_hand'
L2, L3, LH, Z2, YOFF = 0.083, 0.0935, 0.045, 0.0714, 0.0145
LIM = [(-2.618, 2.618), (-1.8247, 1.7837), (-2.5566, 2.4543), (-1.8401, 1.7684)]

def ik(x, y, z, f):
    """指先を真下に向け、指先の下端を (x, y, z)（ロボット座標）に置く関節角 [q1..q4]"""
    R = math.hypot(x, y)
    r = math.sqrt(max(R * R - YOFF * YOFF, 1e-9))
    q1 = math.atan2(y, x) + math.atan2(YOFF, r)
    wz = z + LH + f - Z2                       # 手首(joint4)の、joint2 からの高さ
    c = (r * r + wz * wz - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if abs(c) > 1:
        raise ValueError(f'届かない: x={x:.3f} y={y:.3f} z={z:.3f}')
    d = math.acos(c)
    a2 = math.atan2(r, wz) - math.atan2(L3 * math.sin(d), L2 + L3 * math.cos(d))
    q = [q1, a2, -d, a2 + d - math.pi]
    for i, (v, (lo, hi)) in enumerate(zip(q, LIM)):
        if not lo <= v <= hi:
            raise ValueError(f'関節{i+1}が範囲外: {v:.3f} (x={x:.3f} y={y:.3f} z={z:.3f})')
    return q

def line(p0, p1, dur, ease=True):
    n = max(1, int(dur * HZ)); out = []
    for i in range(1, n + 1):
        s = i / n
        if ease: s = 3 * s * s - 2 * s ** 3
        out.append([a + (b - a) * s for a, b in zip(p0, p1)])
    return out

class PushDemo(Node):
    def __init__(self):
        super().__init__('push_demo')
        defaults = dict(cube_x=0.0, cube_y=-0.1, use_topic=False, f=0.05, finger_half=0.008,
                        standoff=0.03, push=0.07, z_push=-0.01, z_hi=0.05, speed=0.02,
                        grip=-0.5236, dry_run=False)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.g = lambda k: self.get_parameter(k).value
        self.cube = None
        self.create_subscription(PoseStamped, '/cube_pose', lambda m: setattr(self, 'cube', m), 10)
        self.pa = self.create_publisher(JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10)
        self.pg = self.create_publisher(JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10)

    def build(self):
        cx, cy = self.g('cube_x'), self.g('cube_y')
        if self.g('use_topic'):
            t0 = self.get_clock().now()
            while self.cube is None and (self.get_clock().now() - t0).nanoseconds < 5e9:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.cube is None:
                raise RuntimeError('/cube_pose が受信できません')
            cx, cy = self.cube.pose.position.x, self.cube.pose.position.y
        self.get_logger().info(f'立方体位置: x={cx:.3f} y={cy:.3f}')
        f, fh = self.g('f'), self.g('finger_half')
        zp, zh = self.g('z_push'), self.g('z_hi')
        x0 = cx - 0.025 - fh - self.g('standoff')        # 押し始め（指の中心）
        x1 = cx - 0.025 + self.g('push') - fh            # 立方体が push 分動いたときの指の中心
        P0, P1, P2, P3 = (x0, cy, zh), (x0, cy, zp), (x1, cy, zp), (x1, cy, zh)
        qh = [0.0] * 4
        traj = line(qh, ik(*P0, f), 3.0)                 # 関節空間
        for a, b, dur, ease in [(P0, P1, 1.5, True),
                                (P1, P2, (x1 - x0) / self.g('speed'), False),
                                (P2, P3, 1.0, True)]:
            traj += [ik(*p, f) for p in line(a, b, dur, ease)]
        traj += line(traj[-1], qh, 2.5)                  # 直立姿勢へ戻す
        traj += [qh] * int(0.5 * HZ)
        return traj

    def make(self, names, pos):
        m = JointTrajectory(); m.joint_names = names
        p = JointTrajectoryPoint(); p.positions = [float(v) for v in pos]
        p.time_from_start = Duration(sec=0, nanosec=100_000_000)
        m.points.append(p); return m

    def run(self):
        traj = self.build()
        n = len(traj)
        self.get_logger().info(f'{n} 点 ({n / HZ:.1f} 秒)。押し始め q={[round(v, 3) for v in traj[3 * HZ - 1]]}')
        if self.g('dry_run'):
            return
        for i, q in enumerate(traj):
            self.pa.publish(self.make(ARM, q))
            self.pg.publish(self.make([GRIP], [self.g('grip')]))
            rclpy.spin_once(self, timeout_sec=1.0 / HZ)
        self.get_logger().info('DONE: 記録を Ctrl+C で止めてください')

def main():
    rclpy.init(args=sys.argv)
    n = PushDemo()
    try: n.run()
    except (ValueError, RuntimeError) as e: n.get_logger().error(str(e))
    finally: n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""立方体を +x 方向へ押し出す自動デモ（姿勢ファイル経由）。

動作:
  初期姿勢(全関節 0) -> startp0.txt の姿勢 -> startp.txt の姿勢
  -> joint1 を回して手先を +x 方向へ掃引し、立方体を押す
     （手先 x が x_widen を超えたあたりから joint4 も少し広げ、手先を外側(-y 側)へ膨らませる）
  -> 逆順に戻る（startp -> startp0 -> 初期姿勢）

姿勢ファイルは show_arm_pose.sh の出力書式（例: "crane_plus_joint1: -2.5000 rad (...)"）。
20 Hz で arm / gripper の joint_trajectory を publish する。
dry_run:=true なら publish せず、軌道の検証結果（関節範囲・最大速度・手先の通り道）だけ表示する。

使い方（ROS を source した状態で）:
  python3 push_demo.py --ros-args -p dry_run:=true
  python3 push_demo.py --ros-args -p use_topic:=true -p x_end:=0.03
"""
import math
import os
import re
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HZ = 20
ARM = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint3', 'crane_plus_joint4']
GRIP = 'crane_plus_joint_hand'
L2, L3, LH, Z2, YOFF = 0.083, 0.0935, 0.045, 0.0714, 0.0145
LIM = [(-2.6179, 2.6179), (-1.8247, 1.7837), (-2.5565, 2.4542), (-1.8400, 1.7683)]
POSE_LINE = re.compile(r'^\s*(crane_plus_\w+)\s*:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*rad')


def parse_pose_file(path):
    """(arm=[q1..q4], grip or None)。4 関節が揃わなければ ValueError。"""
    vals = {}
    with open(os.path.expanduser(path), encoding='utf-8') as fh:
        for line in fh:
            m = POSE_LINE.match(line)
            if m:
                vals[m.group(1)] = float(m.group(2))
    missing = [j for j in ARM if j not in vals]
    if missing:
        raise ValueError(f'{path}: 関節の行が見つかりません: ' + ', '.join(missing))
    return [vals[j] for j in ARM], vals.get(GRIP)


def fk(q, f):
    """関節角 -> 制御点 (x, y, z, pitch)。teleop_keyboard_task.fk と同じ。"""
    q1, q2, q3, q4 = q
    phi = q2 - q3 - q4
    rw = L2 * math.sin(q2) + L3 * math.sin(q2 - q3)
    zw = Z2 + L2 * math.cos(q2) + L3 * math.cos(q2 - q3)
    k = LH + f
    r = rw + k * math.sin(phi)
    z = zw + k * math.cos(phi)
    return (r * math.cos(q1) + YOFF * math.sin(q1),
            r * math.sin(q1) - YOFF * math.cos(q1), z, phi)


def check_limits(q, label):
    for i, (v, (lo, hi)) in enumerate(zip(q, LIM)):
        if not lo <= v <= hi:
            raise ValueError(f'{label}: joint{i + 1}={v:.3f} が可動範囲 [{lo}, {hi}] の外です')


def ease(s):
    return 3 * s * s - 2 * s ** 3


def seg(q0, q1, dur):
    """関節空間の直線補間（始点を除く）。"""
    n = max(1, int(round(dur * HZ)))
    return [[a + (b - a) * ease(i / n) for a, b in zip(q0, q1)] for i in range(1, n + 1)]


class PushDemo(Node):
    def __init__(self):
        super().__init__('push_demo')
        defaults = dict(
            pose0='~/pai_ws/scripts/startp0.txt',   # 経由姿勢
            pose1='~/pai_ws/scripts/startp.txt',    # 押し始めの姿勢
            f=0.05,            # hand リンク原点から指先までの長さ [m]（teleop の tool_length と同じ）
            x_end=0.03,        # 掃引の終点：joint4 を広げる前の手先 x がこの値になるまで joint1 を回す [m]
            x_widen=0.0,       # この手先 x を超えたあたりから joint4 を広げ始める [m]
            j4_widen=0.15,     # joint4 を広げる量 [rad]（+ で手先が外側へ）
            speed=0.02,        # 押すときの手先速度 [m/s]
            back_factor=2,     # 戻りの速さ（押すときの何倍か）
            t_home=3.0,        # 初期姿勢 <-> startp0 の所要時間 [s]
            t_pose=3.0,        # startp0 <-> startp の所要時間 [s]
            settle=0.5,        # 各姿勢で止まる時間 [s]
            hold_end=0.5,      # 押し終わりで止まる時間 [s]
            grip=0.1745,       # グリッパ指令（閉）
            use_topic=False,   # True: /cube_pose を読み、掃引との位置関係と押した量を表示
            dry_run=False,
        )
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.g = lambda k: self.get_parameter(k).value
        self.cube = None
        self.create_subscription(PoseStamped, '/cube_pose', lambda m: setattr(self, 'cube', m), 10)
        self.pa = self.create_publisher(JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10)
        self.pg = self.create_publisher(JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10)

    # --- 軌道生成 -----------------------------------------------------------
    def sweep(self, q_s, f):
        """startp から joint1 を回して手先を +x へ。joint4 は x>x_widen で広げる。"""
        x_end, x_widen = self.g('x_end'), self.g('x_widen')
        if x_end <= x_widen:
            raise ValueError('x_end は x_widen より大きくしてください')

        def x_of(q1):
            return fk([q1, q_s[1], q_s[2], q_s[3]], f)[0]

        if x_of(q_s[0]) >= x_end:
            raise ValueError(f'startp の手先 x={x_of(q_s[0]):+.3f} が既に x_end={x_end:+.3f} 以上です')
        lo, hi = q_s[0], 0.0
        if x_of(hi) < x_end:
            raise ValueError(f'x_end={x_end:+.3f} は joint1 を回しても届きません（最大 {x_of(hi):+.3f}）')
        for _ in range(60):                         # x は q1 に対して単調増加
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if x_of(mid) < x_end else (lo, mid)
        q1_end = hi
        x0, y0, _, _ = fk(q_s, f)
        radius = math.hypot(x0, y0)
        t = abs(q1_end - q_s[0]) * radius / self.g('speed')
        n = max(2, int(round(t * HZ)))
        pts = []
        for i in range(1, n + 1):
            q1 = q_s[0] + (q1_end - q_s[0]) * i / n
            xi = x_of(q1)
            u = min(max((xi - x_widen) / (x_end - x_widen), 0.0), 1.0)
            q4 = q_s[3] + self.g('j4_widen') * ease(u)
            pts.append([q1, q_s[1], q_s[2], q4])
        return pts, q1_end, t

    def build(self):
        f = self.g('f')
        p0, _ = parse_pose_file(self.g('pose0'))
        p1, _ = parse_pose_file(self.g('pose1'))
        home = [0.0] * 4
        check_limits(p0, 'startp0')
        check_limits(p1, 'startp')
        sw, q1_end, t_sw = self.sweep(p1, f)
        for i, q in enumerate(sw):
            check_limits(q, f'掃引 {i}')

        hold = lambda q, s: [list(q)] * int(round(s * HZ))
        traj = seg(home, p0, self.g('t_home'))
        traj += seg(p0, p1, self.g('t_pose'))
        traj += hold(p1, self.g('settle'))
        traj += sw
        traj += hold(sw[-1], self.g('hold_end'))
        pts = [p1] + sw
        back = pts[::-1][::max(1, int(self.g('back_factor')))]
        if back[-1] != pts[0]:
            back.append(pts[0])
        traj += back
        traj += hold(p1, self.g('settle'))
        traj += seg(p1, p0, self.g('t_pose'))
        traj += seg(p0, home, self.g('t_home'))
        traj += hold(home, 0.5)
        self.report(f, p0, p1, sw, q1_end, t_sw, traj)
        return traj

    def report(self, f, p0, p1, sw, q1_end, t_sw, traj):
        n = len(traj)
        vmax = max(abs(b - a) for u, v in zip(traj, traj[1:]) for a, b in zip(u, v)) * HZ
        self.get_logger().info(f'{n} 点 ({n / HZ:.1f} 秒)  最大関節速度 {vmax:.2f} rad/s')
        for name, q in [('startp0', p0), ('startp ', p1), ('掃引中間', sw[len(sw) // 2]), ('掃引終点', sw[-1])]:
            x, y, z, ph = fk(q, f)
            self.get_logger().info(f'  {name}: 手先 x={x:+.3f} y={y:+.3f} z={z:+.3f} m  pitch={math.degrees(ph):+.0f}deg'
                                   f'  q={[round(v, 3) for v in q]}')
        self.get_logger().info(f'  掃引: joint1 {p1[0]:+.3f} -> {q1_end:+.3f} rad ({t_sw:.1f} s), '
                               f'joint4 {p1[3]:+.3f} -> {sw[-1][3]:+.3f} rad')
        if vmax > 1.5:
            self.get_logger().warn('関節速度が大きい区間があります（t_home / t_pose を長くしてください）')
        if self.cube is not None:
            cx, cy = self.cube.pose.position.x, self.cube.pose.position.y
            x0, y0 = fk(p1, f)[:2]
            xe, ye = fk(sw[-1], f)[:2]
            rc, rf = math.hypot(cx, cy), math.hypot(x0, y0)
            self.get_logger().info(
                f'  立方体 x={cx:+.3f} y={cy:+.3f}: 根元からの距離 {rc:.3f} m / 手先の掃引半径 {rf:.3f} m '
                f'(差 {rc - rf:+.3f} m)')
            if abs(rc - rf) > 0.02:
                self.get_logger().warn('立方体が掃引の通り道から 2 cm 以上ずれています（startp の取り直しを検討）')
            if not (min(x0, xe) - 0.025 <= cx <= max(x0, xe) + 0.025):
                self.get_logger().warn('立方体の x が掃引範囲の外です')
            # 指の半幅 8 mm + 立方体の半幅 25 mm を使った幾何的な目安（円弧の曲がり・滑りは無視）
            self.get_logger().info(f'  押し量の目安: 約 {xe - (cx - 0.033):+.3f} m（x 方向。実測は use_topic:=true で表示）')

    # --- 実行 -------------------------------------------------------------
    def make(self, names, pos):
        m = JointTrajectory()
        m.joint_names = names
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in pos]
        p.time_from_start = Duration(sec=0, nanosec=100_000_000)
        m.points.append(p)
        return m

    def wait_cube(self, sec=5.0):
        t0 = time.monotonic()
        while self.cube is None and time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.cube is None:
            raise RuntimeError('/cube_pose が受信できません（cube_pose_relay.py を起動してください）')

    def run(self):
        if self.g('use_topic'):
            self.wait_cube()
            c0 = (self.cube.pose.position.x, self.cube.pose.position.y)
        traj = self.build()
        if self.g('dry_run'):
            return
        grip = self.g('grip')
        t0 = time.monotonic()
        for i, q in enumerate(traj):
            self.pa.publish(self.make(ARM, q))
            self.pg.publish(self.make([GRIP], [grip]))
            target = t0 + (i + 1) / HZ                     # 20 Hz に正確にそろえる
            while True:
                rem = target - time.monotonic()
                if rem <= 0:
                    break
                rclpy.spin_once(self, timeout_sec=rem)
        if self.g('use_topic'):
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
            c1 = (self.cube.pose.position.x, self.cube.pose.position.y)
            self.get_logger().info(f'立方体の移動量: dx={c1[0] - c0[0]:+.3f} dy={c1[1] - c0[1]:+.3f} m')
        self.get_logger().info('DONE: 記録を Ctrl+C で止めてください')


def main():
    rclpy.init(args=sys.argv)
    n = PushDemo()
    try:
        n.run()
    except (ValueError, RuntimeError, OSError) as e:
        n.get_logger().error(str(e))
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

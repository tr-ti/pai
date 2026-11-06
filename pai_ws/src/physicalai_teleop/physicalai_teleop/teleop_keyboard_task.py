#!/usr/bin/env python3
"""CRANE+ V2 キーボードテレオペ（関節空間 + 手先空間）

講義の teleop_keyboard の操作キーをそのまま残し、手先空間の操作を追加したノード。

  関節空間（teleop_keyboard と同じキー）
    Q/E : 操作する関節を選択（選択順 joint1 -> joint2 -> joint4 -> joint3）
    W/S : 選択した関節角を増加／減少
    R/F : 全関節角を増加／減少（粗調整）
    O/C : グリッパを開く／閉じる
    X   : 直立姿勢（全関節 0）に戻し、グリッパを閉じる（起動時の状態）
    G   : ファイル名を入力し、show_arm_pose.sh の出力を保存したファイルの姿勢へ移動
    ESC : 終了
  手先空間（追加）
    ↑/↓ : 手先を前（+x）／後（-x）へ
    ←/→ : 手先を左（+y）／右（-y）へ
    I/K : 手先を上（+z）／下（-z）へ
    手先の向き（ピッチ）は、その時点の値を保ったまま位置だけを動かす。

指令は 20 Hz で出し続ける（記録時に arm / gripper 両方のトピックが必ず入るようにするため）。
"""
import math
import os
import re
import select
import sys
import termios
import tty

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint3', 'crane_plus_joint4']
GRIP = 'crane_plus_joint_hand'
ARM_TOPIC = '/crane_plus_arm_controller/joint_trajectory'
GRIP_TOPIC = '/crane_plus_gripper_controller/joint_trajectory'

# Q/E の選択順: teleop_keyboard の [joint1, joint2, joint4, joint3] に合わせた並び（添字は ARM の位置）
SELECT_ORDER = [0, 1, 3, 2]

# URDF (crane_plus.urdf.xacro) から読み取った寸法 [m] と関節範囲 [rad]
L2, L3, LH, Z2, YOFF = 0.083, 0.0935, 0.045, 0.0714, 0.0145
LIM = [(-2.6179, 2.6179), (-1.8247, 1.7837), (-2.5565, 2.4542), (-1.8400, 1.7683)]
GRIP_LIM = (-0.7089, 0.6679)


# --------------------------------------------------------------------------
# 運動学（アーム 4 関節。平面 3 リンク + 根元のヨー）
# --------------------------------------------------------------------------
def fk(q, f):
    """関節角 q=[q1..q4] -> (x, y, z, pitch)。ロボット根元座標（原点は床側の根元）。
    f は hand リンク原点から制御点までの長さ。pitch = q2 - q3 - q4（0 で鉛直上向き）。"""
    q1, q2, q3, q4 = q
    phi = q2 - q3 - q4
    rw = L2 * math.sin(q2) + L3 * math.sin(q2 - q3)
    zw = Z2 + L2 * math.cos(q2) + L3 * math.cos(q2 - q3)
    k = LH + f
    r = rw + k * math.sin(phi)
    z = zw + k * math.cos(phi)
    x = r * math.cos(q1) + YOFF * math.sin(q1)
    y = r * math.sin(q1) - YOFF * math.cos(q1)
    return x, y, z, phi


def _jacobian(q, f, h=1e-6):
    base = fk(q, f)
    cols = []
    for j in range(4):
        qp = list(q)
        qp[j] += h
        v = fk(qp, f)
        cols.append([(v[i] - base[i]) / h for i in range(4)])
    return [[cols[j][i] for j in range(4)] for i in range(4)]


def _solve4(a, b):
    n = len(b)
    m = [list(a[i]) + [b[i]] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        m[c], m[p] = m[p], m[c]
        piv = m[c][c]
        for r in range(c + 1, n):
            k = m[r][c] / piv
            for cc in range(c, n + 1):
                m[r][cc] -= k * m[c][cc]
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = (m[i][n] - sum(m[i][j] * x[j] for j in range(i + 1, n))) / m[i][i]
    return x


def solve_task(q0, target, f, iters=60, lam=0.01, tol=1e-6, accept=2e-4):
    """減衰最小二乗法（特異姿勢でも発散しにくい）で (x, y, z, pitch) を満たす関節角を求める。
    解なし／関節範囲外なら None。"""
    q = list(q0)
    for _ in range(iters):
        cur = fk(q, f)
        e = [t - c for t, c in zip(target, cur)]
        if max(abs(v) for v in e) < tol:
            break
        J = _jacobian(q, f)
        a = [[sum(J[i][k] * J[j][k] for k in range(4)) + (lam * lam if i == j else 0.0)
              for j in range(4)] for i in range(4)]
        y = _solve4(a, e)
        dq = [sum(J[i][j] * y[i] for i in range(4)) for j in range(4)]
        m = max(abs(v) for v in dq)
        if m > 0.2:
            dq = [v * 0.2 / m for v in dq]
        q = [min(max(qj + d, lo), hi) for qj, d, (lo, hi) in zip(q, dq, LIM)]
    cur = fk(q, f)
    if max(abs(t - c) for t, c in zip(target, cur)) > accept:
        return None
    return q


def task_move(q, delta, f, max_jump=0.35):
    """現在の関節角 q から、手先を delta=(dx,dy,dz) だけ動かした関節角（向きは維持）。不可なら None。"""
    x, y, z, phi = fk(q, f)
    qn = solve_task(q, (x + delta[0], y + delta[1], z + delta[2], phi), f)
    if qn is None or max(abs(a - b) for a, b in zip(qn, q)) > max_jump:
        return None
    return qn


# --------------------------------------------------------------------------
# キー入力の解析（矢印キーは ESC [ A..D の 3 バイトで届く）
# --------------------------------------------------------------------------
ARROWS = {0x41: 'UP', 0x42: 'DOWN', 0x43: 'RIGHT', 0x44: 'LEFT'}


def parse_keys(data):
    keys, i, n = [], 0, len(data)
    while i < n:
        b = data[i]
        if b != 0x1b:
            keys.append(chr(b).lower())
            i += 1
        elif i + 1 >= n:
            keys.append('ESC')          # ESC 単独
            i += 1
        elif data[i + 1] in (0x5b, 0x4f):   # ESC [ ... または ESC O ...
            j = i + 2
            while j < n and not (0x40 <= data[j] <= 0x7e):
                j += 1                   # パラメータ部（例: 1;5）を読み飛ばす
            if j < n and data[j] in ARROWS:
                keys.append(ARROWS[data[j]])
            i = j + 1
        else:
            i += 1                       # Alt+キー等は無視
    return keys


# --------------------------------------------------------------------------
# show_arm_pose.sh の出力書式（例: "crane_plus_joint1: +0.1234 rad (  +7.1 deg)"）の読み取り
# --------------------------------------------------------------------------
POSE_LINE = re.compile(r'^\s*(crane_plus_\w+)\s*:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*rad')


def parse_pose_file(path):
    """(arm=[q1..q4], grip=値 or None) を返す。4関節が揃わなければ ValueError。"""
    vals = {}
    with open(os.path.expanduser(path), encoding='utf-8') as fh:
        for line in fh:
            m = POSE_LINE.match(line)
            if m:
                vals[m.group(1)] = float(m.group(2))
    missing = [j for j in ARM if j not in vals]
    if missing:
        raise ValueError('関節の行が見つかりません: ' + ', '.join(missing))
    return [vals[j] for j in ARM], vals.get(GRIP)


# --------------------------------------------------------------------------
class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_task')
        defaults = dict(
            joint_step=0.05,      # W/S 1回の変化量 [rad]（teleop_keyboard の ARM_STEP）
            coarse_step=0.2,      # R/F 1回の変化量 [rad]（teleop_keyboard の COARSE_STEP）
            task_step=0.005,      # 矢印・I/K 1回の移動量 [m]
            tool_length=0.05,     # hand リンク原点から制御点（指先）までの長さ [m]（推定値）
            grip_open=-0.5236,    # teleop_keyboard の GRIPPER_OPEN
            grip_close=0.1745,    # teleop_keyboard の GRIPPER_CLOSE
            move_speed=0.5,       # G キーの姿勢移動の最大速さ [rad/s]（各関節）
            rate_hz=20.0,
        )
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        self.cmd = [0.0] * 4
        self.grip = self.p['grip_close']  # グリッパは閉じた状態から始める
        self.goal = None                  # G キーで指定した目標姿勢（移動中のみ）
        self.sel = 0                      # SELECT_ORDER の添字
        self.synced = False
        self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.grip_pub = self.create_publisher(JointTrajectory, GRIP_TOPIC, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self.old_tty = None
        if self.fd is not None:
            self.old_tty = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        self.create_timer(1.0 / self.p['rate_hz'], self.tick)
        self.show_help()
        self.show_selected()

    # --- 表示 -------------------------------------------------------------
    def show_help(self):
        print('\nTeleop Crane+ V2 (Joint + Task space):')
        print('---------------------------------------')
        print('[Joint space]')
        print('Q/E: Select joint (1-4)   W/S: +/- selected joint   R/F: +/- all joints (coarse)')
        print('O: Open gripper   C: Close gripper   X: Reset to vertical (all-zero, gripper closed)   ESC: Quit')
        print('G: Go to pose saved by show_arm_pose.sh (asks for file name)')
        print('[Task space]  (tool pitch is kept)')
        print('Up/Down: +x/-x (forward/back)   Left/Right: +y/-y   I/K: +z/-z (up/down)')
        print(f'step: joint {self.p["joint_step"]} rad, coarse {self.p["coarse_step"]} rad, '
              f'task {self.p["task_step"]} m, tool_length {self.p["tool_length"]} m')

    def show_selected(self):
        print(f'Selected joint: {ARM[SELECT_ORDER[self.sel]]}')

    def show_tool(self):
        x, y, z, phi = fk(self.cmd, self.p['tool_length'])
        print(f'tool: x={x:+.3f} y={y:+.3f} z={z:+.3f} m  pitch={math.degrees(phi):+.0f} deg')

    # --- 入出力 -----------------------------------------------------------
    def on_joint_state(self, msg):
        if self.synced:
            return
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in ARM):
            self.cmd = [pos[n] for n in ARM]       # 起動時の姿勢から始める（急な飛びを防ぐ）
            self.synced = True

    def read_keys(self):
        if self.fd is None or not select.select([self.fd], [], [], 0)[0]:
            return []
        return parse_keys(os.read(self.fd, 64))

    @staticmethod
    def _traj(names, pos):
        m = JointTrajectory()
        m.joint_names = list(names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in pos]
        pt.time_from_start = Duration(sec=0, nanosec=100_000_000)
        m.points.append(pt)
        return m

    def tick(self):
        if self.handle_keys(self.read_keys()):
            raise KeyboardInterrupt
        self.advance_goal()
        self.arm_pub.publish(self._traj(ARM, self.cmd))
        self.grip_pub.publish(self._traj([GRIP], [self.grip]))

    # --- キー処理（True を返すと終了）---------------------------------------
    def clamp_arm(self, q):
        return [min(max(v, lo), hi) for v, (lo, hi) in zip(q, LIM)]

    def handle_keys(self, keys):
        P = self.p
        for k in keys:
            if self.goal is not None and k not in ('ESC', 'q', 'e', 'g'):
                self.goal = None                      # 手動操作で目標姿勢への移動を中止
                print('goal move cancelled')
            if k == 'ESC':
                return True
            elif k == 'g':
                self.go_to_pose_file()
                break                                 # 入力待ちの間にたまったキーは捨てる
            elif k == 'q':
                self.sel = (self.sel - 1) % 4
                self.show_selected()
            elif k == 'e':
                self.sel = (self.sel + 1) % 4
                self.show_selected()
            elif k in ('w', 's'):
                j = SELECT_ORDER[self.sel]
                self.cmd[j] += P['joint_step'] if k == 'w' else -P['joint_step']
                self.cmd = self.clamp_arm(self.cmd)
                print(f'{ARM[j]} = {self.cmd[j]:+.3f} rad')
            elif k in ('r', 'f'):
                d = P['coarse_step'] if k == 'r' else -P['coarse_step']
                self.cmd = self.clamp_arm([v + d for v in self.cmd])
                print('all joints: ' + ' '.join(f'{v:+.3f}' for v in self.cmd))
            elif k == 'o':
                self.grip = P['grip_open']
                print('gripper: open')
            elif k == 'c':
                self.grip = P['grip_close']
                print('gripper: close')
            elif k == 'x':
                self.cmd = [0.0] * 4
                self.grip = P['grip_close']
                print('reset: vertical (all-zero), gripper closed')
            elif k in ('UP', 'DOWN', 'LEFT', 'RIGHT', 'i', 'k'):
                s = P['task_step']
                d = {'UP': (s, 0, 0), 'DOWN': (-s, 0, 0), 'LEFT': (0, s, 0), 'RIGHT': (0, -s, 0),
                     'i': (0, 0, s), 'k': (0, 0, -s)}[k]
                qn = task_move(self.cmd, d, P['tool_length'])
                if qn is None:
                    if abs(self.cmd[2]) < 0.4:
                        print('  -> 動かせません: 腕がほぼ伸び切った姿勢（直立付近や最大リーチ）では、'
                              'その向きには動かせません。逆向き、または W/S 等で腕を曲げてから操作してください')
                    else:
                        print('  -> 動かせません: 到達範囲の外、または関節の可動範囲外です')
                else:
                    self.cmd = qn
                    self.show_tool()
        return False

    def ask_line(self, prompt):
        """端末を通常モードに戻して 1 行入力し、cbreak に戻す。入力中は指令の publish が止まる。"""
        if self.fd is None:
            return ''
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_tty)
        try:
            return input(prompt).strip().strip('\'"')
        except EOFError:
            return ''
        finally:
            tty.setcbreak(self.fd)

    def go_to_pose_file(self):
        name = self.ask_line('\npose file (empty = cancel): ')
        if not name:
            print('cancelled')
            return
        try:
            arm, grip = parse_pose_file(name)
        except (OSError, ValueError) as e:
            print(f'  -> 読み込めません: {e}')
            return
        target = self.clamp_arm(arm)
        if any(abs(a - b) > 1e-9 for a, b in zip(arm, target)):
            print('  -> 関節範囲外の値は範囲内に丸めました')
        self.goal = target
        if grip is not None:
            self.grip = min(max(grip, GRIP_LIM[0]), GRIP_LIM[1])
        print('go to: ' + ' '.join(f'{v:+.3f}' for v in target)
              + ('' if grip is None else f'  gripper={self.grip:+.3f}'))

    def advance_goal(self):
        """目標姿勢に向かって、各関節を最大 move_speed で動かす（急な飛びを防ぐ）。"""
        if self.goal is None:
            return
        step = self.p['move_speed'] / self.p['rate_hz']
        self.cmd = [c + max(-step, min(step, g - c)) for c, g in zip(self.cmd, self.goal)]
        if all(abs(g - c) < 1e-6 for c, g in zip(self.cmd, self.goal)):
            self.goal = None
            print('arrived')
            self.show_tool()

    def restore_tty(self):
        if self.fd is not None and self.old_tty is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_tty)
            self.old_tty = None


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_tty()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

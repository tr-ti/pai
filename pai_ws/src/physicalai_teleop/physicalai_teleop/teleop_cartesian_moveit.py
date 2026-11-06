#!/usr/bin/env python3
"""手先空間テレオペ（MoveIt /compute_ik 版）。

ikpy 版 (teleop_cartesian) と同じトピックに同じメッセージ型を 20 Hz で出す。
IK の計算だけを move_group の /compute_ik サービスに任せる。
手先姿勢は「方位角に合わせたヨー + 一定ピッチ」に限定する（4 関節で解ける姿勢）。
前提（未検証）: joint1 の軸が根元の z 軸 / joint1=0 で +x を向く / 他3関節は y 軸まわり。
食い違う場合は qy を qx に替えるなど軸を合わせる。
"""
import sys, termios, tty, select, math
import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros

ARM = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint3', 'crane_plus_joint4']
GRIP = 'crane_plus_joint_hand'


def qmul(a, b):  # クォータニオン (x, y, z, w) の積
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2,
                     w1*w2 - x1*x2 - y1*y2 - z1*z2])


def qz(a):
    return np.array([0, 0, math.sin(a / 2), math.cos(a / 2)])


def qy(a):
    return np.array([0, math.sin(a / 2), 0, math.cos(a / 2)])


class MoveItTeleop(Node):
    def __init__(self):
        super().__init__('teleop_cartesian_moveit')
        for n, v in [('base_frame', 'crane_plus_base'),            # 要確認
                     ('tip_link', 'crane_plus_link_endeffector'),  # 要確認
                     ('step', 0.005), ('pitch_step', 0.05),
                     ('grip_open', -0.5236),    # teleop_keyboard / teleop_cartesian と同じ値
                     ('grip_close', 0.1745)]:
            self.declare_parameter(n, v)
        self.base = self.get_parameter('base_frame').value
        self.tip = self.get_parameter('tip_link').value
        self.step = self.get_parameter('step').value
        self.pstep = self.get_parameter('pitch_step').value
        self.grip = self.get_parameter('grip_open').value
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.js = {}
        self.create_subscription(JointState, 'joint_states', self.on_js, 10)
        self.pub_arm = self.create_publisher(
            JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10)
        self.pub_grip = self.create_publisher(
            JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10)
        self.p = None        # 目標位置
        self.phi = 0.0       # ピッチ変化量 [rad]
        self.Q0 = None       # joint1=0 のときの基準姿勢
        self.cmd = None
        self.busy = False
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self.create_timer(0.05, self.tick)  # 20 Hz
        self.get_logger().info('W/S:x A/D:y R/F:z T/G:pitch Q/E:step O/C:grip X:reset ESC:quit')

    def on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.js[n] = p

    def init_ref(self):
        try:
            t = self.tfbuf.lookup_transform(self.base, self.tip, rclpy.time.Time())
        except Exception:
            return
        if not all(n in self.js for n in ARM):
            return
        tr, ro = t.transform.translation, t.transform.rotation
        self.p = np.array([tr.x, tr.y, tr.z])
        qcur = np.array([ro.x, ro.y, ro.z, ro.w])
        # 現在の joint1 のヨーを打ち消して、joint1=0 のときの姿勢を得る（要検証の仮定）
        self.Q0 = qmul(qz(-self.js['crane_plus_joint1']), qcur)
        self.cmd = [self.js[n] for n in ARM]

    def target_quat(self, p):
        r = math.hypot(p[0], p[1])
        theta = math.atan2(p[1], p[0]) if r > 1e-3 else self.js['crane_plus_joint1']
        return qmul(qz(theta), qmul(qy(self.phi), self.Q0))

    def request_ik(self, p):
        if self.busy or not self.ik.service_is_ready():
            return
        q = self.target_quat(p)
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = 'arm'
        r.ik_link_name = self.tip
        r.avoid_collisions = False
        r.timeout = Duration(sec=0, nanosec=50_000_000)
        r.pose_stamped.header.frame_id = self.base
        pos = r.pose_stamped.pose.position
        pos.x, pos.y, pos.z = map(float, p)
        o = r.pose_stamped.pose.orientation
        o.x, o.y, o.z, o.w = map(float, q)
        self.busy = True
        fut = self.ik.call_async(req)
        fut.add_done_callback(lambda f, p=p.copy(): self.on_ik(f, p))

    def on_ik(self, fut, p):
        self.busy = False
        res = fut.result()
        if res is None or res.error_code.val != 1:
            self.get_logger().warn('IK失敗（到達不可の可能性）')
            return
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        new = [sol[n] for n in ARM]
        if max(abs(a - b) for a, b in zip(new, self.cmd)) > 0.3:  # 解の飛び防止
            self.get_logger().warn('IKの解が大きく飛ぶため棄却')
            return
        self.p = p
        self.cmd = new

    def traj(self, names, pos):
        m = JointTrajectory()
        m.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in pos]
        pt.time_from_start = Duration(sec=0, nanosec=100_000_000)
        m.points.append(pt)
        return m

    def tick(self):
        if self.p is None:
            self.init_ref()
            return
        s = self.step
        if select.select([sys.stdin], [], [], 0)[0]:
            k = sys.stdin.read(1)
            if k == '\x1b':
                raise KeyboardInterrupt
            d = {'w': (s, 0, 0), 's': (-s, 0, 0), 'a': (0, s, 0), 'd': (0, -s, 0),
                 'r': (0, 0, s), 'f': (0, 0, -s)}.get(k)
            if d:
                self.request_ik(self.p + np.array(d))
            elif k == 't':
                self.phi += self.pstep
                self.request_ik(self.p)
            elif k == 'g':
                self.phi -= self.pstep
                self.request_ik(self.p)
            elif k == 'q':
                self.step = max(0.001, self.step - 0.001)
            elif k == 'e':
                self.step = min(0.02, self.step + 0.001)
            elif k == 'o':
                self.grip = self.get_parameter('grip_open').value
            elif k == 'c':
                self.grip = self.get_parameter('grip_close').value
            elif k == 'x':
                self.cmd = [0.0] * 4   # 直立へ → 基準を取り直す
                self.p = None
                self.phi = 0.0
        if self.cmd is not None:
            self.pub_arm.publish(self.traj(ARM, self.cmd))
            self.pub_grip.publish(self.traj([GRIP], [self.grip]))

    def destroy_node(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        super().destroy_node()


def main():
    rclpy.init()
    n = MoveItTeleop()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import sys, termios, tty, select
import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ikpy.chain import Chain

ARM = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint3', 'crane_plus_joint4']
GRIP = 'crane_plus_joint_hand'

class CartesianTeleop(Node):
    def __init__(self):
        super().__init__('teleop_cartesian')
        self.declare_parameter('urdf', '/home/ubuntu/crane_plus.urdf')
        self.declare_parameter('base_link', 'crane_plus_base')  # 要確認
        self.declare_parameter('step', 0.005)       # 1回のキー入力で動く距離 [m]
        self.declare_parameter('grip_open', -0.5236)  # OPEN  -30 deg 
        self.declare_parameter('grip_close', 0.1745)  # CLOSE  10 deg
        urdf = self.get_parameter('urdf').value
        base = self.get_parameter('base_link').value
        c0 = Chain.from_urdf_file(urdf, base_elements=[base])
        mask = [l.name in ARM for l in c0.links]
        self.chain = Chain(c0.links, active_links_mask=mask, name='crane')
        self.idx = {l.name: i for i, l in enumerate(self.chain.links)}
        self.step = self.get_parameter('step').value
        self.q = None                     # ikpy 形式の現在関節角
        self.cmd = None                   # 指令（アーム4軸）
        self.target = None                # 先端の目標位置 [x, y, z]
        self.grip = self.get_parameter('grip_open').value
        self.create_subscription(JointState, 'joint_states', self.on_js, 10)
        self.pub_arm = self.create_publisher(
            JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10)
        self.pub_grip = self.create_publisher(
            JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10)
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self.create_timer(0.05, self.tick)  # 20 Hz
        self.get_logger().info('W/S:x  A/D:y  R/F:z  Q/E:step -/+  O/C:grip  X:reset  ESC:quit')

    def on_js(self, msg):
        q = np.zeros(len(self.chain.links))
        for n, p in zip(msg.name, msg.position):
            if n in self.idx:
                q[self.idx[n]] = p
        self.q = q
        if self.target is None:
            self.target = self.chain.forward_kinematics(q)[:3, 3].copy()
            self.cmd = [q[self.idx[n]] for n in ARM]

    def move(self, d):
        new = self.target + np.array(d)
        sol = self.chain.inverse_kinematics(new, initial_position=self.q)
        err = np.linalg.norm(self.chain.forward_kinematics(sol)[:3, 3] - new)
        if err < 0.002:                     # 2 mm 以内に解けたときだけ採用
            self.target = new
            self.cmd = [sol[self.idx[n]] for n in ARM]
        else:
            self.get_logger().warn(f'IK失敗または到達不可 (err={err:.3f} m)')

    def traj(self, names, pos):
        m = JointTrajectory()
        m.joint_names = names
        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in pos]
        p.time_from_start = Duration(sec=0, nanosec=100_000_000)
        m.points.append(p)
        return m

    def tick(self):
        if self.cmd is None:
            return
        s = self.step
        if select.select([sys.stdin], [], [], 0)[0]:
            k = sys.stdin.read(1)
            if k == '\x1b':
                raise KeyboardInterrupt
            if k == 'w': self.move([s, 0, 0])
            elif k == 's': self.move([-s, 0, 0])
            elif k == 'a': self.move([0, s, 0])
            elif k == 'd': self.move([0, -s, 0])
            elif k == 'r': self.move([0, 0, s])
            elif k == 'f': self.move([0, 0, -s])
            elif k == 'q': self.step = max(0.001, self.step - 0.001)
            elif k == 'e': self.step = min(0.02, self.step + 0.001)
            elif k == 'o': self.grip = self.get_parameter('grip_open').value
            elif k == 'c': self.grip = self.get_parameter('grip_close').value
            elif k == 'x':
                z = np.zeros(len(self.chain.links))
                self.target = self.chain.forward_kinematics(z)[:3, 3].copy()
                self.cmd = [0.0] * 4
        self.pub_arm.publish(self.traj(ARM, self.cmd))
        self.pub_grip.publish(self.traj([GRIP], [self.grip]))

    def destroy_node(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        super().destroy_node()

def main():
    rclpy.init()
    n = CartesianTeleop()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

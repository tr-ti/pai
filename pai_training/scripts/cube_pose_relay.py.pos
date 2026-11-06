#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped

ORIGIN = np.array([0.0, 0.0, 1.015])   # ロボット原点（世界座標）
SEED = np.array([0.0, -0.1, 1.015])    # 起動時の立方体の位置（世界座標）
GATE = 0.03                            # 前回位置からの許容距離 [m]

class CubePose(Node):
    def __init__(self):
        super().__init__('cube_pose_relay')
        self.pub = self.create_publisher(PoseStamped, '/cube_pose', 10)
        self.create_subscription(TFMessage, '/world/default/pose/info', self.cb, 10)
        self.ref = SEED.copy()
        self.last = 0
        self.lost = 0
        self.create_timer(5.0, self.diag)

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds
        if now - self.last < 49_000_000:
            return
        best, bd = None, 1e9
        for t in msg.transforms:
            tr = t.transform.translation
            p = np.array([tr.x, tr.y, tr.z])
            d = np.linalg.norm(p - self.ref)
            if d < bd:
                bd, best = d, (t, p)
        if best is None or bd > GATE:
            self.lost += 1
            return
        t, p = best
        self.ref = p
        self.last = now
        self.lost = 0
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'crane_plus'
        q = p - ORIGIN
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(q[0]), float(q[1]), float(q[2])
        m.pose.orientation = t.transform.rotation
        self.pub.publish(m)

    def diag(self):
        if self.lost > 0:
            self.get_logger().warn(
                f'立方体を見失いました（前回位置 {np.round(self.ref - ORIGIN, 3)} から {GATE} m 以内に物体なし）。落下か、起動時の位置のずれの可能性')

def main():
    rclpy.init(); rclpy.spin(CubePose()); rclpy.shutdown()

if __name__ == '__main__':
    main()

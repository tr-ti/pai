#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped

NAME = 'wood_cube_5cm'
ORIGIN = (0.0, 0.0, 1.015)   # ロボット原点（世界座標）

class CubePose(Node):
    def __init__(self):
        super().__init__('cube_pose_relay')
        self.pub = self.create_publisher(PoseStamped, '/cube_pose', 10)
        self.create_subscription(TFMessage, '/world/default/pose/info', self.cb, 10)
        self.last = 0

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds
        if now - self.last < 49_000_000:   # 約 20 Hz に間引く
            return
        for t in msg.transforms:
            if t.child_frame_id == NAME:
                p = PoseStamped()
                p.header.stamp = self.get_clock().now().to_msg()
                p.header.frame_id = 'crane_plus'
                p.pose.position.x = t.transform.translation.x - ORIGIN[0]
                p.pose.position.y = t.transform.translation.y - ORIGIN[1]
                p.pose.position.z = t.transform.translation.z - ORIGIN[2]
                p.pose.orientation = t.transform.rotation
                self.pub.publish(p)
                self.last = now
                return

def main():
    rclpy.init(); rclpy.spin(CubePose()); rclpy.shutdown()

if __name__ == '__main__':
    main()
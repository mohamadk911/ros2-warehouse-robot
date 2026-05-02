#!/usr/bin/env python3
import csv
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class RecordRoute(Node):
    def __init__(self):
        super().__init__('record_route')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('out_file', '/home/user/ros2_ws/routes/route.csv')
        self.declare_parameter('sample_hz', 10.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.out_file = self.get_parameter('out_file').value
        self.sample_hz = float(self.get_parameter('sample_hz').value)

        os.makedirs(os.path.dirname(self.out_file), exist_ok=True)
        self.f = open(self.out_file, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['x', 'y'])

        self.last_xy = None
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.timer = self.create_timer(1.0 / self.sample_hz, self.on_timer)

        self.get_logger().info(f"Recording odom from {self.odom_topic} -> {self.out_file}")

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.last_xy = (float(p.x), float(p.y))

    def on_timer(self):
        if self.last_xy is None:
            return
        self.w.writerow([self.last_xy[0], self.last_xy[1]])

    def close(self):
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass

def main():
    rclpy.init()
    node = RecordRoute()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
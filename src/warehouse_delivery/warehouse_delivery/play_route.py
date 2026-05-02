#!/usr/bin/env python3
import csv
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))

class PlayRoute(Node):
    def __init__(self):
        super().__init__('play_route')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('route_file', '/home/user/ros2_ws/routes/route.csv')
        self.declare_parameter('lookahead', 22)
        self.declare_parameter('goal_tol', 0.8)
        self.declare_parameter('v', 2.0)
        self.declare_parameter('k_yaw', 2.0)
        self.declare_parameter('w_max', 2.5)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.route_file = self.get_parameter('route_file').value

        self.lookahead = int(self.get_parameter('lookahead').value)
        self.goal_tol = float(self.get_parameter('goal_tol').value)
        self.v = float(self.get_parameter('v').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.w_max = float(self.get_parameter('w_max').value)

        self.route = self.load_route(self.route_file)
        self.idx = 0
        self.initialized = False
        self.done = False

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)

        self.get_logger().info(f"Loaded {len(self.route)} points from {self.route_file}")
        self.get_logger().info(f"Using odom={self.odom_topic} cmd_vel={self.cmd_topic}")

    def load_route(self, path):
        pts = []
        with open(path, 'r') as f:
            r = csv.DictReader(f)
            for row in r:
                pts.append((float(row['x']), float(row['y'])))
        if len(pts) < 5:
            raise RuntimeError("Route file too short.")
        return pts

    def stop(self):
        self.pub.publish(Twist())

    def on_odom(self, msg: Odometry):
        if self.done:
            self.stop()
            return

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        if not self.initialized:
            # start from nearest point
            best_i = 0
            best_d = 1e18
            for i, (rx, ry) in enumerate(self.route):
                d = (rx - x)**2 + (ry - y)**2
                if d < best_d:
                    best_d = d
                    best_i = i
            self.idx = best_i
            self.initialized = True
            self.get_logger().info(f"Initialized at route index {self.idx}")

        # advance index
        while self.idx < len(self.route) - 1:
            cx, cy = self.route[self.idx]
            if math.hypot(cx - x, cy - y) < self.goal_tol:
                self.idx += 1
            else:
                break

        if self.idx >= len(self.route) - 1:
            self.get_logger().info("Route complete.")
            self.done = True
            self.stop()
            return

        ti = min(self.idx + self.lookahead, len(self.route) - 1)
        tx, ty = self.route[ti]
        heading = math.atan2(ty - y, tx - x)
        yaw_err = wrap_pi(heading - yaw)

        w = max(-self.w_max, min(self.w_max, self.k_yaw * yaw_err))
        v = self.v
        if abs(yaw_err) > 0.9:
            v *= 0.4

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = PlayRoute()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
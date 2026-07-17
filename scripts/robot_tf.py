#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

def make_tf(parent, child, x=0.0, y=0.0, z=0.0):
    t = TransformStamped()
    t.header.stamp.sec = 0
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.w = 1.0
    return t

class TFPub(Node):
    def __init__(self):
        super().__init__("robot_static_tf")
        self.br = StaticTransformBroadcaster(self)
        # odom→base_footprint 现在由 bridge 动态发, 这里只发链尾
        self.br.sendTransform([
            make_tf("base_footprint", "base_link"),
            make_tf("base_link", "laser_frame", z=0.1),
        ])
        self.get_logger().info("static tf: base_footprint→base_link→laser_frame")

def main():
    rclpy.init()
    n = TFPub()
    rclpy.spin(n)

if __name__ == "__main__":
    main()

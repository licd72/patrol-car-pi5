#!/usr/bin/env python3
"""发布 odom->base_footprint TF + 静态 base_footprint->base_link->laser_frame"""
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time

class OdometryTF(Node):
    def __init__(self):
        super().__init__('odom_tf_node')
        # 动态 TF: odom -> base_footprint
        self.br = TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = time.time()

        # 静态 TF: base_footprint -> base_link -> laser_frame
        sbr = StaticTransformBroadcaster(self)
        t1 = TransformStamped()
        t1.header.frame_id = 'base_footprint'
        t1.child_frame_id = 'base_link'
        t1.transform.rotation.w = 1.0

        t2 = TransformStamped()
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'laser_frame'
        t2.transform.translation.z = 0.1
        t2.transform.rotation.w = 1.0

        sbr.sendTransform([t1, t2])
        self.create_timer(0.05, self._tick)
        self.get_logger().info('odom_tf ready: odom->base_footprint (20Hz)')

    def _tick(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OdometryTF()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

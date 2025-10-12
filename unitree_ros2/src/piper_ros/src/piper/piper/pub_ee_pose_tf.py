#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

def _normalize_quat(q):
    n = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if n == 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return q.x/n, q.y/n, q.z/n, q.w/n

class EndPoseTFBroadcaster(Node):
    def __init__(self):
        super().__init__('end_pose_tf_broadcaster')
        self.declare_parameter('pose_topic', '/end_pose')          # Pose or PoseStamped
        self.declare_parameter('stamped', False)                   # set True if PoseStamped
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'link6_from_end_pose')  # avoid colliding with RSP
        self.declare_parameter('rate_hz', 50.0)

        self.pose_topic  = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.stamped     = self.get_parameter('stamped').get_parameter_value().bool_value
        self.parent_frame= self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        self.rate_hz     = float(self.get_parameter('rate_hz').value)

        self.br = TransformBroadcaster(self)
        self.latest_pose = None
        self.latest_stamp = None

        if self.stamped:
            self.sub = self.create_subscription(PoseStamped, self.pose_topic, self.on_pose_stamped, 50)
            self.get_logger().info(f"Subscribing to PoseStamped: {self.pose_topic}")
        else:
            self.sub = self.create_subscription(Pose, self.pose_topic, self.on_pose, 50)
            self.get_logger().info(f"Subscribing to Pose (no header): {self.pose_topic}")

        self.timer = self.create_timer(1.0/self.rate_hz, self.on_timer)
        self.get_logger().info(f"Broadcasting TF {self.parent_frame} -> {self.child_frame} at {self.rate_hz:.0f} Hz")

    def on_pose_stamped(self, msg: PoseStamped):
        self.latest_pose = msg.pose
        self.latest_stamp = msg.header.stamp
        self.publish_tf(msg.pose, msg.header.stamp)

    def on_pose(self, msg: Pose):
        self.latest_pose = msg
        self.latest_stamp = self.get_clock().now().to_msg()  # best-effort stamp

    def on_timer(self):
        if self.latest_pose is None:
            return
        stamp = self.latest_stamp or self.get_clock().now().to_msg()
        self.publish_tf(self.latest_pose, stamp)

    def publish_tf(self, pose: Pose, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id  = self.child_frame
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        x,y,z,w = _normalize_quat(pose.orientation)
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = EndPoseTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

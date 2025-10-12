#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
import math

def quat_to_euler(qx, qy, qz, qw):
    """Convert quaternion to Euler angles (roll, yaw, pitch) in radians."""
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90° if out of range
    else:
        pitch = math.asin(sinp)

    return roll, yaw, pitch


class PosCmdListener(Node):
    def __init__(self):
        super().__init__('pos_cmd_listener')
        self.sub = self.create_subscription(
            Pose,          # change to PoseStamped if needed
            '/end_pose',
            self.callback,
            10
        )

    def callback(self, msg):
        if isinstance(msg, PoseStamped):
            q = msg.pose.orientation
        else:
            q = msg.orientation
        roll, yaw, pitch = quat_to_euler(q.x, q.y, q.z, q.w)
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        self.get_logger().info(
            f"x: {x:.4f} y: {y:.4f} z: {z:.4f} roll: {roll:.4f} pitch: {pitch:.4f} yaw: {yaw:.4f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PosCmdListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

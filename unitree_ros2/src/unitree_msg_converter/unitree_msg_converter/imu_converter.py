#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from unitree_go.msg import LowState
from sensor_msgs.msg import Imu

class LowstateToImu(Node):
    def __init__(self):
        super().__init__('lowstate_to_imu')
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 50)
        self.sub = self.create_subscription(
            LowState, '/lowstate', self.cb, 50)
        self.get_logger().info('lowstate_to_imu node started')

    def cb(self, msg: LowState):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'go2_imu'

        # quaternion [x,y,z,w]
        q = msg.imu_state.quaternion
        imu_msg.orientation.x = float(q[0])
        imu_msg.orientation.y = float(q[1])
        imu_msg.orientation.z = float(q[2])
        imu_msg.orientation.w = float(q[3])

        # angular velocity
        g = msg.imu_state.gyroscope
        imu_msg.angular_velocity.x = float(g[0])
        imu_msg.angular_velocity.y = float(g[1])
        imu_msg.angular_velocity.z = float(g[2])

        # linear acceleration
        a = msg.imu_state.accelerometer
        imu_msg.linear_acceleration.x = float(a[0])
        imu_msg.linear_acceleration.y = float(a[1])
        imu_msg.linear_acceleration.z = float(a[2])

        # unknown covariance
        imu_msg.orientation_covariance       = [-1.0] * 9
        imu_msg.angular_velocity_covariance  = [-1.0] * 9
        imu_msg.linear_acceleration_covariance = [-1.0] * 9

        self.pub_imu.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LowstateToImu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

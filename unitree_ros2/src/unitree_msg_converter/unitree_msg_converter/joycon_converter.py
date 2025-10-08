#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from unitree_go.msg import WirelessController
from std_msgs.msg import UInt16

class ControllerToKey(Node):
    def __init__(self):
        super().__init__('controller_to_key')
        self.pub_key = self.create_publisher(UInt16, '/controller/key', 50)
        self.sub = self.create_subscription(
            WirelessController, '/wirelesscontroller', self.cb, 50)
        self.get_logger().info('controller_to_key node started')

    def cb(self, msg: WirelessController):
        key_msg = UInt16()
        key_msg.data = msg.keys

        self.pub_key.publish(key_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerToKey()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

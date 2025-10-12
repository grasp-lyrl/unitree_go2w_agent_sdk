#!/usr/bin/env python3
# -*-coding:utf8-*-

import rospy
import rosnode
from std_msgs.msg import UInt16
from piper_msgs.msg import PosCmd
import math 

class PoseNode(object):
    def __init__(self):
        # Publisher & subscriber
        self.pose_pub = rospy.Publisher('/pos_cmd', PosCmd, queue_size=1)
        self.key_sub = rospy.Subscriber('/controller/key', UInt16, self.key_callback, queue_size=1)

        rospy.loginfo("Grasping pose node is running")

    def pose_msg_cmd(self, x, y, z, roll, pitch, yaw, gripper):
        pose_cmd = PosCmd()
        pose_cmd.x = x / 1000
        pose_cmd.y = y / 1000
        pose_cmd.z = z / 1000
        pose_cmd.roll = math.radians(roll)
        pose_cmd.yaw = math.radians(yaw)
        pose_cmd.pitch = math.radians(pitch)
        pose_cmd.gripper = gripper
        return pose_cmd

    def key_callback(self, msg):
        rospy.loginfo(f"key callback: {msg.data}")
        pose_cmd = None

        if msg.data == 4096: # up button: desk looking position
            pose_cmd = self.pose_msg_cmd(
                47.671, -1.131, 489.954,
                -175.856, 83.128, -176.167, 0
            )
        elif msg.data == 4098: # L1 + up: desk forward position
            pose_cmd = self.pose_msg_cmd(
                460, -0.131, 489.965,
                -175.856, 83.128, -176.167, 0
            )
        elif msg.data == 32768: # left botton: left ground position
            pose_cmd = self.pose_msg_cmd(
                23.258, 435.064, 49.822,
                177.580, 3.342, -89.935, 0
            )
        elif msg.data == 8192: # right button: right ground position
            pose_cmd = self.pose_msg_cmd(
                -26.375, -415.745, 37.366,
                -170.860, -1.456, 87.491, 0
            )
        elif msg.data == 16384: # down button: home base
            pose_cmd = self.pose_msg_cmd(
                47.671, -1.131, 174.954,
                -175.856, 66.128, -176.167, 0
            )
        else:
            rospy.loginfo("Key not initialized")


        if pose_cmd != None:
            self.pose_pub.publish(pose_cmd)

    def spin(self):
        rospy.spin()   

if __name__ == '__main__':
    rospy.init_node('grasping_pose_node', anonymous=False)
    try:
        PoseNode().spin()
    except rospy.ROSInterruptException:
        pass

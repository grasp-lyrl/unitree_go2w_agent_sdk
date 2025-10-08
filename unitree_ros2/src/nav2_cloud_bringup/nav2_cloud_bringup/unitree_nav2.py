#!/usr/bin/env python3
# cmd_vel_to_unitree_move.py
import math
import time
from pathlib import Path
import tempfile
import subprocess
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class CmdVelToUnitree(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_unitree_move')

        # --- params (tweak as needed) ---
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('stream_rate_hz', 3.0)      # how often to send to Unitree
        self.declare_parameter('stale_timeout', 0.25)       # if no msg for this long → stop
        self.declare_parameter('vx_max', 1.2)               # m/s  (adjust to your robot)
        self.declare_parameter('vy_max', 0.5)               # m/s
        self.declare_parameter('vyaw_max', 2.5)             # rad/s
        self.declare_parameter('vx_scale', 1.0)             # scaling hooks if needed
        self.declare_parameter('vy_scale', 1.0)
        self.declare_parameter('vyaw_scale', 1.0)
        self.declare_parameter('enable_lease', False)       # pass-through to SDK
        self.declare_parameter('stop_on_shutdown', True)

        # Read params
        self.topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.dt = 1.0 / float(self.get_parameter('stream_rate_hz').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)
        self.vx_max = float(self.get_parameter('vx_max').value)
        self.vy_max = float(self.get_parameter('vy_max').value)
        self.vyaw_max = float(self.get_parameter('vyaw_max').value)
        self.vx_scale = float(self.get_parameter('vx_scale').value)
        self.vy_scale = float(self.get_parameter('vy_scale').value)
        self.vyaw_scale = float(self.get_parameter('vyaw_scale').value)
        enable_lease = bool(self.get_parameter('enable_lease').value)
        self.stop_on_shutdown = bool(self.get_parameter('stop_on_shutdown').value)

        # State
        self.last_cmd = (0.0, 0.0, 0.0)   # vx, vy, vyaw (base_link frame)
        self.last_time = 0.0
        self.temp_dir = Path(tempfile.mkdtemp())

       
        # ROS I/O
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub = self.create_subscription(Twist, self.topic, self.cb_cmd, qos_profile)
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'Bridging {self.topic} → Unitree.Move() at {1.0/self.dt:.0f} Hz '
            f'(timeout {self.stale_timeout*1000:.0f} ms).'
        )
    

    def cb_cmd(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        vyaw = msg.angular.z

        # Scale and clamp (safety)
        vx = clamp(vx * self.vx_scale, -self.vx_max, self.vx_max)
        vy = clamp(vy * self.vy_scale, -self.vy_max, self.vy_max)
        vyaw = clamp(vyaw * self.vyaw_scale, -self.vyaw_max, self.vyaw_max)

        self.last_cmd = (vx, vy, vyaw)
        self.last_time = time.time()

    def tick(self):
        now = time.time()
        stale = (now - self.last_time) > self.stale_timeout

        if stale:
            vx, vy, vyaw = 0.0, 0.0, 0.0
        else:
            vx, vy, vyaw = self.last_cmd

        # The SDK Move expects (x= vx, y= vy, z= vyaw)
        try:
            self.move_base_to(vx, vy, vyaw)
            self.get_logger().info(f'Move command sent: x={vx}, y={vy}, yaw={vyaw}.')
        except Exception as e:
            self.get_logger().warn(f'Unitree Move() exception: {e}')

    def destroy_node(self):
        if self.stop_on_shutdown:
            try:
                self.cli.StopMove()
            except Exception:
                pass
        super().destroy_node()

    def write_code_to_file(self, code, filename):
        """Write code to a Python file in the temp directory"""
        file_path = self.temp_dir / filename
        with open(file_path, 'w') as f:
            f.write(code)
        return file_path

    def move_base_to(self, x: float, y: float, yaw: float,
                     timeout: float = 30.0) -> bool:

        move_base_code = f"""
import sys
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
import time

try:
    print("Initializing ChannelFactory...")
    ChannelFactoryInitialize(0, "eth0")
    print("ChannelFactory initialized. Creating SportClient...")
    dog_client = SportClient()
    print("SportClient created. Setting timeout...")
    dog_client.SetTimeout(10.0)
    print("Timeout set. Initializing SportClient...")
    dog_client.Init()
    print("SportClient initialized. Sending Move command...")

    # The Move command does not return a value. Check its effect by subsequent state.
    dog_client.Move({x}, {y}, {yaw})
    print(f"Move command sent: x={x}, y={y}, yaw={yaw}. Waiting for 1 second...")
    time.sleep(0.5)
    print("Move command finished.")
except Exception as e:
    print(f"Error in move_dog.py: {{e}}", file=sys.stderr)
    sys.exit(1)
"""
        move_script = self.write_code_to_file(move_base_code, "/home/unitree/unitree_ros2/src/nav2_cloud_bringup/nav2_cloud_bringup/move_dog.py")

        try:
            cmd = [
                '/bin/bash', '-c',
                f'/usr/bin/python3.8 {move_script}'
            ]
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                cwd=self.temp_dir,
                timeout=30,
                env=os.environ.copy()
            )
            if result.returncode != 0:
                self.get_logger().error(f"Error running move_dog.py (stderr): {result.stderr}")
                self.get_logger().error(f"Error running move_dog.py (stdout): {result.stdout}")
            return result
        except subprocess.TimeoutExpired:
            self.get_logger().error("move_dog.py timed out.")
            return subprocess.CompletedProcess(args=cmd, returncode=1, stdout="", stderr="TimeoutExpired")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred while moving dog: {e}")
            return subprocess.CompletedProcess(args=cmd, returncode=1, stdout="", stderr=str(e))

def main():
    rclpy.init()
    node = CmdVelToUnitree()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

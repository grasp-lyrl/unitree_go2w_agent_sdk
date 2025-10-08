#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JSBridge(Node):
    def __init__(self):
        super().__init__("js_bridge")
        # params
        self.declare_parameter("in_topic", "/joint_states_single")
        self.declare_parameter("out_topic", "/joint_states")
        self.declare_parameter("map_gripper", True)     # create joint7/8
        # If you know how gripper angle maps to finger travel (m), set scale
        self.declare_parameter("gripper_to_m_scale", 1)  # example: 1 rad -> 0.25 mm (TUNE!)
        self.declare_parameter("finger_max", 0.038)     # URDF limit for each prism

        self.in_topic  = self.get_parameter("in_topic").value
        self.out_topic = self.get_parameter("out_topic").value
        self.map_grip  = self.get_parameter("map_gripper").value
        self.scale     = float(self.get_parameter("gripper_to_m_scale").value)
        self.fmax      = float(self.get_parameter("finger_max").value)

        self.pub = self.create_publisher(JointState, self.out_topic, 10)
        self.sub = self.create_subscription(JointState, self.in_topic, self.cb, 50)

        self.get_logger().info(f"Bridging {self.in_topic} → {self.out_topic}")

    def cb(self, msg: JointState):
        # Expect joint0..joint6 (6 arm + 1 gripper)
        name_in = list(msg.name)
        pos_in  = list(msg.position)
        vel_in  = list(msg.velocity) if msg.velocity else []
        eff_in  = list(msg.effort)   if msg.effort   else []

        # Build new JointState matching URDF
        out = JointState()
        out.header = msg.header

        # Map joint0..5 -> joint1..6
        mapped_names = []
        mapped_pos   = []
        mapped_vel   = []
        mapped_eff   = []

        for i_arm in range(6):
            src = f"joint{i_arm}"
            if src in name_in:
                idx = name_in.index(src)
                mapped_names.append(f"joint{i_arm+1}")     # -> joint1..6
                mapped_pos.append(pos_in[idx])
                mapped_vel.append(vel_in[idx] if idx < len(vel_in) else 0.0)
                mapped_eff.append(eff_in[idx] if idx < len(eff_in) else 0.0)
            else:
                # missing? default to 0
                mapped_names.append(f"joint{i_arm+1}")
                mapped_pos.append(0.0); mapped_vel.append(0.0); mapped_eff.append(0.0)

        # Handle gripper -> joint7/8 (prismatic)
        if self.map_grip:
            # try to find joint6 (combined) angle
            j6_idx = name_in.index("joint6") if "joint6" in name_in else -1
            if j6_idx >= 0:
                grip_angle = pos_in[j6_idx]  # whatever your publisher uses (likely radians)
                # crude symmetric split: both fingers extend/close equally
                d = max(-self.fmax, min(self.fmax, grip_angle * self.scale))
                # URDF limits: joint7 ∈ [0, 0.038], joint8 ∈ [-0.038, 0]
                joint7 = max(0.0,  d)
                joint8 = min(0.0, -d)
            else:
                joint7 = 0.0; joint8 = 0.0

            mapped_names += ["joint7", "joint8"]
            mapped_pos   += [joint7, joint8]
            mapped_vel   += [0.0, 0.0]
            mapped_eff   += [0.0, 0.0]

        out.name     = mapped_names
        out.position = mapped_pos
        out.velocity = mapped_vel
        out.effort   = mapped_eff
        self.pub.publish(out)

def main():
    rclpy.init()
    node = JSBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()

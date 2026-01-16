#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

class JointStateToArmCmd(Node):
    def __init__(self):
        super().__init__("jointstate_to_armcmd")

        self.pub = self.create_publisher(Float32MultiArray, "/arm_joint_commands", 10)
        self.sub = self.create_subscription(JointState, "/esp32/joint_states", self.cb, 10)

        # Optional fixed gripper command (radians)
        self.declare_parameter("gripper", 0.0)

        self.get_logger().info("Bridging /esp32/joint_states -> /arm_joint_commands")

    def cb(self, msg: JointState):
        if len(msg.position) < 6:
            return

        gripper = float(self.get_parameter("gripper").value)

        out = Float32MultiArray()
        out.data = list(msg.position[:6]) + [gripper]  # 6 joints + gripper
        self.pub.publish(out)

def main():
    rclpy.init()
    node = JointStateToArmCmd()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import math, time, serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SerialToJointState(Node):
    def __init__(self):
        super().__init__('serial_to_jointstate')
        self.declare_parameter('port', '/dev/cu.SLAB_USBtoUART')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        self.pub = self.create_publisher(JointState, '/esp32/joint_states', 10)
        self.joint_names = ['joint0','joint1','joint2','joint3','joint4','joint5']

        self.ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz
        self.get_logger().info(f"Reading {port} @ {baud} and publishing /esp32/joint_states")

    def tick(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return
        parts = line.split(',')
        if len(parts) != 6:
            return
        try:
            deg = [float(x) for x in parts]
        except ValueError:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [d * math.pi / 180.0 for d in deg]  # ROS uses radians
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = SerialToJointState()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

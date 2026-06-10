from __future__ import annotations

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class CmdVelToBlueROV(Node):
    """
    Simple simulation adapter:
    /cmd_vel -> BlueROV thrust topics

    Assumption for quick planar test:
    - Thrusters 1..4 are the horizontal thrusters
    - Thrusters 5..6 are vertical and kept at zero
    - We only test surge + yaw behavior

    Mapping:
      left side thrust  = k_u * u - k_r * r
      right side thrust = k_u * u + k_r * r
    """

    def __init__(self):
        super().__init__("cmdvel_to_bluerov")

        self.declare_parameter("k_u", 20.0)
        self.declare_parameter("k_r", 10.0)
        self.declare_parameter("thrust_limit", 30.0)

        self.k_u = float(self.get_parameter("k_u").value)
        self.k_r = float(self.get_parameter("k_r").value)
        self.thrust_limit = float(self.get_parameter("thrust_limit").value)

        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)

        self.pub_t1 = self.create_publisher(Float64, "/model/bluerov2/joint/thruster1_joint/cmd_thrust", 10)
        self.pub_t2 = self.create_publisher(Float64, "/model/bluerov2/joint/thruster2_joint/cmd_thrust", 10)
        self.pub_t3 = self.create_publisher(Float64, "/model/bluerov2/joint/thruster3_joint/cmd_thrust", 10)
        self.pub_t4 = self.create_publisher(Float64, "/model/bluerov2/joint/thruster4_joint/cmd_thrust", 10)
        self.pub_t5 = self.create_publisher(Float64, "/model/bluerov2/joint/thruster5_joint/cmd_thrust", 10)
        self.pub_t6 = self.create_publisher(Float64, "/model/bluerov2/joint/thruster6_joint/cmd_thrust", 10)

        self.get_logger().info("cmdvel_to_bluerov adapter started.")

    def publish_thruster(self, pub, value: float):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    def on_cmd(self, msg: Twist):
        u = float(msg.linear.x)
        r = float(msg.angular.z)

        # Simple planar differential mapping
        left = self.k_u * u + self.k_r * r
        right = self.k_u * u - self.k_r * r

        left = clamp(left, -self.thrust_limit, self.thrust_limit)
        right = clamp(right, -self.thrust_limit, self.thrust_limit)

        # Apply same thrust to the two left-side horizontal thrusters
        # and same thrust to the two right-side horizontal thrusters.
        # If yaw direction is reversed in sim, swap signs or swap left/right.
        self.publish_thruster(self.pub_t1, left)
        self.publish_thruster(self.pub_t2, right)
        self.publish_thruster(self.pub_t3, left)
        self.publish_thruster(self.pub_t4, right)

        # Keep vertical thrusters off
        self.publish_thruster(self.pub_t5, 0.0)
        self.publish_thruster(self.pub_t6, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToBlueROV()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
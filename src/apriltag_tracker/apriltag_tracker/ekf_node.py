from __future__ import annotations
import math
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class EKF_Filter(Node):
    def __init__(self):
        super().__init__("ekf_node")

        self.declare_parameter("ekf_hz", 10.0)
        self.declare_parameter("pose_timeout", 1.0)

        self.ekf_hz = float(self.get_parameter("ekf_hz").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout").value)

        
        self.x = np.zeros((6, 1), dtype=float)

        self.P = np.eye(6) * 0.5

        # Process noise of the filter 
        self.Q = np.diag([
            0.001,    # 0.01,   # x
            0.001,    # 0.01,   # z
            0.0005,    # 0.005,  # yaw
            0.005,    # 0.05,   # vx
            0.005,    # 0.05,   # vz
            0.002,    # 0.02,   # yaw_rate
        ])

        # Measurement noise of the filter
        self.R = np.diag([
            0.25,    # 0.04,   
            0.25,    # 0.04,   
            0.10,    # 0.02,   
        ])

        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ], dtype=float)

        self.initialized = False
        self.raw_pose_valid = False
        self.last_measurement_time = -1e9
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.z_meas = np.zeros((3, 1), dtype=float)

        self.sub_pose = self.create_subscription(
            PoseStamped,
            "/dock/fused_pose",
            self.on_pose,
            10,
        )

        self.sub_valid = self.create_subscription(
            Bool,
            "/dock/fused_pose_valid",
            self.on_valid,
            10,
        )

        self.pub_pose = self.create_publisher(
            PoseStamped,
            "/dock/filtered_pose",
            10,
        )

        self.pub_valid = self.create_publisher(
            Bool,
            "/dock/filtered_pose_valid",
            10,
        )

        self.timer = self.create_timer(1.0 / self.ekf_hz, self.step)

        self.get_logger().info("EKF filter node started.")

    def yaw_from_quaternion(self, z: float, w: float) -> float:
        return wrap_angle(2.0 * math.atan2(z, w))

    def on_valid(self, msg: Bool):
        self.raw_pose_valid = bool(msg.data)

    # def on_pose(self, msg: PoseStamped):
    #     px = float(msg.pose.position.x)
    #     pz = float(msg.pose.position.z)

    #     yaw = self.yaw_from_quaternion(
    #         float(msg.pose.orientation.z),
    #         float(msg.pose.orientation.w),
    #     )

    #     self.z_meas = np.array([[px], [pz], [yaw]], dtype=float)
    #     self.last_measurement_time = self.get_clock().now().nanoseconds / 1e9

    #     if not self.initialized:
    #         self.x[0, 0] = px
    #         self.x[1, 0] = pz
    #         self.x[2, 0] = yaw
    #         self.x[3, 0] = 0.0
    #         self.x[4, 0] = 0.0
    #         self.x[5, 0] = 0.0
    #         self.initialized = True
    #         self.get_logger().info("EKF initialized from first fused pose.")
    
    
    
    def on_pose(self, msg: PoseStamped):

        if not self.raw_pose_valid:
            return

        px = float(msg.pose.position.x)
        pz = float(msg.pose.position.z)

        yaw = self.yaw_from_quaternion(
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        )

        z_new = np.array([[px], [pz], [yaw]], dtype=float)

        if self.initialized:
            dx = abs(px - self.x[0, 0])
            dz = abs(pz - self.x[1, 0])

            if dx > 1.0 or dz > 1.0:
                self.get_logger().warn(
                    f"Rejected EKF measurement jump: dx={dx:.2f}, dz={dz:.2f}"
                )
                return

        self.z_meas = z_new
        self.last_measurement_time = self.get_clock().now().nanoseconds / 1e9

        if not self.initialized:
            self.x[0, 0] = px
            self.x[1, 0] = pz
            self.x[2, 0] = yaw
            self.x[3, 0] = 0.0
            self.x[4, 0] = 0.0
            self.x[5, 0] = 0.0
            self.initialized = True
            self.get_logger().info("EKF initialized from first valid fused pose.")

    def predict(self, dt: float):
        

        F = np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1],
        ], dtype=float)

        self.x = F @ self.x
        self.x[2, 0] = wrap_angle(self.x[2, 0])

        self.P = F @ self.P @ F.T + self.Q

    def update(self):
        z = self.z_meas.copy()

        # Innovation
        y = z - (self.H @ self.x)
        y[2, 0] = wrap_angle(y[2, 0])

        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2, 0] = wrap_angle(self.x[2, 0])

        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P

    def publish_state(self, valid: bool):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "dock_center"

        msg.pose.position.x = float(self.x[0, 0])
        msg.pose.position.y = 0.0
        msg.pose.position.z = float(self.x[1, 0])

        yaw = float(self.x[2, 0])
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.pub_pose.publish(msg)
        self.pub_valid.publish(Bool(data=valid))

    def step(self):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self.ekf_hz

        if not self.initialized:
            self.pub_valid.publish(Bool(data=False))
            return

        # self.predict(dt)

        # measurement_fresh = (now - self.last_measurement_time) <= self.pose_timeout
        # measurement_ok = self.raw_pose_valid and measurement_fresh

        # if measurement_ok:
        #     self.update()

        # self.publish_state(valid=measurement_ok)
        
        measurement_fresh = (now - self.last_measurement_time) <= self.pose_timeout
        measurement_ok = self.raw_pose_valid and measurement_fresh

        if measurement_ok:
            self.predict(dt)
            self.update()
            self.publish_state(valid=True)
        else:
            # hold last pose, stop velocity drift
            self.x[3, 0] = 0.0
            self.x[4, 0] = 0.0
            self.x[5, 0] = 0.0
            self.publish_state(valid=False)


def main(args=None):
    rclpy.init(args=args)
    node = EKF_Filter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
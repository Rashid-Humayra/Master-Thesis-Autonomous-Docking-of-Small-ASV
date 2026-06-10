from __future__ import annotations
import cv2
import numpy as np
import pyapriltags
import os
import math
from dataclasses import dataclass
from typing import Dict, List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Bool, Float32MultiArray, String
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


@dataclass
class DockEstimate:
    tag_id: int
    x: float
    z: float
    yaw: float
    outlier_score: float = 0.0


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class MultiTagDockPoseEstimator(Node):
    def __init__(self):
        super().__init__("multi_tag_dock_pose_estimator")

        self.bridge = CvBridge()

        
        self.declare_parameter("tag_size", 0.24)
        self.declare_parameter("image_topic", "/world/oceans_waves/model/blueboat/link/camera_link/sensor/front_camera/image")  #"/model/blueboat/camera/image") "/image_raw")
        self.declare_parameter("annotated_topic", "/image_annotated")

        self.declare_parameter("max_outlier_score", 0.5)
        
        self.declare_parameter("publish_hz", 10.0)

        self.tag_size = float(self.get_parameter("tag_size").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.annotated_topic = str(self.get_parameter("annotated_topic").value)
        self.max_outlier_score = float(self.get_parameter("max_outlier_score").value)
        
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        
        self.last_good_fused = None
        self.max_fused_jump_x = 1.5
        self.max_fused_jump_z = 1.5
        self.max_fused_jump_yaw = 0.8

        
        
        # Tag offset from each other 
        ###############
        # Tag 0 at center, tag 1 at right and tag 3 at left, x is lateral offset and x is forward offset
        self.tag_offsets: Dict[int, np.ndarray] = {
            0: np.array([0.0, 0.0], dtype=np.float32), 
            1: np.array([-1, 1], dtype=np.float32), 
            3: np.array([1, 1], dtype=np.float32),  
        }
        ###############

        self.valid_tag_ids = set(self.tag_offsets.keys())

        
        pkg_path = get_package_share_directory("apriltag_tracker")
        calibration_path = os.path.join(pkg_path, "calibration_simulation")

        self.camera_matrix = np.loadtxt(
            os.path.join(calibration_path, "camera_matrix.txt"),
            dtype=np.float32,
        )
        
        ##this is used when using real camera, otherwise it is zeros.
        # self.dist_coeffs = np.loadtxt(
        #     os.path.join(calibration_path, "distortion_coefficients.txt"),
        #     dtype=np.float32,
        # ).reshape(-1)
        self.dist_coeffs = np.zeros((5, 1))

        
        s = self.tag_size
        self.object_points = np.array(
            [
                [-s / 2.0, -s / 2.0, 0.0],
                [ s / 2.0, -s / 2.0, 0.0],
                [ s / 2.0,  s / 2.0, 0.0],
                [-s / 2.0,  s / 2.0, 0.0],
            ],
            dtype=np.float32,
        )

        self.detector = pyapriltags.Detector(families="tag36h11")

    
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.image_pub = self.create_publisher(
            Image,
            self.annotated_topic,
            10,
        )
        
        
        ####################
        #### for individual tag publishing
        self.individual_pose_pubs = {}

        for tag_id in self.valid_tag_ids:

            self.individual_pose_pubs[tag_id] = self.create_publisher(
                PoseStamped,
                f"/dock/tag_{tag_id}_pose",
                10,
            )
        ####################

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/dock/fused_pose",
            10,
        )

        self.valid_pub = self.create_publisher(
            Bool,
            "/dock/fused_pose_valid",
            10,
        )

        self.debug_pub = self.create_publisher(
            Float32MultiArray,
            "/dock/fusion_debug",
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/dock/fusion_status",
            10,
        )

        self.get_logger().info("Multi-tag dock pose estimator started.")


    def robot_pose_relative_to_tag(self, tag_id, rvec, tvec):
        R_tag_to_cam, _ = cv2.Rodrigues(rvec)

        # Inverting transform because OpenCV gives tag pose in camera frame.
        R_cam_to_tag = R_tag_to_cam.T
        camera_pos_tag = -R_cam_to_tag @ tvec.reshape(3, 1)
        camera_pos_tag = camera_pos_tag.flatten()
        

       
        yaw = math.atan2(-R_cam_to_tag[2, 0], R_cam_to_tag[0, 0])
        yaw = wrap_angle(yaw)
        
    
        x = -float(camera_pos_tag[0])
        z = float(camera_pos_tag[2])-0.2
        
        # R_tag_to_cam, _ = cv2.Rodrigues(rvec)

        # R_cam_to_tag = R_tag_to_cam.T
        # camera_pos_tag = -R_cam_to_tag @ tvec.reshape(3, 1)
        # camera_pos_tag = camera_pos_tag.flatten()

        # # Position in tag/dock convention
        # x = float(camera_pos_tag[0])   # left positive
        # z = float(camera_pos_tag[2])   # distance toward vehicle positive

        # # Camera/vehicle x-axis expressed in tag frame
        # cam_x_in_tag = R_cam_to_tag[:, 2]

        # yaw = math.atan2(cam_x_in_tag[0], cam_x_in_tag[2])
        # yaw = wrap_angle(yaw)
        
        
        
        
        
        # self.get_logger().info(
        # f"x={x:.2f}, z={z:.2f}, yaw={yaw:.2f}"
        # )
        self.get_logger().info(
            f"tag={tag_id}, camera_pos_tag="
            f"[{camera_pos_tag[0]:.2f}, {camera_pos_tag[1]:.2f}, {camera_pos_tag[2]:.2f}], "
            f"x={x:.2f}, z={z:.2f}, yaw={yaw:.2f}"
            f"tvec={tvec}, rvec={rvec}"
        )


        return x, z, yaw

   
    def convert_tag_pose_to_dock_center(
        self,
        tag_id: int,
        robot_x_tag: float,
        robot_z_tag: float,
        robot_yaw_tag: float,
    ) -> DockEstimate:

        tag_offset = self.tag_offsets[tag_id]

       
        x_dock = robot_x_tag + float(tag_offset[0])
        z_dock = robot_z_tag + float(tag_offset[1])

        return DockEstimate(
            tag_id=tag_id,
            x=x_dock,
            z=z_dock,
            yaw=robot_yaw_tag,
        )

   
    def compute_outlier_scores(self, estimates: List[DockEstimate]) -> List[DockEstimate]:
        n = len(estimates)

        if n <= 1:
            estimates[0].outlier_score = 0.0
            return estimates

        pos_sums = []
        yaw_sums = []

        for i in range(n):
            pos_sum = 0.0
            yaw_sum = 0.0

            for j in range(n):
                if i == j:
                    continue

                dx = estimates[i].x - estimates[j].x
                dz = estimates[i].z - estimates[j].z
                pos_sum += math.sqrt(dx * dx + dz * dz)

                dyaw = wrap_angle(estimates[i].yaw - estimates[j].yaw)
                yaw_sum += abs(dyaw)

            pos_sums.append(pos_sum)
            yaw_sums.append(yaw_sum)

        pos_min, pos_max = min(pos_sums), max(pos_sums)
        yaw_min, yaw_max = min(yaw_sums), max(yaw_sums)

        for i, est in enumerate(estimates):
            if abs(pos_max - pos_min) < 1e-9:
                pos_norm = 0.0
            else:
                pos_norm = (pos_sums[i] - pos_min) / (pos_max - pos_min)

            if abs(yaw_max - yaw_min) < 1e-9:
                yaw_norm = 0.0
            else:
                yaw_norm = (yaw_sums[i] - yaw_min) / (yaw_max - yaw_min)

            
            est.outlier_score = 0.5 * pos_norm + 0.5 * yaw_norm

        return estimates

  
    def reject_outliers(self, estimates: List[DockEstimate]) -> List[DockEstimate]:
        if len(estimates) <= 1:
            return estimates

        estimates = self.compute_outlier_scores(estimates)

        reliable = [
            est for est in estimates
            if est.outlier_score <= self.max_outlier_score
        ]

       
        if len(reliable) == 0:
            #reliable = [min(estimates, key=lambda e: e.outlier_score)]
            self.publish_invalid()
            return

        return reliable

    
    def fuse_estimates(self, estimates: List[DockEstimate]) -> DockEstimate:
        if len(estimates) == 1:
            return estimates[0]

        
        weights = []
        for est in estimates:
            w = 1.0 / (est.outlier_score + 1e-3)
            weights.append(w)

        weights = np.array(weights, dtype=np.float64)
        weights = weights / np.sum(weights)

        x_fused = 0.0
        z_fused = 0.0
        yaw_sin = 0.0
        yaw_cos = 0.0

        for w, est in zip(weights, estimates):
            x_fused += w * est.x
            z_fused += w * est.z
            yaw_sin += w * math.sin(est.yaw)
            yaw_cos += w * math.cos(est.yaw)

        yaw_fused = math.atan2(yaw_sin, yaw_cos)

        return DockEstimate(
            tag_id=-1,
            x=float(x_fused),
            z=float(z_fused),
            yaw=float(yaw_fused),
            outlier_score=float(np.mean([e.outlier_score for e in estimates])),
        )

    
    def publish_fused_pose(self, fused: DockEstimate, header):
        msg = PoseStamped()
        msg.header = header
        msg.header.frame_id = "dock_center"

        msg.pose.position.x = fused.x
        msg.pose.position.y = 0.0
        msg.pose.position.z = fused.z

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(fused.yaw / 2.0)
        msg.pose.orientation.w = math.cos(fused.yaw / 2.0)

        self.pose_pub.publish(msg)
        self.valid_pub.publish(Bool(data=True))
        

    def publish_invalid(self):
        self.valid_pub.publish(Bool(data=False))

    
    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        annotated = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ########
        gray = cv2.GaussianBlur(gray, (3, 3), 0)

        clahe = cv2.createCLAHE(
            clipLimit=2.0,
            tileGridSize=(8, 8)
        )

        gray = clahe.apply(gray)
        ##############

        detections = self.detector.detect(gray)

        estimates: List[DockEstimate] = []
        
        individual_msg = PoseArray()
        individual_msg.header = msg.header
        individual_msg.header.frame_id = "dock_center"

        for det in detections:
            tag_id = int(det.tag_id)

            if tag_id not in self.valid_tag_ids:
                continue

            image_points = det.corners.astype(np.float32)

            ret, rvec, tvec = cv2.solvePnP(
                self.object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
            )

            if not ret:
                continue

            
            for i in range(4):
                p1 = tuple(det.corners[i].astype(int))
                p2 = tuple(det.corners[(i + 1) % 4].astype(int))
                cv2.line(annotated, p1, p2, (0, 255, 0), 2)

            cv2.drawFrameAxes(
                annotated,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                0.1,
            )

            robot_x_tag, robot_z_tag, robot_yaw_tag = self.robot_pose_relative_to_tag(
                tag_id,
                rvec,
                tvec,
            )

            dock_estimate = self.convert_tag_pose_to_dock_center(
                tag_id,
                robot_x_tag,
                robot_z_tag,
                robot_yaw_tag,
            )
            
            ################
            pose_msg = PoseStamped()

            pose_msg.header = msg.header
            pose_msg.header.frame_id = "dock_center"

            pose_msg.pose.position.x = float(dock_estimate.x)
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = float(dock_estimate.z)

            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = math.sin(dock_estimate.yaw / 2.0)
            pose_msg.pose.orientation.w = math.cos(dock_estimate.yaw / 2.0)

            self.individual_pose_pubs[tag_id].publish(pose_msg)
            ###################
            
            
            ##################
            # reject_this = False
            # for prev in estimates:
            #     dx = abs(dock_estimate.x - prev.x)
            #     dz = abs(dock_estimate.z - prev.z)

            #     if dx > 1 or dz > 1:
            #         reject_this = True
            #         self.get_logger().warn(
            #             f"Rejected tag {tag_id}: dx={dx:.2f}, dz={dz:.2f}"
            #         )
            #         break

            # if reject_this:
            #     continue
            ###################

            estimates.append(dock_estimate)

            text_x = int(det.center[0])
            text_y = int(det.center[1] - 30)

            cv2.putText(
                annotated,
                f"ID:{tag_id}",
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )

            cv2.putText(
                annotated,
                f"dock x:{dock_estimate.x:.2f} z:{dock_estimate.z:.2f}",
                (text_x, text_y + 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )
            

        if len(estimates) == 0:
            self.publish_invalid()
            self.status_pub.publish(String(data="NO_TAGS"))

        else:
            scored = self.compute_outlier_scores(estimates)
            reliable = self.reject_outliers(scored)
            fused = self.fuse_estimates(reliable)
            
            ######################
            
            if self.last_good_fused is not None:
                dx = abs(fused.x - self.last_good_fused.x)
                dz = abs(fused.z - self.last_good_fused.z)
                dyaw = abs(wrap_angle(fused.yaw - self.last_good_fused.yaw))

                if dx > self.max_fused_jump_x or dz > self.max_fused_jump_z or dyaw > self.max_fused_jump_yaw:
                    self.get_logger().warn(
                        f"Rejected temporal jump: dx={dx:.2f}, dz={dz:.2f}, dyaw={dyaw:.2f}"
                    )
                    self.publish_invalid()
                    return

            self.last_good_fused = fused
            ######################

            self.publish_fused_pose(fused, msg.header)

            debug_msg = Float32MultiArray()
            debug_msg.data = [
                float(len(estimates)),
                float(len(reliable)),
                float(fused.x),
                float(fused.z),
                float(fused.yaw),
                float(fused.outlier_score),
            ]
            self.debug_pub.publish(debug_msg)

            used_ids = [str(e.tag_id) for e in reliable]
            self.status_pub.publish(
                String(
                    data=(
                        f"VISIBLE={len(estimates)} "
                        f"USED={len(reliable)} "
                        f"IDS={','.join(used_ids)} "
                        f"X={fused.x:.2f} Z={fused.z:.2f} YAW={fused.yaw:.2f}"
                    )
                )
            )

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = msg.header
        self.image_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiTagDockPoseEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
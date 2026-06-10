import cv2
import math
import pyapriltags#apriltag
import numpy as np
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, Vector3Stamped
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32MultiArray, Bool


class AprilTagTracker(Node):
    
    def __init__(self):
        super().__init__('apriltag_tracker')

        self.bridge = CvBridge()
        
        self.declare_parameter("publish_hz", 30.0)
    
        pkg_path = get_package_share_directory('apriltag_tracker')
        calibration_path = os.path.join(pkg_path, 'calibration')

        #calibration_path = 'C:/Users/hrzan/Documents/NTNU 1st Semester/Specialization Project/camera-calibration/output'

        self.dist_coeffs = np.loadtxt(os.path.join(calibration_path, 'distortion_coefficients.txt'), dtype=np.float32)
        self.dist_coeffs = self.dist_coeffs.reshape(-1)
        
        self.publish_hz = float(self.get_parameter("publish_hz").value)


        # tag parameters
        tag_size = 0.22  # meters
        self.tag_ids = [0, 1, 2] #id 0 in the middle, id 1 on the right and id 2 on the left

        # defining the 3D corners of the tag in its local frame
        self.object_points = np.array([
            [-tag_size/2, -tag_size/2, 0],
            [ tag_size/2, -tag_size/2, 0],
            [ tag_size/2,  tag_size/2, 0],
            [-tag_size/2,  tag_size/2, 0]
        ], dtype=np.float32)




        #cap = cv2.VideoCapture(0) # camera initialization

        target_width = 640
        target_height = 480
        #cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_width)
        #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_height)


        self.camera_matrix = np.loadtxt(os.path.join(calibration_path, 'camera_matrix.txt'), dtype=np.float32)

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        self.camera_matrix = np.array([[fx, 0, cx],
                                    [0, fy, cy],
                                    [0,  0,  1]], dtype=np.float32)
        
        ###############
        self.tag_offsets = {
            0: np.array([0.0,  0.0]),   # [x, z]
            1: np.array([0.5,  1.0]),   # right tag 1m in front and 50cm to the right
            2: np.array([-0.5, 1.0]),   # left tag 1m in front and 50cm to the left
        }
        ###############



        self.detector = pyapriltags.Detector(families="tag36h11")
        
        self.image_sub = self.create_subscription(
            Image,
            '/model/blueboat/camera/image', #'/image_raw',
            self.image_callback,
            10
        )
        
        self.image_pub = self.create_publisher(
            Image,
            '/image_annotated',
            10
        )
        
        
        self.pose_pub = self.create_publisher(
            PoseArray,
            '/april_tags/poses',
            10
        )
        
        self.stage_pub = self.create_publisher(
            Float32MultiArray,
            '/april_tags/stage_metrics',
            10
        )
        
        self.tag0_pub = self.create_publisher(
            Vector3Stamped,
            "/dock/tag0", # gives the pose
            10
        )
        
        self.tag0_valid_pub = self.create_publisher(
            Bool,
            "/dock/tag0_valid",
            10
        )

                
        
    def image_callback(self, msg):
        
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        annotated = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   
    
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)
    
        ###### updates adding new logic
        tag_poses = {}
        

        for det in detections:
            tag_id = det.tag_id
            if tag_id not in self.tag_ids:
                continue

            # this is to get 3D pose
            image_points = det.corners.astype(np.float32)
            ret, rvec, tvec = cv2.solvePnP(self.object_points, image_points, self.camera_matrix, self.dist_coeffs)
            
            x, y, z = tvec.flatten()
            
            #This is the part for the drawing the detection frame on the tag
            if ret:
                tag_poses[tag_id] = {'rvec': rvec, 'tvec': tvec}
                
                for i in range(4):
                    pt1 = tuple(det.corners[i].astype(int))
                    pt2 = tuple(det.corners[(i+1)%4].astype(int))
                    cv2.line(annotated, pt1, pt2, (0, 255, 0), 2)
                    
                cv2.drawFrameAxes(annotated, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                cv2.putText(annotated, f"ID: {tag_id} X: {x:.2f} Z: {z:.2f} m", 
                        (int(det.center[0]), int(det.center[1]-30)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)   
            
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = msg.header
        self.image_pub.publish(annotated_msg)
        
            
        pose_array = PoseArray()
        pose_array.header = msg.header
  
        for tag_id, pose_data in tag_poses.items():
        #for tag_id , pose_data in camera_pose.items():
            
            tvec = pose_data['tvec'].flatten()
            rvec = pose_data['rvec'].flatten()
            
            #new line
            Rotation, _ = cv2.Rodrigues(rvec) # this is the actual rotation
            #new line
            Rotation_inv = Rotation.T
            #new line
            camera_pose = -Rotation.T @ tvec  # this is the position of the camera(vehicle) in the tag frame
            #new line 
            yaw = np.arctan2(Rotation_inv[1,0], Rotation_inv[0,0])

            pose = Pose()
            pose.position.x = float(camera_pose[0]) #float(tvec[0])
            pose.position.y = float(camera_pose[1]) #float(tvec[1])
            pose.position.z = float(camera_pose[2]) #float(tvec[2])
            
            #new line
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = float(np.sin(yaw / 2.0))
            pose.orientation.w = float(np.cos(yaw / 2.0))
            ########

            pose_array.poses.append(pose)

        self.pose_pub.publish(pose_array)
        


        

        # Checking if the final center tag (0) is present for Stage 3 Precision
        if 0 in tag_poses:
            tvec_0 = tag_poses[0]['tvec'].flatten()
            rvec_0 = tag_poses[0]['rvec'].flatten()
            
            
            x = tvec_0[0]  # X-coordinate of the Middle Tag
            z = tvec_0[2]  # Z-coordinate of the Middle Tag
            x_line = 0     # Assuming the 'floor vertical line' is the camera's X=0 axis
            
        
            horizontal_distance = abs(x - x_line)

            # angle calculation
            angle_rad = math.atan2(x, z) if z > 1e-6 else 0.0
            #angle_deg = np.degrees(angle_rad)
            
            v = Bool()
            v.data = True
            self.tag0_valid_pub.publish(v)   ## publishing the validity flag
            
            m = Vector3Stamped()
            m.header = msg.header
            m.header.frame_id = "camera"
            m.vector.x = x
            m.vector.y = angle_rad
            m.vector.z = z
            self.tag0_pub.publish(m)   ## publishing the pose data
            
            msg = Float32MultiArray()
            msg.data = [
                x,
                z,
                horizontal_distance,
                angle_rad
            ]
            self.stage_pub.publish(msg)
            
            
        else:
            v = Bool()
            v.data = False
            self.tag0_valid_pub.publish(v)
            
            
            


def main(args=None):
    rclpy.init(args= args)
    node = AprilTagTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
    
if __name__ == "__main__":
    main()
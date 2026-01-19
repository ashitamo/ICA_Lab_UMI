#!/usr/bin/env python3
"""AprilTag Detection and 3D Position Estimation using dt_apriltags

This node detects AprilTag markers and computes their 3D position using two methods:
1. Depth-based unprojection (preferred): Uses depth image + camera intrinsics
2. AprilTag pose estimation (fallback): Uses detector's built-in PnP algorithm

The node publishes:
- apriltag_position: 3D position from AprilTag detector pose estimation
- depth_position: 3D position from depth unprojection
- position_error: Difference between the two methods

Displays live visualization with detected markers and 3D coordinates.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from dt_apriltags import Detector
import message_filters

# Configuration
TARGET_MARKER_ID = 0        # Which AprilTag ID to detect
MARKER_LENGTH = 0.103       # Physical size of the tag in meters (measure your tag!)
SHOW_VISUALIZATION = True   # Set to False to disable cv2.imshow window

# Camera topic names - UPDATE THESE if your camera uses different topics
COLOR_TOPIC = '/camera/color/image_rect_raw'
DEPTH_TOPIC = '/camera/depth/image_rect_raw'


class AprilTagDetector(Node):
    """ROS2 node for AprilTag detection and 3D position estimation."""
    
    def __init__(self):
        super().__init__('apriltag_detector')
        
        # Camera intrinsics - UPDATE THESE FOR YOUR CAMERA
        # You can get these from camera calibration or camera_info topic
        self.camera_matrix = np.array([
            [439.09984732, 0.0, 440.97311686],
            [0.0, 428.10459275, 236.80079264],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        
        # Distortion coefficients: [k1, k2, p1, p2, k3]
        # k1, k2, k3 = radial distortion
        # p1, p2 = tangential distortion
        # These correct for lens distortion (barrel/pincushion effects)
        self.dist_coeffs = np.array([
            -0.05268922, 0.04680826, -0.00317253, 0.00438352, 0
        ], dtype=np.float64)

        self.new_mtx, self.roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (848,480), 1, (848,480))
        
        # Extract intrinsic parameters
        self.fx = self.camera_matrix[0, 0]  # Focal length X
        self.fy = self.camera_matrix[1, 1]  # Focal length Y
        self.cx = self.camera_matrix[0, 2]  # Principal point X
        self.cy = self.camera_matrix[1, 2]  # Principal point Y
        
        # Initialize AprilTag detector
        self.detector = Detector(
            families='tag36h11',      # AprilTag family
            nthreads=1,               # Number of threads
            quad_decimate=1.0,        # Detection decimation (1.0 = no decimation)
            quad_sigma=0.0,           # Gaussian blur sigma
            refine_edges=1,           # Edge refinement
            decode_sharpening=0.25,   # Sharpening for decoding
            debug=0                   # Debug level (0 = no debug)
        )
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.pub_apriltag = self.create_publisher(Point, 'apriltag_position', 10)
        self.pub_depth = self.create_publisher(Point, 'depth_position', 10)
        self.pub_error = self.create_publisher(Float32MultiArray, 'position_error', 10)
        
                # Synchronized subscribers for color and depth images
        color_sub = message_filters.Subscriber(self, Image, COLOR_TOPIC)
        depth_sub = message_filters.Subscriber(self, Image, DEPTH_TOPIC)
        
        # Time synchronizer (allows 100ms difference between messages)
        ts = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], 
            queue_size=10, 
            slop=0.1
        )
        ts.registerCallback(self.image_callback)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('AprilTag Detector Node Started')
        self.get_logger().info(f'  Target Marker ID: {TARGET_MARKER_ID}')
        self.get_logger().info(f'  Marker Size: {MARKER_LENGTH*100:.2f} cm ({MARKER_LENGTH:.4f} m)')
        self.get_logger().info(f'  AprilTag Family: tag36h11')
        self.get_logger().info(f'  Visualization: {"Enabled" if SHOW_VISUALIZATION else "Disabled"}')
        self.get_logger().info('=' * 70)
        self.get_logger().info('Waiting for camera images on topics:')
        self.get_logger().info(f'  - {COLOR_TOPIC}')
        self.get_logger().info(f'  - {DEPTH_TOPIC}')
        self.get_logger().info('If no images appear, check: ros2 topic list | grep camera')
        self.get_logger().info('=' * 70)
    
    def image_callback(self, color_msg, depth_msg):
        """Process synchronized color and depth images."""
        
        # Debug: Log that we received images
        self.get_logger().info('Received images - processing...', throttle_duration_sec=2.0)
        
        # Convert ROS images to OpenCV format
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return
        
        # Debug: Log image size
        self.get_logger().info(f'Image size: {color_img.shape}', throttle_duration_sec=5.0)
        
        # Convert to grayscale for detection
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        
        # Undistort the image using camera distortion coefficients
        # This corrects for lens distortion before detection
        gray_undistorted = cv2.undistort(gray, self.camera_matrix, self.dist_coeffs, None, self.new_mtx)
        x, y, w, h = self.roi
        gray_undistorted = gray_undistorted[y:y+h, x:x+w]
        
        # Detect AprilTags with pose estimation enabled on UNDISTORTED image
        tags = self.detector.detect(
            gray_undistorted,  # Use undistorted image!
            estimate_tag_pose=True,
            camera_params=[self.new_mtx[0,0], self.new_mtx[1,1], self.new_mtx[0,2], self.new_mtx[1,2]],
            tag_size=MARKER_LENGTH
        )
        
        # Debug: Log detection results
        self.get_logger().info(f'Detected {len(tags)} AprilTags', throttle_duration_sec=1.0)
        if len(tags) > 0:
            tag_ids = [tag.tag_id for tag in tags]
            self.get_logger().info(f'Tag IDs found: {tag_ids}')
        
        # Check if any tags were detected
        if len(tags) == 0:
            # Show camera feed even when no tags detected
            if SHOW_VISUALIZATION:
                display_img = color_img.copy()
                cv2.putText(display_img, 'No AprilTags detected', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                cv2.putText(display_img, f'Looking for tag36h11 ID {TARGET_MARKER_ID}', (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.namedWindow('AprilTag Detection', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('AprilTag Detection', 1280, 960)
                cv2.imshow('AprilTag Detection', display_img)
                cv2.waitKey(1)
            return
        
        # Find the target marker
        target_tag = None
        for tag in tags:
            if tag.tag_id == TARGET_MARKER_ID:
                target_tag = tag
                break
        
        if target_tag is None:
            return
        
        # Process the detected tag
        self.process_tag(target_tag, color_img, depth_img, depth_msg.encoding)
    
    def process_tag(self, tag, color_img, depth_img, depth_encoding):
        """Process detected AprilTag and compute 3D positions."""
        
        # Extract tag information
        tag_id = tag.tag_id
        corners = tag.corners      # (4, 2) array of corner pixel positions
        center = tag.center        # (2,) array of center pixel position
        
        # Method 1: AprilTag's built-in pose estimation
        apriltag_pos = None
        if tag.pose_t is not None:
            apriltag_pos = tuple(tag.pose_t.flatten())
            self.get_logger().info(f'[AprilTag Pose] X={apriltag_pos[0]:.4f}, Y={apriltag_pos[1]:.4f}, Z={apriltag_pos[2]:.4f} m')
        
        # Method 2: Depth-based unprojection
        depth_pos = self.compute_depth_position(corners, depth_img, depth_encoding)
        
        # Publish positions
        if apriltag_pos is not None:
            msg = Point()
            msg.x, msg.y, msg.z = apriltag_pos
            self.pub_apriltag.publish(msg)
        
        if depth_pos is not None:
            msg = Point()
            msg.x, msg.y, msg.z = depth_pos
            self.pub_depth.publish(msg)
        
        # Compute and publish error if both methods succeeded
        if apriltag_pos is not None and depth_pos is not None:
            error = np.array(apriltag_pos) - np.array(depth_pos)
            error_magnitude = np.linalg.norm(error)
            
            error_msg = Float32MultiArray()
            error_msg.data = [float(error[0]), float(error[1]), float(error[2]), float(error_magnitude)]
            self.pub_error.publish(error_msg)
            
            self.get_logger().info(f'[Error] ΔX={error[0]*1000:.1f}mm, ΔY={error[1]*1000:.1f}mm, ΔZ={error[2]*1000:.1f}mm, |error|={error_magnitude*1000:.1f}mm')
        
        # Visualization
        if SHOW_VISUALIZATION:
            self.visualize(color_img, tag, apriltag_pos, depth_pos)
    
    def compute_depth_position(self, corners, depth_img, encoding):
        """Compute 3D position using depth image unprojection.
        
        Args:
            corners: (4, 2) array of corner pixel positions
            depth_img: Depth image array
            encoding: Depth image encoding ('16UC1' or '32FC1')
        
        Returns:
            tuple: (X, Y, Z) in camera frame in meters, or None if failed
        """
        
        # Compute marker center pixel
        center_u = int(np.mean(corners[:, 0]))
        center_v = int(np.mean(corners[:, 1]))
        
        # Sample a small patch around the center for robustness
        patch_size = 5
        v_start = max(0, center_v - patch_size // 2)
        v_end = min(depth_img.shape[0], center_v + patch_size // 2 + 1)
        u_start = max(0, center_u - patch_size // 2)
        u_end = min(depth_img.shape[1], center_u + patch_size // 2 + 1)
        
        depth_patch = depth_img[v_start:v_end, u_start:u_end]
        
        # Filter out invalid depth values (0 or negative)
        valid_depths = depth_patch[depth_patch > 0]
        
        if len(valid_depths) == 0:
            self.get_logger().warn('No valid depth values at marker center')
            return None
        
        # Use median for robustness against outliers
        depth_value = np.median(valid_depths)
        
        # Convert depth to meters based on encoding
        if encoding == '16UC1':
            Z = float(depth_value) / 1000.0  # mm to m
        elif encoding == '32FC1':
            Z = float(depth_value)           # already in m
        else:
            # Heuristic: values > 1000 are likely in mm
            if depth_value > 1000:
                Z = float(depth_value) / 1000.0
            else:
                Z = float(depth_value)
        
        # Unproject pixel to 3D camera coordinates
        # Using pinhole camera model: X = (u - cx) * Z / fx
        X = (center_u - self.cx) * Z / self.fx
        Y = (center_v - self.cy) * Z / self.fy
        
        self.get_logger().info(f'[Depth Method] Pixel=({center_u}, {center_v}), Z={Z:.4f}m → X={X:.4f}, Y={Y:.4f}, Z={Z:.4f} m')
        
        return (X, Y, Z)
    
    def visualize(self, img, tag, apriltag_pos, depth_pos):
        """Create visualization overlay on image.
        
        Args:
            img: Color image (BGR)
            tag: Detected AprilTag object
            apriltag_pos: Position from AprilTag pose estimation
            depth_pos: Position from depth unprojection
        """
        
        display_img = img.copy()
        
        # Draw tag boundary (green)
        corners_int = tag.corners.astype(int)
        cv2.polylines(display_img, [corners_int], isClosed=True, color=(0, 255, 0), thickness=2)
        
        # Draw center point (red circle)
        center = tuple(tag.center.astype(int))
        cv2.circle(display_img, center, radius=8, color=(0, 0, 255), thickness=-1)
        
        # Draw crosshair at center (green)
        cv2.line(display_img, (center[0] - 15, center[1]), (center[0] + 15, center[1]), (0, 255, 0), 2)
        cv2.line(display_img, (center[0], center[1] - 15), (center[0], center[1] + 15), (0, 255, 0), 2)
        
        # Draw tag ID
        cv2.putText(display_img, f'ID: {tag.tag_id}',
                    (center[0] - 30, center[1] - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Display positions on image
        y_offset = 30
        
        # AprilTag pose position (cyan)
        if apriltag_pos is not None:
            text = f'AprilTag: X={apriltag_pos[0]:.3f} Y={apriltag_pos[1]:.3f} Z={apriltag_pos[2]:.3f}m'
            cv2.putText(display_img, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            y_offset += 35
        
        # Depth position (yellow)
        if depth_pos is not None:
            text = f'Depth:    X={depth_pos[0]:.3f} Y={depth_pos[1]:.3f} Z={depth_pos[2]:.3f}m'
            cv2.putText(display_img, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            y_offset += 35
        
        # Error magnitude (red)
        if apriltag_pos is not None and depth_pos is not None:
            error = np.array(apriltag_pos) - np.array(depth_pos)
            error_mag = np.linalg.norm(error)
            text = f'Error: {error_mag*1000:.1f}mm'
            cv2.putText(display_img, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Display image
        cv2.namedWindow('AprilTag Detection', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('AprilTag Detection', 1280, 960)
        cv2.imshow('AprilTag Detection', display_img)
        cv2.waitKey(1)


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    node = AprilTagDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down AprilTag Detector...')
    finally:
        if SHOW_VISUALIZATION:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

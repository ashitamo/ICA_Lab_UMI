#!/usr/bin/env python3
"""ROS2 (Foxy) port of distance.py

This node subscribes to color and depth images, detects an ArUco marker,
computes position via solvePnP and via depth unprojection, and publishes
the two positions plus the error for comparison.

Compatible with OpenCV 4.x (prefers ArucoDetector when available).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import cv2.aruco as aruco
import message_filters

TARGET_MARKER_ID = 0

MARKER_LENGTH = 0.018  # unit: m


class ArucoPositionPublisher(Node):
    def __init__(self):
        super().__init__('aruco_position_publisher')

        # Flag to ensure we only print once
        self.printed_once = False

        # Publishers
        self.pub_aruco = self.create_publisher(Point, 'aruco_position', 10)
        self.pub_depth = self.create_publisher(Point, 'depth_position', 10)
        self.pub_error = self.create_publisher(Float32MultiArray, 'position_error', 10)

        # Camera intrinsics (from original script)
        self.camera_matrix = np.array([[439.09984732, 0.0, 440.97311686],
                                      [0.0, 428.10459275, 236.80079264],
                                      [0.0, 0.0, 1.0]], dtype=np.float64)
        self.dist_coeffs = np.array([-0.05268922, 0.04680826, -0.00317253, 0.00438352, 0], dtype=np.float64)

        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]

        # --- ArUco settings with compatibility across OpenCV versions ---
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = None
        self.aruco_detector = None

        # Try DetectorParameters_create() first
        if hasattr(aruco, 'DetectorParameters_create'):
            try:
                self.aruco_params = aruco.DetectorParameters_create()
                self.get_logger().info('Using aruco.DetectorParameters_create()')
            except Exception:
                self.aruco_params = None

        # Fallback to DetectorParameters class if present
        if self.aruco_params is None and hasattr(aruco, 'DetectorParameters'):
            try:
                self.aruco_params = aruco.DetectorParameters()
                self.get_logger().info('Using aruco.DetectorParameters()')
            except Exception:
                self.aruco_params = None

        # Prefer ArucoDetector (new API available in newer OpenCV)
        if hasattr(aruco, 'ArucoDetector'):
            try:
                if self.aruco_params is not None:
                    self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
                else:
                    self.aruco_detector = aruco.ArucoDetector(self.aruco_dict)
                self.get_logger().info('Using aruco.ArucoDetector()')
            except Exception:
                self.aruco_detector = None

        self.bridge = CvBridge()

        self.get_logger().info('ArucoPositionPublisher (ROS2) started. Detecting marker ID %d' % TARGET_MARKER_ID)

        # Subscribers and time synchronizer
        color_sub = message_filters.Subscriber(self, Image, '/camera/color/image_rect_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_rect_raw')
        ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1)
        ts.registerCallback(self.sync_callback)

    def sync_callback(self, color_msg, depth_msg):
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().warn('CvBridge error: %s' % str(e))
            return

        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

        # Detection using ArucoDetector or fallback
        if getattr(self, 'aruco_detector', None) is not None:
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        else:
            if self.aruco_params is not None:
                corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            else:
                corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict)

        if ids is None or len(ids) == 0:
            return

        # find target marker
        target_idx = None
        for i, marker_id in enumerate(ids):
            if marker_id[0] == TARGET_MARKER_ID:
                target_idx = i
                break

        if target_idx is None:
            return

        corner = corners[target_idx]

        # Visualize the detected marker and center point (every frame)
        display_img = color_img.copy()
        
        # Debug: check if image is valid
        self.get_logger().info(f'Image shape: {display_img.shape}, dtype: {display_img.dtype}, min: {display_img.min()}, max: {display_img.max()}')
        
        # Draw the ArUco marker corners
        cv2.aruco.drawDetectedMarkers(display_img, corners, ids)
        
        # Compute and draw the center point
        corners_flat = corner[0]
        center_u = int(np.mean(corners_flat[:, 0]))
        center_v = int(np.mean(corners_flat[:, 1]))
        
        self.get_logger().info(f'Center pixel: ({center_u}, {center_v})')
        
        # Draw center point (large red circle)
        cv2.circle(display_img, (center_u, center_v), 8, (0, 0, 255), -1)
        
        # Draw crosshair at center
        cv2.line(display_img, (center_u - 15, center_v), (center_u + 15, center_v), (0, 255, 0), 2)
        cv2.line(display_img, (center_u, center_v - 15), (center_u, center_v + 15), (0, 255, 0), 2)
        
        # Add text showing pixel coordinates
        cv2.putText(display_img, f'Center: ({center_u}, {center_v})', 
                    (center_u + 20, center_v - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Compute 3D positions to display on image
        aruco_pos = self.compute_aruco_position(corner)
        depth_pos = self.compute_depth_position(corner, depth_img, getattr(depth_msg, 'encoding', ''))
        
        # Display ArUco (solvePnP) position on image
        y_offset = 30
        if aruco_pos is not None:
            cv2.putText(display_img, f'ArUco: X={aruco_pos[0]:.3f} Y={aruco_pos[1]:.3f} Z={aruco_pos[2]:.3f}m', 
                        (10, y_offset), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            y_offset += 35
        
        # Display Depth position on image
        if depth_pos is not None:
            cv2.putText(display_img, f'Depth: X={depth_pos[0]:.3f} Y={depth_pos[1]:.3f} Z={depth_pos[2]:.3f}m', 
                        (10, y_offset), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            y_offset += 35
        
        # Display error if both available
        if aruco_pos is not None and depth_pos is not None:
            error = np.array(aruco_pos) - np.array(depth_pos)
            error_magnitude = np.linalg.norm(error)
            cv2.putText(display_img, f'Error: {error_magnitude*1000:.1f}mm', 
                        (10, y_offset), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Save one frame for debugging
        #cv2.imwrite('/tmp/aruco_debug.png', display_img)
        #self.get_logger().info('Saved debug image to /tmp/aruco_debug.png')
        
        # Show the image (updates every frame) - make window bigger
        cv2.namedWindow('ArUco Detection - Live', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('ArUco Detection - Live', 1280, 960)  # Make window larger
        cv2.imshow('ArUco Detection - Live', display_img)
        cv2.waitKey(1)  # Reduced wait time for smoother updates

        # Stop position calculation/logging if already printed once
        if self.printed_once:
            return

        # Recompute positions for publishing (already computed for display above)
        aruco_pos = self.compute_aruco_position(corner)
        depth_pos = self.compute_depth_position(corner, depth_img, getattr(depth_msg, 'encoding', ''))

        if aruco_pos is not None and depth_pos is not None:
            # Publish ArUco position
            aruco_msg = Point()
            aruco_msg.x, aruco_msg.y, aruco_msg.z = aruco_pos
            self.pub_aruco.publish(aruco_msg)

            # Publish depth position
            depth_msg_out = Point()
            depth_msg_out.x, depth_msg_out.y, depth_msg_out.z = depth_pos
            self.pub_depth.publish(depth_msg_out)

            # Error
            error = np.array(aruco_pos) - np.array(depth_pos)
            error_magnitude = np.linalg.norm(error)
            error_msg = Float32MultiArray()
            error_msg.data = [float(error[0]), float(error[1]), float(error[2]), float(error_magnitude)]
            self.pub_error.publish(error_msg)

            self.get_logger().info('=' * 60)
            self.get_logger().info('COMPARISON RESULTS (Marker ID %d):' % TARGET_MARKER_ID)
            self.get_logger().info('ArUco (solvePnP):   X=%.4f Y=%.4f Z=%.4f [m]' % tuple(aruco_pos))
            self.get_logger().info('Depth (unproject):  X=%.4f Y=%.4f Z=%.4f [m]' % tuple(depth_pos))
            self.get_logger().info('Error (dx,dy,dz):   %.4f, %.4f, %.4f [m]' % (error[0], error[1], error[2]))
            self.get_logger().info('Error magnitude:    %.4f m = %.1f mm' % (error_magnitude, error_magnitude * 1000))
            self.get_logger().info('=' * 60)

            self.printed_once = True
            # Optionally stop the node after one comparison
            # self.get_logger().info('Comparison complete - shutting down')
            # rclpy.shutdown()

    def compute_aruco_position(self, corner):
        retval, rvec, tvec = cv2.solvePnP(
            objectPoints=np.array([
                [-MARKER_LENGTH/2,  MARKER_LENGTH/2, 0],
                [ MARKER_LENGTH/2,  MARKER_LENGTH/2, 0],
                [ MARKER_LENGTH/2, -MARKER_LENGTH/2, 0],
                [-MARKER_LENGTH/2, -MARKER_LENGTH/2, 0]
            ], dtype=np.float32),
            imagePoints=corner[0],
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if retval:
            return tuple(tvec.flatten())
        return None

    def compute_depth_position(self, corner, depth_img, encoding):
        corners_flat = corner[0]
        center_u = int(np.mean(corners_flat[:, 0]))
        center_v = int(np.mean(corners_flat[:, 1]))

        patch_size = 5
        v_start = max(0, center_v - patch_size//2)
        v_end = min(depth_img.shape[0], center_v + patch_size//2 + 1)
        u_start = max(0, center_u - patch_size//2)
        u_end = min(depth_img.shape[1], center_u + patch_size//2 + 1)

        depth_patch = depth_img[v_start:v_end, u_start:u_end]
        try:
            depth_patch = depth_patch[depth_patch > 0]
        except Exception:
            # if depth_patch is not a numpy array or comparison fails
            depth_patch = np.array(depth_patch).flatten()
            depth_patch = depth_patch[depth_patch > 0]

        if len(depth_patch) == 0:
            self.get_logger().warn('No valid depth at marker center')
            return None

        depth_value = np.median(depth_patch)

        if encoding == '16UC1':
            Z = depth_value / 1000.0
        elif encoding == '32FC1':
            Z = depth_value
        else:
            # Some ROS2 setups may not set encoding as expected; try to detect units heuristically
            if depth_value > 1000:  # likely in mm
                Z = depth_value / 1000.0
            else:
                Z = float(depth_value)

        X = (center_u - self.cx) * Z / self.fx
        Y = (center_v - self.cy) * Z / self.fy

        return (X, Y, Z)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
AprilTag Grid Pose Estimation Module
Provides functions to detect AprilTags in a grid and compute mean transformation to tag 21 frame.
Can be imported and used by other scripts.
"""

import numpy as np
import cv2
from aprilgrid import Detector
from scipy.spatial.transform import Rotation


def get_mean_transform_to_tag21(image, 
                                 tag_size=0.022,
                                 tag_spacing_ratio=0.3,
                                 camera_matrix=None,
                                 dist_coeffs=None,
                                 visualize=False):
    """
    Calculate mean transformation from camera to tag 21 using AprilGrid detection.
    
    Args:
        image (np.array): Input image (BGR or grayscale)
        tag_size (float): Size of individual AprilTag in meters (default: 0.022)
        tag_spacing_ratio (float): Spacing between tags as ratio of tag_size (default: 0.3)
        camera_matrix (np.array): 3x3 camera intrinsic matrix (optional)
        dist_coeffs (np.array): Camera distortion coefficients (optional)
        visualize (bool): If True, returns visualization image
        
    Returns:
        np.array: Mean 4x4 transformation matrix from camera to tag 21
        Returns None if no tags detected.
    """
    # Default camera parameters (RealSense D435)
    if camera_matrix is None:
        camera_matrix = np.array([
            [439.36662085, 0.0, 425.84192341],
            [0.0, 440.59543929, 239.69119773],
            [0.0, 0.0, 1.0]
        ])
    
    if dist_coeffs is None:
        dist_coeffs = np.array([-0.04084391, 0.03451489, 0.00108604, 0.00311664])
    
    # Calculate tag distance
    tag_dist = tag_size * tag_spacing_ratio + tag_size
    
    # Initialize detector
    detector = Detector("t36h11")
    
    # Convert to grayscale if needed
    if len(image.shape) == 3:
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        img_color = image.copy()
    else:
        img_gray = image
        img_color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    
    # Detect AprilTags
    detections = detector.detect(img_gray)
    
    if len(detections) == 0:
        return None
    
    # Storage for transformations
    transforms_to_21 = []
    positions_to_21 = []
    tag_ids = []
    
    # Define object points for PnP
    half_size = tag_size / 2
    object_points = np.array([
        [-half_size, -half_size, 0],  # Bottom-left
        [half_size, -half_size, 0],   # Bottom-right
        [half_size, half_size, 0],    # Top-right
        [-half_size, half_size, 0]    # Top-left
    ], dtype=np.float32)
    
    # Process each detection
    for detection in detections:
        tag_id = detection.tag_id
        corners = detection.corners
        image_points = np.array(corners, dtype=np.float32)
        
        # Solve PnP to get pose
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs
        )
        
        if not success:
            print(f"PnP failed for tag {tag_id}, skipping...")
            continue
        
        # Build T_C_tagID (camera to current tag)
        R_C_tagID, _ = cv2.Rodrigues(rvec)
        T_C_tagID = np.eye(4)
        T_C_tagID[:3, :3] = R_C_tagID
        T_C_tagID[:3, 3] = tvec.flatten()
        
        # Get transformation from current tag to tag 21
        T_tagID_21 = _get_transform_tagID_to_tag21(tag_id, tag_dist)
        
        # Chain transformations: camera → tagID → tag21
        T_C_21 = T_C_tagID @ T_tagID_21
        
        # Extract camera position in tag 21's frame
        camera_pos_in_21 = T_C_21[:3, 3]
        
        transforms_to_21.append(T_C_21)
        positions_to_21.append(camera_pos_in_21)
        tag_ids.append(tag_id)
        
        # Visualization
        if visualize:
            corners_array = np.array(corners).reshape(-1, 2)
            center = np.mean(corners_array, axis=0)
            center_tuple = (int(center[0]), int(center[1]))
            
            # Draw center and tag ID
            cv2.circle(img_color, center_tuple, 5, (255, 0, 255), -1)
            cv2.putText(img_color, f"{tag_id}", center_tuple,
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # Draw corners
            for j in range(len(corners_array)):
                c = corners_array[j]
                c_int = (int(c[0]), int(c[1]))
                corner_id = tag_id * 4 + j
                cv2.putText(img_color, f"{corner_id}", c_int,
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
                cv2.circle(img_color, c_int, 3, (0, 255, 0), -1)
    
    # Calculate mean transformation
    if len(transforms_to_21) == 0:
        return None
    
    # Average positions
    mean_position = np.mean(positions_to_21, axis=0)
    
    # Average rotations using quaternions
    rotations = [Rotation.from_matrix(T[:3, :3]) for T in transforms_to_21]
    quaternions = np.array([r.as_quat() for r in rotations])
    mean_quat = np.mean(quaternions, axis=0)
    mean_quat = mean_quat / np.linalg.norm(mean_quat)  # Normalize
    mean_rotation = Rotation.from_quat(mean_quat)
    
    # Build mean transformation matrix
    mean_T_C_21 = np.eye(4)
    mean_T_C_21[:3, :3] = mean_rotation.as_matrix()
    mean_T_C_21[:3, 3] = mean_position
    if visualize:
        return mean_T_C_21, img_color
    return mean_T_C_21


def _get_transform_tagID_to_tag21(tag_id, tag_dist):
    """
    Calculate transformation from given tag to tag 21 based on grid geometry.
    
    Args:
        tag_id (int): ID of the source tag
        tag_dist (float): Distance between tag centers in meters
        
    Returns:
        np.array: 4x4 transformation matrix from tag_id to tag 21
    """
    x_21 = 21 % 6
    y_21 = 21 // 6
    x = tag_id % 6
    y = tag_id // 6
    
    if x != x_21 or y != y_21:
        x_diff = x_21 - x
        y_diff = y_21 - y
        
        # Build 4×4 transformation matrices
        T_x = np.array([
            [1, 0, 0, x_diff * tag_dist],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        T_y = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, y_diff * tag_dist],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        T_tagID_21 = T_x @ T_y
        return T_tagID_21
    else:
        return np.identity(4)


# Convenience class for repeated calls (more efficient)
class AprilGridPoseEstimator:
    """
    Class for efficient repeated pose estimation calls.
    Detector is initialized once and reused.
    """
    
    def __init__(self, tag_size=0.022, tag_spacing_ratio=0.3,
                 camera_matrix=None, dist_coeffs=None):
        """
        Initialize the pose estimator.
        
        Args:
            tag_size (float): Size of individual AprilTag in meters
            tag_spacing_ratio (float): Spacing between tags as ratio of tag_size
            camera_matrix (np.array): 3x3 camera intrinsic matrix
            dist_coeffs (np.array): Camera distortion coefficients
        """
        self.detector = Detector("t36h11")
        self.tag_size = tag_size
        self.tag_dist = tag_size * tag_spacing_ratio + tag_size
        
        # Default camera parameters (RealSense D435)
        if camera_matrix is None:
            self.camera_matrix = np.array([
                [439.36662085, 0.0, 425.84192341],
                [0.0, 440.59543929, 239.69119773],
                [0.0, 0.0, 1.0]
            ])
        else:
            self.camera_matrix = camera_matrix
            
        if dist_coeffs is None:
            self.dist_coeffs = np.array([-0.04084391, 0.03451489, 0.00108604, 0.00311664])
        else:
            self.dist_coeffs = dist_coeffs
    
    def estimate_pose(self, image, visualize=False):
        """
        Estimate camera pose relative to tag 21 from an image.
        
        Args:
            image (np.array): Input image (BGR or grayscale)
            visualize (bool): If True, returns visualization image
            
        Returns:
            np.array: Mean 4x4 transformation matrix from camera to tag 21
            Returns None if no tags detected.
        """
        return get_mean_transform_to_tag21(
            image,
            tag_size=self.tag_size,
            tag_spacing_ratio=(self.tag_dist - self.tag_size) / self.tag_size,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            visualize=visualize
        )


if __name__ == "__main__":
    # Example usage
    print("AprilTag Grid Pose Estimator Module")
    print("=" * 50)
    print()
    print("Import this module to use in your own scripts:")
    print()
    print("from apriltag_grid_pose import get_mean_transform_to_tag21")
    print()
    print("# Quick usage:")
    print("mean_T_C_21 = get_mean_transform_to_tag21(image)")
    print("if mean_T_C_21 is not None:")
    print("    print(mean_T_C_21)")
    print()
    print("# Or use the class for repeated calls:")
    print("estimator = AprilGridPoseEstimator()")
    print("mean_T_C_21 = estimator.estimate_pose(image)")

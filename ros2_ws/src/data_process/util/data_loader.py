"""
Data Loader for Robot Demonstration Data

This module loads image sequences and trajectory data from demonstration folders.
It combines images with corresponding trajectory information (position, orientation, gripper width).

Usage:
    python data_loader.py <image_folder> <trajectory_json_path>
    
Example:
    python data_loader.py data/demo_data1/images data/demo_data1/trajectory.json
"""

import cv2
import os
import json
import argparse
import numpy as np
from typing import List, Dict, Optional
from scipy import signal


class TrajectoryFrame:
    """
    A single frame containing image and trajectory data.
    
    Attributes:
        id: Frame identifier
        timestamp: Time in seconds
        image: Image array (numpy array from cv2)
        pos: Position as numpy array [x, y, z]
        quat: Quaternion as numpy array [x, y, z, w]
        gripper_width: Gripper opening width
        gripper_width_filtered: Low-pass filtered gripper width
        gripper_state: Gripper state (1=closed, 0=open)
        velocity: Velocity as numpy array [vx, vy, vz] (computed from position)
        v_abs: Absolute velocity (speed, scalar value)
        v_abs_filtered: Low-pass filtered absolute velocity
    """
    
    def __init__(self, frame_id: int, timestamp: float, image: np.ndarray, 
                 pos: np.ndarray, quat: np.ndarray, gripper_width: float,
                 velocity: Optional[np.ndarray] = None):
        self.id = frame_id
        self.timestamp = timestamp
        self.image = image
        self.pos = pos
        self.quat = quat
        self.gripper_width = gripper_width
        self.gripper_width_filtered = gripper_width  # Will be computed later
        self.gripper_state = 0  # Will be computed later (0=closed, 1=open)
        self.velocity = velocity if velocity is not None else np.zeros(3)
        self.v_abs = np.linalg.norm(self.velocity)
        self.v_abs_filtered = 0.0  # Will be computed later
    
    def to_dict(self) -> Dict:
        """Convert to dictionary (excluding image for JSON serialization)."""
        return {
            'id': self.id,
            'timestamp': self.timestamp,
            'pos': self.pos.tolist(),
            'quat': self.quat.tolist(),
            'gripper_width': self.gripper_width,
            'gripper_width_filtered': float(self.gripper_width_filtered),
            'gripper_state': int(self.gripper_state),
            'velocity': self.velocity.tolist(),
            'v_abs': float(self.v_abs),
            'v_abs_filtered': float(self.v_abs_filtered)
        }
    
    def __repr__(self):
        return (f"TrajectoryFrame(id={self.id}, timestamp={self.timestamp:.3f}, "
                f"pos={self.pos}, quat={self.quat}, gripper_width={self.gripper_width:.4f}, "
                f"gripper_state={self.gripper_state}, "
                f"velocity={self.velocity}, v_abs={self.v_abs:.4f}, "
                f"v_abs_filtered={self.v_abs_filtered:.4f})")


class DemonstrationDataLoader:
    """
    Loader for robot demonstration data combining images and trajectory.
    """
    
    def __init__(self, image_folder: str, trajectory_json_path: str, 
                 filename_format: str = "frame_{id}.png"):
        """
        Initialize the data loader.
        
        Args:
            image_folder: Path to folder containing image files
            trajectory_json_path: Path to trajectory.json file
            filename_format: Format string for image filenames (default: "frame_{id}.png")
        """
        self.image_folder = image_folder
        self.trajectory_json_path = trajectory_json_path
        self.filename_format = filename_format
        self.frames: List[TrajectoryFrame] = []
        
        # Validate paths
        if not os.path.isdir(image_folder):
            raise FileNotFoundError(f"Image folder not found: {image_folder}")
        if not os.path.exists(trajectory_json_path):
            raise FileNotFoundError(f"Trajectory file not found: {trajectory_json_path}")
    
    def load_trajectory_json(self) -> List[Dict]:
        """Load trajectory data from JSON file."""
        print(f"Loading trajectory data from {self.trajectory_json_path}...")
        with open(self.trajectory_json_path, 'r') as f:
            trajectory_data = json.load(f)
        print(f"Loaded {len(trajectory_data)} trajectory entries.")
        return trajectory_data
    
    def load_image(self, frame_id: int) -> Optional[np.ndarray]:
        """
        Load a single image by frame ID.
        
        Args:
            frame_id: The frame identifier
            
        Returns:
            Image as numpy array, or None if image cannot be loaded
        """
        image_path = os.path.join(self.image_folder, self.filename_format.format(id=frame_id))
        
        if not os.path.exists(image_path):
            print(f"Warning: Image file not found: {image_path}")
            return None
        
        image = cv2.imread(image_path)
        if image is None:
            print(f"Warning: Could not read image file: {image_path}")
            return None
        
        return image
    
    def load_all_data(self) -> List[TrajectoryFrame]:
        """
        Load all trajectory data with corresponding images.
        
        Returns:
            List of TrajectoryFrame objects
        """
        # Load trajectory JSON
        trajectory_data = self.load_trajectory_json()
        
        print(f"Loading images from {self.image_folder}...")
        self.frames = []
        failed_count = 0
        
        for entry in trajectory_data:
            frame_id = entry['id']
            
            # Load image
            image = self.load_image(frame_id)
            if image is None:
                failed_count += 1
                continue
            
            # Extract trajectory data
            pos = np.array([entry['pos_x'], entry['pos_y'], entry['pos_z']])
            quat = np.array([entry['quat_x'], entry['quat_y'], entry['quat_z'], entry['quat_w']])
            gripper_width = entry['gripper_width']
            timestamp = entry['timestamp']
            
            # Create TrajectoryFrame object
            frame = TrajectoryFrame(
                frame_id=frame_id,
                timestamp=timestamp,
                image=image,
                pos=pos,
                quat=quat,
                gripper_width=gripper_width
            )
            
            self.frames.append(frame)
        
        if failed_count > 0:
            print(f"Warning: Failed to load {failed_count} images.")
        
        # Compute velocities based on position
        self._compute_velocities()
        
        # Apply low-pass filter to velocity
        self._apply_velocity_filter()
        
        # Apply low-pass filter to gripper_width and compute gripper_state
        self._apply_gripper_width_filter()
        
        print(f"Successfully loaded {len(self.frames)} frames with images and trajectory data.")
        return self.frames
    
    def _compute_velocities(self):
        """
        Compute velocities for all frames based on position differences.
        Uses forward difference for all frames except the last one (backward difference).
        Velocity is calculated as: v = (pos_next - pos_current) / (time_next - time_current)
        """
        if len(self.frames) < 2:
            print("Warning: Not enough frames to compute velocities.")
            return
        
        for i in range(len(self.frames) - 1):
            current_frame = self.frames[i]
            next_frame = self.frames[i + 1]
            
            dt = next_frame.timestamp - current_frame.timestamp
            if dt > 0:
                pos_diff = next_frame.pos - current_frame.pos
                current_frame.velocity = pos_diff / dt
                current_frame.v_abs = np.linalg.norm(current_frame.velocity)
            else:
                current_frame.velocity = np.zeros(3)
                current_frame.v_abs = 0.0
        
        # For the last frame, use backward difference
        if len(self.frames) >= 2:
            last_frame = self.frames[-1]
            prev_frame = self.frames[-2]
            
            dt = last_frame.timestamp - prev_frame.timestamp
            if dt > 0:
                pos_diff = last_frame.pos - prev_frame.pos
                last_frame.velocity = pos_diff / dt
                last_frame.v_abs = np.linalg.norm(last_frame.velocity)
            else:
                last_frame.velocity = np.zeros(3)
                last_frame.v_abs = 0.0
        
        print(f"Computed velocities for {len(self.frames)} frames.")
    
    def _apply_velocity_filter(self, cutoff_freq: float = 1.0, order: int = 4):
        """
        Apply low-pass Butterworth filter to velocity absolute values.
        
        Args:
            cutoff_freq: Cutoff frequency in Hz (default: 2.0 Hz)
            order: Filter order (default: 4)
        """
        if len(self.frames) < 10:
            print("Warning: Not enough frames for filtering. Copying v_abs to v_abs_filtered.")
            for frame in self.frames:
                frame.v_abs_filtered = frame.v_abs
            return
        
        # Extract v_abs values and compute sampling frequency
        v_abs_array = np.array([frame.v_abs for frame in self.frames])
        timestamps = np.array([frame.timestamp for frame in self.frames])
        
        # Compute average sampling frequency
        dt_mean = np.mean(np.diff(timestamps))
        if dt_mean <= 0:
            print("Warning: Invalid timestamps. Copying v_abs to v_abs_filtered.")
            for frame in self.frames:
                frame.v_abs_filtered = frame.v_abs
            return
        
        sampling_freq = 1.0 / dt_mean
        
        # Design Butterworth low-pass filter
        nyquist_freq = sampling_freq / 2.0
        normalized_cutoff = cutoff_freq / nyquist_freq
        
        # Ensure cutoff frequency is valid
        if normalized_cutoff >= 1.0:
            print(f"Warning: Cutoff frequency {cutoff_freq} Hz is too high for sampling rate {sampling_freq:.2f} Hz.")
            print("Copying v_abs to v_abs_filtered without filtering.")
            for frame in self.frames:
                frame.v_abs_filtered = frame.v_abs
            return
        
        # Create filter
        b, a = signal.butter(order, normalized_cutoff, btype='low', analog=False)
        
        # Apply filter (using filtfilt for zero-phase filtering)
        v_abs_filtered = signal.filtfilt(b, a, v_abs_array)
        
        # Assign filtered values back to frames
        for i, frame in enumerate(self.frames):
            frame.v_abs_filtered = float(v_abs_filtered[i])
        
        print(f"Applied low-pass filter (cutoff={cutoff_freq} Hz, order={order}, sampling={sampling_freq:.2f} Hz)")
    
    def _apply_gripper_width_filter(self, cutoff_freq: float = 2.0, order: int = 4):
        """
        Apply low-pass Butterworth filter to gripper_width values and compute gripper_state.
        
        Args:
            cutoff_freq: Cutoff frequency in Hz (default: 2.0 Hz)
            order: Filter order (default: 4)
        """
        if len(self.frames) < 10:
            print("Warning: Not enough frames for gripper width filtering. Using raw values.")
            for frame in self.frames:
                frame.gripper_width_filtered = frame.gripper_width
                frame.gripper_state = 0
            return
        
        # Extract gripper_width values and compute sampling frequency
        gripper_width_array = np.array([frame.gripper_width for frame in self.frames])
        timestamps = np.array([frame.timestamp for frame in self.frames])
        
        # Compute average sampling frequency
        dt_mean = np.mean(np.diff(timestamps))
        if dt_mean <= 0:
            print("Warning: Invalid timestamps. Using raw gripper width values.")
            for frame in self.frames:
                frame.gripper_width_filtered = frame.gripper_width
                frame.gripper_state = 0
            return
        
        sampling_freq = 1.0 / dt_mean
        
        # Design Butterworth low-pass filter
        nyquist_freq = sampling_freq / 2.0
        normalized_cutoff = cutoff_freq / nyquist_freq
        
        # Ensure cutoff frequency is valid
        if normalized_cutoff >= 1.0:
            print(f"Warning: Cutoff frequency {cutoff_freq} Hz is too high for sampling rate {sampling_freq:.2f} Hz.")
            print("Using raw gripper width values without filtering.")
            gripper_width_filtered = gripper_width_array.copy()
        else:
            # Create filter
            b, a = signal.butter(order, normalized_cutoff, btype='low', analog=False)
            
            # Apply filter (using filtfilt for zero-phase filtering)
            gripper_width_filtered = signal.filtfilt(b, a, gripper_width_array)
            
            print(f"Applied gripper width filter (cutoff={cutoff_freq} Hz, order={order}, sampling={sampling_freq:.2f} Hz)")
        
        # Compute threshold based on min and max of filtered gripper width
        min_gripper = np.min(gripper_width_filtered)
        max_gripper = np.max(gripper_width_filtered)
        threshold = (min_gripper + max_gripper) / 2.0  # Midpoint threshold
        
        print(f"Gripper width range: [{min_gripper:.4f}, {max_gripper:.4f}], threshold: {threshold:.4f}")
        
        # Assign filtered values and compute gripper state
        for i, frame in enumerate(self.frames):
            frame.gripper_width_filtered = float(gripper_width_filtered[i])
            # gripper_state: 1 = closed (below threshold), 0 = open (above threshold)
            frame.gripper_state = 1 if gripper_width_filtered[i] <= threshold else 0
        
        print(f"Computed gripper states (1=closed, 0=open)")
    
    def get_frame_by_id(self, frame_id: int) -> Optional[TrajectoryFrame]:
        """Get a specific frame by ID."""
        for frame in self.frames:
            if frame.id == frame_id:
                return frame
        return None
    
    def get_frame_by_timestamp(self, timestamp: float, tolerance: float = 0.01) -> Optional[TrajectoryFrame]:
        """
        Get frame closest to a specific timestamp.
        
        Args:
            timestamp: Target timestamp in seconds
            tolerance: Maximum time difference allowed
            
        Returns:
            Closest TrajectoryFrame or None if no frame within tolerance
        """
        closest_frame = None
        min_diff = float('inf')
        
        for frame in self.frames:
            diff = abs(frame.timestamp - timestamp)
            if diff < min_diff:
                min_diff = diff
                closest_frame = frame
        
        if min_diff <= tolerance or tolerance < 0:
            return closest_frame
        return None
    
    def get_frames_in_time_range(self, start_time: float, end_time: float) -> List[TrajectoryFrame]:
        """Get all frames within a time range."""
        return [frame for frame in self.frames 
                if start_time <= frame.timestamp <= end_time]
    
    def save_summary(self, output_path: str):
        """Save a summary of loaded data (without images) to JSON."""
        summary = {
            'total_frames': len(self.frames),
            'duration': self.frames[-1].timestamp if self.frames else 0,
            'frames': [frame.to_dict() for frame in self.frames]
        }
        
        with open(output_path, 'w') as f:
            json.dump(summary, f, indent=2)
        
        print(f"Summary saved to {output_path}")


def main():
    """Main function for command-line usage."""
    parser = argparse.ArgumentParser(description="Load robot demonstration data (images + trajectory)")
    parser.add_argument("image_folder", help="Path to folder containing images")
    parser.add_argument("trajectory_json", help="Path to trajectory.json file")
    parser.add_argument("--filename_format", default="frame_{id}.png", 
                       help="Image filename format (default: frame_{id}.png)")
    parser.add_argument("--output", help="Optional: Save summary to JSON file")
    parser.add_argument("--show_sample", action="store_true", 
                       help="Display a sample frame")
    
    args = parser.parse_args()
    
    # Create loader
    loader = DemonstrationDataLoader(
        image_folder=args.image_folder,
        trajectory_json_path=args.trajectory_json,
        filename_format=args.filename_format
    )
    
    # Load all data
    frames = loader.load_all_data()
    
    if not frames:
        print("No frames loaded. Exiting.")
        return
    
    # Print statistics
    print("\n=== Data Summary ===")
    print(f"Total frames: {len(frames)}")
    print(f"Duration: {frames[-1].timestamp:.2f} seconds")
    print(f"First frame: {frames[0]}")
    print(f"Last frame: {frames[-1]}")
    
    # Show sample frame info
    if len(frames) > 0:
        mid_frame = frames[len(frames) // 2]
        print(f"\nMiddle frame example:")
        print(f"  ID: {mid_frame.id}")
        print(f"  Timestamp: {mid_frame.timestamp:.3f}s")
        print(f"  Position: {mid_frame.pos}")
        print(f"  Quaternion: {mid_frame.quat}")
        print(f"  Gripper width: {mid_frame.gripper_width:.4f}")
        print(f"  Velocity: {mid_frame.velocity}")
        print(f"  v_abs (Speed): {mid_frame.v_abs:.4f}")
        print(f"  Image shape: {mid_frame.image.shape}")
    
    # Save summary if requested
    if args.output:
        loader.save_summary(args.output)
    
    # Display sample image if requested
    if args.show_sample and len(frames) > 0:
        sample_frame = frames[len(frames) // 2]
        cv2.imshow(f"Sample Frame (ID: {sample_frame.id}, Time: {sample_frame.timestamp:.2f}s)", 
                   sample_frame.image)
        print("\nPress any key to close the image window...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

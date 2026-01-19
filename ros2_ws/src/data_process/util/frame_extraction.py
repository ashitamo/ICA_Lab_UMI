#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
frame_extraction.py

Extract keyframes (local minima of velocity) and perform adaptive sampling
for trajectory data. This module provides functions for intelligent frame
extraction based on velocity analysis.
"""

import os
import cv2
import numpy as np
from scipy import signal
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt

from util.data_loader import DemonstrationDataLoader
from util.adative_sampling import sample_times_rate_proportional


def _sample_segment_by_velocity(
    frames,
    start_idx: int,
    end_idx: int,
    v_abs: np.ndarray,
    timestamps: np.ndarray,
    base_samples: float = 0.5,
    max_samples: int = 10,
    velocity_scale: float = 20
) -> Tuple[List, List[int], List[bool]]:
    """
    Sample frames in a segment based on velocity (higher velocity = more samples).
    
    Args:
        frames: All frames from the data loader
        start_idx: Start index of segment (inclusive)
        end_idx: End index of segment (exclusive)
        v_abs: Array of all filtered velocity values
        timestamps: Array of all timestamps
        base_samples: Minimum sampling rate (Hz)
        max_samples: Maximum sampling rate (Hz)
        velocity_scale: Scaling factor for velocity-based sampling
    
    Returns:
        Tuple of (sampled_frames, sampled_indices, is_keyframe_flags)
    """
    if end_idx - start_idx <= 1:
        return [], [], []
    
    segment_velocities = v_abs[start_idx:end_idx]
    segment_timestamps = timestamps[start_idx:end_idx]
    
    T, sampled_idx = sample_times_rate_proportional(
        segment_timestamps, 
        segment_velocities, 
        r_min=base_samples, 
        k=velocity_scale, 
        r_max=max_samples, 
        include_t1=False
    )
    
    # Remove head and tail from the slice
    sampled_idx = sampled_idx[1:-1]
    
    # Adjust indices to global frame list
    sampled_idx = sampled_idx + start_idx

    sampled_frames = [frames[i] for i in sampled_idx]
    is_keyframe_flags = [False] * len(sampled_idx)
    
    return sampled_frames, list(sampled_idx), is_keyframe_flags


def extract_keyframes_with_adaptive_sampling(
    loader: DemonstrationDataLoader,
    base_samples: float = 0.5,
    max_samples: int = 5,
    velocity_scale: float = 25,
    local_minima_order: int = 5,
    save_images: bool = False,
    output_dir: Optional[str] = None,
    save_plot: bool = True,
    plot_path: Optional[str] = None
) -> Tuple[List, List[int], List[bool], List]:
    """
    Extract keyframes (local minima) and perform adaptive sampling with gripper state.
    
    This function identifies keyframes as local minima in the velocity profile and
    performs adaptive sampling between keyframes based on velocity magnitude.
    
    Args:
        loader: DemonstrationDataLoader instance with loaded data
        base_samples: Minimum sampling rate in Hz (default: 0.5)
        max_samples: Maximum sampling rate in Hz (default: 5)
        velocity_scale: Scaling factor for velocity-based sampling (default: 25)
        local_minima_order: Order parameter for local minima detection (default: 5)
        save_images: Whether to save annotated images (default: False)
        output_dir: Directory to save images (default: "output/keyframe_adaptive_frames")
        save_plot: Whether to save visualization plot (default: True)
        plot_path: Path to save the plot (default: "output/keyframe_adaptive_sampling.png")
    
    Returns:
        Tuple of (sampled_frames, sampled_indices, is_keyframe_flags, sampled_frames_annotated)
        - sampled_frames: List of selected frames (original Frame objects)
        - sampled_indices: Indices of selected frames in the original sequence
        - is_keyframe_flags: Boolean list indicating which frames are keyframes
        - sampled_frames_annotated: List of annotated images (numpy arrays) with labels and gripper state
    """
    print("\n=== Keyframe Detection & Adaptive Sampling ===\n")
    
    # Extract velocity and timestamp data
    v_abs_filtered_values = np.array([frame.v_abs_filtered for frame in loader.frames])
    timestamps = np.array([frame.timestamp for frame in loader.frames], dtype=float)
    
    # Find local minima (keyframes)
    local_minima_indices = signal.argrelextrema(
        v_abs_filtered_values, 
        np.less, 
        order=local_minima_order
    )[0]
    
    print(f"Found {len(local_minima_indices)} keyframes (local minima)")
    
    # Initialize output lists
    sampled_frames = []
    sampled_indices = []
    is_keyframe_flags = []
    
    # Sample between keyframes
    if len(local_minima_indices) > 0:
        start_idx = 0
        
        # Process each segment between keyframes
        for i in range(len(local_minima_indices)):
            end_idx = local_minima_indices[i]
            
            # Sample frames in this segment based on velocity
            segment_frames, segment_indices, segment_flags = _sample_segment_by_velocity(
                frames=loader.frames,
                start_idx=start_idx,
                end_idx=end_idx,
                v_abs=v_abs_filtered_values,
                timestamps=timestamps,
                base_samples=base_samples,
                max_samples=max_samples,
                velocity_scale=velocity_scale,
            )
            
            sampled_frames.extend(segment_frames)
            sampled_indices.extend(segment_indices)
            is_keyframe_flags.extend(segment_flags)
            
            # Add the keyframe (marked as special)
            sampled_frames.append(loader.frames[end_idx])
            sampled_indices.append(end_idx)
            is_keyframe_flags.append(True)
            
            start_idx = end_idx + 1
        
        # Add last segment (from last keyframe to end)
        if start_idx < len(loader.frames):
            segment_frames, segment_indices, segment_flags = _sample_segment_by_velocity(
                frames=loader.frames,
                start_idx=start_idx,
                end_idx=len(loader.frames) - 1,
                v_abs=v_abs_filtered_values,
                timestamps=timestamps,
                base_samples=base_samples,
                max_samples=max_samples,
                velocity_scale=velocity_scale,
            )
            sampled_frames.extend(segment_frames)
            sampled_indices.extend(segment_indices)
            is_keyframe_flags.extend(segment_flags)
    else:
        # No keyframes found, sample the entire sequence
        segment_frames, segment_indices, segment_flags = _sample_segment_by_velocity(
            frames=loader.frames,
            start_idx=0,
            end_idx=len(loader.frames) - 1,
            v_abs=v_abs_filtered_values,
            timestamps=timestamps,
            base_samples=base_samples,
            max_samples=max_samples,
            velocity_scale=velocity_scale,
        )
        sampled_frames.extend(segment_frames)
        sampled_indices.extend(segment_indices)
        is_keyframe_flags.extend(segment_flags)
    
    print(f"\nTotal sampled frames: {len(sampled_frames)}")
    print(f"  - Keyframes: {sum(is_keyframe_flags)}")
    print(f"  - Adaptive sampled frames: {len(sampled_frames) - sum(is_keyframe_flags)}")
    
    # Create annotated frames for all sampled frames
    print(f"\nCreating annotated frames...")
    sampled_frames_annotated = []
    from util.sam2_client import add_segmentation_contours
    from PIL import Image
    
    for i, (frame, is_keyframe) in enumerate(zip(sampled_frames, is_keyframe_flags)):
        # Determine gripper state
        gripper_state_text = "Close" if frame.gripper_state > 0.5 else "Open"
        
        # Convert numpy array to PIL Image
        frame_image_pil = Image.fromarray(cv2.cvtColor(frame.image, cv2.COLOR_BGR2RGB))
        
        # Add segmentation contours
        # img_with_contours = add_segmentation_contours(
        #     frame_image_pil,
        #     points_per_side=32,
        #     max_masks=30,
        #     line_width=1,
        #     min_area=100,
        #     use_fixed_color=True,
        #     fixed_color=(255, 255, 255)
        # )
        img_with_contours =frame_image_pil
        
        # Convert back to numpy array for OpenCV processing
        img_annotated = cv2.cvtColor(np.array(img_with_contours), cv2.COLOR_RGB2BGR)
        
        # Different colors for keyframes vs regular frames
        if is_keyframe:
            color = (0, 0, 255)  # Red for keyframes
            label = "KEYFRAME"
        else:
            color = (0, 255, 255)  # Yellow for regular samples
            label = "Sample"
        
        # Add text overlay
        text_lines = [
            f"{label}",
            f"Frame ID: {frame.id}",
            f"Time: {frame.timestamp:.2f}s",
            f"Gripper: {gripper_state_text}"
        ]
        
        y_offset = 30
        for line in text_lines:
            cv2.putText(img_annotated, line, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
            y_offset += 30
        
        sampled_frames_annotated.append(img_annotated)
    
    print(f"Created {len(sampled_frames_annotated)} annotated frames")
    
    # Save images if requested
    if save_images:
        if output_dir is None:
            output_dir = "output/keyframe_adaptive_frames"
        os.makedirs(output_dir, exist_ok=True)
        
        print(f"\nSaving images to {output_dir}/...")
        for i, (img_annotated, is_keyframe, frame) in enumerate(zip(sampled_frames_annotated, is_keyframe_flags, sampled_frames)):
            # Create filename
            frame_type = "KEYFRAME" if is_keyframe else "sample"
            filename = f"{i+1:03d}_{frame_type}_frame{frame.id:04d}_t{frame.timestamp:.2f}s.png"
            filepath = os.path.join(output_dir, filename)
            
            # Save image
            cv2.imwrite(filepath, img_annotated)
        
        print(f"Saved {len(sampled_frames_annotated)} images successfully!")
    
    # Create visualization plot if requested
    if save_plot:
        if plot_path is None:
            plot_path = "output/keyframe_adaptive_sampling.png"
        
        _create_visualization_plot(
            loader, 
            sampled_frames, 
            sampled_indices,
            is_keyframe_flags,
            plot_path
        )
    
    return sampled_frames, sampled_indices, is_keyframe_flags, sampled_frames_annotated


def _create_visualization_plot(
    loader: DemonstrationDataLoader,
    sampled_frames: List,
    sampled_indices: List[int],
    is_keyframe_flags: List[bool],
    plot_path: str
):
    """
    Create and save a visualization plot showing velocity, sampled points, and gripper state.
    
    Args:
        loader: DemonstrationDataLoader instance
        sampled_frames: List of sampled frames
        sampled_indices: Indices of sampled frames
        is_keyframe_flags: Boolean flags for keyframes
        plot_path: Path to save the plot
    """
    # Prepare data for visualization
    timestamps_all = np.array([frame.timestamp for frame in loader.frames])
    v_abs_filtered_all = np.array([frame.v_abs_filtered for frame in loader.frames])
    gripper_states_all = np.array([frame.gripper_state for frame in loader.frames])
    
    sampled_times = np.array([frame.timestamp for frame in sampled_frames])
    sampled_velocities = np.array([frame.v_abs_filtered for frame in sampled_frames])
    sampled_gripper_states = np.array([frame.gripper_state for frame in sampled_frames])
    
    # Separate keyframes and regular samples for plotting
    is_keyframe_array = np.array(is_keyframe_flags)
    keyframe_times = sampled_times[is_keyframe_array]
    keyframe_velocities = sampled_velocities[is_keyframe_array]
    regular_times = sampled_times[~is_keyframe_array]
    regular_velocities = sampled_velocities[~is_keyframe_array]
    
    # Calculate FPS (frames per second) for sampled frames
    fps_values = []
    fps_times = []
    for i in range(len(sampled_times) - 1):
        dt = sampled_times[i + 1] - sampled_times[i]
        if dt > 0:
            fps = 1.0 / dt
            fps_values.append(fps)
            fps_times.append(sampled_times[i])
    
    # Create figure with two subplots stacked vertically
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
    
    # Subplot 1: Velocity with sampled points and gripper state
    # Plot velocity curve
    ax1.plot(timestamps_all, v_abs_filtered_all, linewidth=2, color='blue', 
             alpha=0.5, label='Velocity')
    
    # Plot gripper state in background
    ax1_twin = ax1.twinx()
    ax1_twin.fill_between(timestamps_all, gripper_states_all, alpha=0.2, 
                          color='green', label='Gripper State')
    ax1_twin.set_ylabel('Gripper State (0=Open, 1=Close)', fontsize=11, color='green')
    ax1_twin.set_ylim([-0.1, 1.1])
    ax1_twin.set_yticks([0, 1])
    ax1_twin.set_yticklabels(['Open', 'Close'])
    ax1_twin.tick_params(axis='y', labelcolor='green')
    
    # Plot sampled points
    ax1.scatter(regular_times, regular_velocities, color='yellow', s=80, zorder=5, 
                edgecolors='black', linewidths=1.5, 
                label=f'Adaptive Samples (n={len(regular_times)})')
    ax1.scatter(keyframe_times, keyframe_velocities, color='red', s=200, zorder=6, 
                marker='v', edgecolors='black', linewidths=2, 
                label=f'Keyframes (n={len(keyframe_times)})')
    
    ax1.set_ylabel('Velocity (m/s)', fontsize=12)
    ax1.set_title('Keyframe Detection & Adaptive Sampling with Gripper State', 
                  fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=10, loc='upper left')
    
    # Subplot 2: Sampling FPS
    if fps_values:
        ax2.plot(fps_times, fps_values, linewidth=2, color='green', marker='o', 
                 markersize=6, label='Sampling FPS', markeredgecolor='black', 
                 markeredgewidth=0.5)
        ax2.fill_between(fps_times, fps_values, alpha=0.3, color='green')
        
        # Mark all sampled points on the FPS plot
        ax2.scatter(sampled_times, [0] * len(sampled_times), color='blue', s=20, 
                   alpha=0.5, marker='|', linewidths=2, 
                   label=f'Sample Points (n={len(sampled_times)})')
    
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('FPS (frames/second)', fontsize=12)
    ax2.set_title('Sampling Rate (FPS) over Time', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=10, loc='best')
    
    # Add statistics text to FPS plot
    if fps_values:
        fps_stats = (f'Mean FPS: {np.mean(fps_values):.2f}\n'
                    f'Max FPS: {np.max(fps_values):.2f}\n'
                    f'Min FPS: {np.min(fps_values):.2f}\n'
                    f'Total Samples: {len(sampled_times)}\n'
                    f'Keyframes: {sum(is_keyframe_flags)}')
        ax2.text(0.02, 0.98, fps_stats, transform=ax2.transAxes,
                fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))
    
    plt.tight_layout()
    
    # Save plot
    os.makedirs(os.path.dirname(plot_path), exist_ok=True)
    plt.savefig(plot_path, dpi=150)
    print(f"\nVisualization plot saved to: {plot_path}")
    plt.close()


# Convenience function for easy import
def extract_keyframes(
    loader: DemonstrationDataLoader,
    **kwargs
) -> Tuple[List, List[int], List[bool]]:
    """
    Convenience wrapper for extract_keyframes_with_adaptive_sampling.
    
    Args:
        loader: DemonstrationDataLoader instance with loaded data
        **kwargs: Additional keyword arguments passed to extract_keyframes_with_adaptive_sampling
    
    Returns:
        Tuple of (sampled_frames, sampled_indices, is_keyframe_flags)
    """
    return extract_keyframes_with_adaptive_sampling(loader, **kwargs)

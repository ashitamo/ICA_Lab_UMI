"""
Example usage of data_loader.py

This script demonstrates how to use the DemonstrationDataLoader class
to load and work with robot demonstration data.
"""

import sys
import os
# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from util.data_loader import DemonstrationDataLoader
from util.viz_tool import images_to_gif, view_gif, save_gif, add_timestamp_overlay, add_velocity_overlay
from util.adative_sampling import sample_times_rate_proportional, sample_times_rate_proportional_opt
import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal


def example_basic_usage():
    """Basic usage: load all data and access frames."""
    print("=== Example 1: Basic Usage ===\n")
    
    # Create loader
    loader = DemonstrationDataLoader(
        image_folder="data/demo_data1/images",
        trajectory_json_path="data/demo_data1/trajectory.json",
        filename_format="frame_{id}.png"
    )
    
    # Load all data
    frames = loader.load_all_data()
    
    print(f"Loaded {len(frames)} frames")
    print(f"Duration: {frames[-1].timestamp:.2f} seconds")
    
    # Access first frame
    first_frame = frames[0]
    print(f"\nFirst frame:")
    print(f"  ID: {first_frame.id}")
    print(f"  Timestamp: {first_frame.timestamp}")
    print(f"  Position: {first_frame.pos}")
    print(f"  Quaternion: {first_frame.quat}")
    print(f"  Gripper width: {first_frame.gripper_width}")
    print(f"  Image shape: {first_frame.image.shape}")
    
    return loader


def example_access_by_id(loader):
    """Example: Access frame by ID."""
    print("\n\n=== Example 2: Access by ID ===\n")
    
    frame_id = 100
    frame = loader.get_frame_by_id(frame_id)
    
    if frame:
        print(f"Frame {frame_id}:")
        print(f"  Timestamp: {frame.timestamp:.3f}s")
        print(f"  Position: {frame.pos}")
        print(f"  Gripper width: {frame.gripper_width:.4f}")
    else:
        print(f"Frame {frame_id} not found")


def example_access_by_timestamp(loader):
    """Example: Access frame by timestamp."""
    print("\n\n=== Example 3: Access by Timestamp ===\n")
    
    target_time = 5.0  # 5 seconds
    frame = loader.get_frame_by_timestamp(target_time, tolerance=0.1)
    
    if frame:
        print(f"Frame closest to {target_time}s:")
        print(f"  ID: {frame.id}")
        print(f"  Actual timestamp: {frame.timestamp:.3f}s")
        print(f"  Position: {frame.pos}")
    else:
        print(f"No frame found near {target_time}s")


def example_time_range(loader):
    """Example: Get frames in a time range."""
    print("\n\n=== Example 4: Frames in Time Range ===\n")
    
    start_time = 3.0
    end_time = 5.0
    
    frames_in_range = loader.get_frames_in_time_range(start_time, end_time)
    
    print(f"Frames between {start_time}s and {end_time}s:")
    print(f"  Total frames: {len(frames_in_range)}")
    if frames_in_range:
        print(f"  First frame ID: {frames_in_range[0].id} at {frames_in_range[0].timestamp:.3f}s")
        print(f"  Last frame ID: {frames_in_range[-1].id} at {frames_in_range[-1].timestamp:.3f}s")


def example_iterate_frames(loader):
    """Example: Iterate through all frames."""
    print("\n\n=== Example 5: Iterate Through Frames ===\n")
    
    print("Iterating through first 5 frames:")
    for i, frame in enumerate(loader.frames[:5]):
        print(f"Frame {i}: ID={frame.id}, t={frame.timestamp:.3f}s, "
              f"pos={frame.pos}, gripper={frame.gripper_width:.4f}")


def example_work_with_images(loader):
    """Example: Work with images."""
    print("\n\n=== Example 6: Working with Images ===\n")
    
    # Get a sample frame
    frame = loader.frames[len(loader.frames) // 2]
    
    print(f"Working with frame {frame.id}:")
    print(f"  Image shape: {frame.image.shape}")
    print(f"  Image dtype: {frame.image.dtype}")
    
    # Example: Convert to grayscale
    gray_image = cv2.cvtColor(frame.image, cv2.COLOR_BGR2GRAY)
    print(f"  Grayscale shape: {gray_image.shape}")
    
    # Example: Resize image
    resized = cv2.resize(frame.image, (320, 240))
    print(f"  Resized shape: {resized.shape}")
    
    # You can display the image
    # cv2.imshow("Sample Frame", frame.image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


def example_data_as_arrays(loader):
    """Example: Extract data as numpy arrays."""
    print("\n\n=== Example 7: Extract as Numpy Arrays ===\n")
    
    # Extract all positions
    positions = np.array([frame.pos for frame in loader.frames])
    print(f"Positions array shape: {positions.shape}")
    print(f"Mean position: {np.mean(positions, axis=0)}")
    
    # Extract all gripper widths
    gripper_widths = np.array([frame.gripper_width for frame in loader.frames])
    print(f"Gripper widths array shape: {gripper_widths.shape}")
    print(f"Min gripper width: {np.min(gripper_widths):.4f}")
    print(f"Max gripper width: {np.max(gripper_widths):.4f}")
    
    # Extract all timestamps
    timestamps = np.array([frame.timestamp for frame in loader.frames])
    print(f"Timestamps array shape: {timestamps.shape}")
    print(f"Duration: {timestamps[-1] - timestamps[0]:.2f}s")


def example_create_and_view_gif(loader):
    """Example: Create GIF from images and display it."""
    print("\n\n=== Example 8: Create and View GIF ===\n")
    
    # Sample every 10th frame to reduce GIF size
    sample_rate = 5
    sampled_frames = loader.frames[::sample_rate]
    
    print(f"Sampling every {sample_rate}th frame")
    print(f"Total frames in GIF: {len(sampled_frames)}")
    
    # Extract images, timestamps, and velocity values (use filtered velocity)
    images = [frame.image for frame in sampled_frames]
    timestamps = [frame.timestamp for frame in sampled_frames]
    v_abs_values = [frame.v_abs_filtered for frame in sampled_frames]  # Use filtered velocity
    
    # Add timestamp overlay (left side)
    print("Adding timestamp overlays...")
    images_with_time = add_timestamp_overlay(images, timestamps)
    
    # Add velocity overlay (right side)
    print("Adding velocity overlays (filtered)...")
    images_with_overlays = add_velocity_overlay(images_with_time, v_abs_values)
    
    # Create GIF (as bytes variable)
    print("Creating GIF animation...")
    gif_data = images_to_gif(
        images_with_overlays, 
        duration=1000,  # 100ms per frame
        loop=0,  # Infinite loop
        resize=(640, 480)  # Resize to reduce file size
    )
    
    print(f"GIF created! Size: {len(gif_data)} bytes ({len(gif_data)/1024:.1f} KB)")
    
    # Optionally save the GIF
    save_path = "output/demo_animation.gif"
    import os
    os.makedirs("output", exist_ok=True)
    save_gif(images_with_overlays, save_path, duration=100, resize=(640, 480))
    
    # Display the GIF
    print("\nDisplaying GIF animation...")
    print("Press SPACE to pause/resume, 'q' or ESC to close")
    view_gif(gif_data, window_name="Demo Animation", fps=10)


def example_plot_velocity(loader):
    """Example: Plot time vs velocity absolute value (v_abs)."""
    print("\n\n=== Example 9: Plot Time vs Velocity ===\n")
    
    # Extract timestamps, velocity absolute values, and filtered values
    timestamps = np.array([frame.timestamp for frame in loader.frames])
    v_abs_values = np.array([frame.v_abs for frame in loader.frames])
    v_abs_filtered_values = np.array([frame.v_abs_filtered for frame in loader.frames])
    
    print(f"Total frames: {len(timestamps)}")
    print(f"Time range: {timestamps[0]:.3f}s to {timestamps[-1]:.3f}s")
    print(f"v_abs range: {np.min(v_abs_values):.4f} to {np.max(v_abs_values):.4f}")
    print(f"Mean v_abs: {np.mean(v_abs_values):.4f}")
    print(f"Std v_abs: {np.std(v_abs_values):.4f}")
    print(f"v_abs_filtered range: {np.min(v_abs_filtered_values):.4f} to {np.max(v_abs_filtered_values):.4f}")
    print(f"Mean v_abs_filtered: {np.mean(v_abs_filtered_values):.4f}")
    print(f"Std v_abs_filtered: {np.std(v_abs_filtered_values):.4f}")
    
    # Create the plot
    plt.figure(figsize=(12, 6))
    plt.plot(timestamps, v_abs_values, linewidth=1, color='blue', alpha=0.5, label='Velocity (v_abs, raw)')
    plt.plot(timestamps, v_abs_filtered_values, linewidth=2, color='red', label='Velocity (v_abs_filtered)')
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Velocity Absolute Value (m/s)', fontsize=12)
    plt.title('Robot End-Effector Velocity over Time', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=10)
    
    # Add statistics as text
    stats_text = (f'Raw - Mean: {np.mean(v_abs_values):.4f} m/s, Max: {np.max(v_abs_values):.4f} m/s\n'
                  f'Filtered - Mean: {np.mean(v_abs_filtered_values):.4f} m/s, Max: {np.max(v_abs_filtered_values):.4f} m/s')
    plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes,
             fontsize=9, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    
    # Save the plot
    output_path = "output/velocity_plot.png"
    os.makedirs("output", exist_ok=True)
    plt.savefig(output_path, dpi=150)
    print(f"\nPlot saved to: {output_path}")
    
    # Display the plot
    plt.show()
    print("Plot displayed. Close the window to continue...")


def example_plot_gripper_width(loader):
    """Example: Plot time vs gripper_width with filtering."""
    print("\n\n=== Example 9b: Plot Gripper Width (Raw and Filtered) ===\n")
    
    # Extract timestamps, gripper width values, and filtered values
    timestamps = np.array([frame.timestamp for frame in loader.frames])
    gripper_widths = np.array([frame.gripper_width for frame in loader.frames])
    gripper_width_filtered = np.array([frame.gripper_width_filtered for frame in loader.frames])
    gripper_states = np.array([frame.gripper_state for frame in loader.frames])
    
    print(f"Total frames: {len(timestamps)}")
    print(f"Time range: {timestamps[0]:.3f}s to {timestamps[-1]:.3f}s")
    print(f"Gripper width range: {np.min(gripper_widths):.4f} to {np.max(gripper_widths):.4f}")
    print(f"Mean gripper width: {np.mean(gripper_widths):.4f}")
    print(f"Std gripper width: {np.std(gripper_widths):.4f}")
    print(f"Filtered gripper width range: {np.min(gripper_width_filtered):.4f} to {np.max(gripper_width_filtered):.4f}")
    print(f"Mean filtered gripper width: {np.mean(gripper_width_filtered):.4f}")
    print(f"Std filtered gripper width: {np.std(gripper_width_filtered):.4f}")
    
    # Count gripper state transitions
    state_changes = np.sum(np.abs(np.diff(gripper_states)))
    print(f"Gripper state changes: {state_changes}")
    
    # Create the plot with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # Subplot 1: Gripper width
    ax1.plot(timestamps, gripper_widths, linewidth=1, color='gray', alpha=0.5, label='Gripper Width (raw)')
    ax1.plot(timestamps, gripper_width_filtered, linewidth=2, color='purple', label='Gripper Width (filtered)')
    ax1.set_ylabel('Gripper Width (m)', fontsize=12)
    ax1.set_title('Robot Gripper Width over Time', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=10)
    
    # Add statistics as text
    stats_text = (f'Raw - Mean: {np.mean(gripper_widths):.4f} m, Range: [{np.min(gripper_widths):.4f}, {np.max(gripper_widths):.4f}]\n'
                  f'Filtered - Mean: {np.mean(gripper_width_filtered):.4f} m, Range: [{np.min(gripper_width_filtered):.4f}, {np.max(gripper_width_filtered):.4f}]')
    ax1.text(0.02, 0.98, stats_text, transform=ax1.transAxes,
             fontsize=9, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='lavender', alpha=0.5))
    
    # Subplot 2: Gripper state
    ax2.fill_between(timestamps, gripper_states, alpha=0.4, color='green', label='Gripper State')
    ax2.plot(timestamps, gripper_states, linewidth=2, color='darkgreen')
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Gripper State', fontsize=12)
    ax2.set_yticks([0, 1])
    ax2.set_yticklabels(['Open (0)', 'Closed (1)'])
    ax2.set_title('Gripper State over Time', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3, axis='x')
    ax2.legend(fontsize=10)
    
    # Add state change count
    state_text = f'State changes: {state_changes}'
    ax2.text(0.02, 0.98, state_text, transform=ax2.transAxes,
             fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))
    
    plt.tight_layout()
    
    # Save the plot
    output_path = "output/gripper_width_plot.png"
    os.makedirs("output", exist_ok=True)
    plt.savefig(output_path, dpi=150)
    print(f"\nPlot saved to: {output_path}")
    
    # Display the plot
    plt.show()
    print("Plot displayed. Close the window to continue...")


def example_extract_local_minima(loader):
    """Example: Extract frames at local minima of v_abs_filtered and save images."""
    print("\n\n=== Example 10: Extract Local Minima of Velocity ===\n")
    
    # Extract v_abs_filtered values
    v_abs_filtered_values = np.array([frame.v_abs_filtered for frame in loader.frames])
    
    # Find local minima using scipy.signal.argrelextrema
    # distance parameter controls minimum distance between peaks
    local_minima_indices = signal.argrelextrema(v_abs_filtered_values, np.less, order=10)[0]
    
    print(f"Total frames: {len(loader.frames)}")
    print(f"Found {len(local_minima_indices)} local minima")
    
    # Extract frames at local minima
    local_minima_frames = [loader.frames[i] for i in local_minima_indices]
    
    # Print information about local minima
    print("\nLocal minima details:")
    for i, (idx, frame) in enumerate(zip(local_minima_indices, local_minima_frames)):
        print(f"  {i+1}. Frame ID={frame.id}, Index={idx}, "
              f"Time={frame.timestamp:.3f}s, v_abs_filtered={frame.v_abs_filtered:.4f}")
    
    # Create output directory
    output_dir = "output/local_minima_frames"
    os.makedirs(output_dir, exist_ok=True)
    
    # Save images
    print(f"\nSaving images to {output_dir}/...")
    for i, frame in enumerate(local_minima_frames):
        # Create filename with frame info
        filename = f"minima_{i+1:03d}_frame{frame.id:04d}_t{frame.timestamp:.2f}s_v{frame.v_abs_filtered:.4f}.png"
        filepath = os.path.join(output_dir, filename)
        
        # Add annotation to image
        img_annotated = frame.image.copy()
        
        # Add text overlay with frame information
        text_lines = [
            f"Frame ID: {frame.id}",
            f"Time: {frame.timestamp:.2f}s",
            f"v_abs_filtered: {frame.v_abs_filtered:.4f} m/s",
            f"Local Minimum #{i+1}"
        ]
        
        y_offset = 30
        for line in text_lines:
            cv2.putText(img_annotated, line, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
            y_offset += 30
        
        # Save image
        cv2.imwrite(filepath, img_annotated)
    
    print(f"Saved {len(local_minima_frames)} images successfully!")
    
    # Create a visualization plot showing local minima on velocity curve
    plt.figure(figsize=(14, 6))
    timestamps = np.array([frame.timestamp for frame in loader.frames])
    v_abs_filtered_all = np.array([frame.v_abs_filtered for frame in loader.frames])
    minima_times = np.array([frame.timestamp for frame in local_minima_frames])
    minima_values = np.array([frame.v_abs_filtered for frame in local_minima_frames])
    
    plt.plot(timestamps, v_abs_filtered_all, linewidth=2, color='blue', label='v_abs_filtered')
    plt.scatter(minima_times, minima_values, color='red', s=100, zorder=5, 
                label=f'Local Minima (n={len(local_minima_frames)})', marker='v')
    
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Velocity (m/s)', fontsize=12)
    plt.title('Velocity with Local Minima Marked', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=10)
    plt.tight_layout()
    
    plot_path = "output/velocity_local_minima_plot.png"
    plt.savefig(plot_path, dpi=150)
    print(f"\nVisualization plot saved to: {plot_path}")
    plt.show()
    
    return local_minima_frames


def example_velocity_based_sampling(loader, base_samples=0.5, max_samples=5, velocity_scale=25):
    """Example: Sample frames between local minima based on velocity (faster = more frames)."""
    print("\n\n=== Example 11: Velocity-Based Adaptive Sampling ===\n")
    
    # First, find local minima
    v_abs_filtered_values = np.array([frame.v_abs_filtered for frame in loader.frames])
    timestamps = np.array([frame.timestamp for frame in loader.frames], dtype=float)
    local_minima_indices = signal.argrelextrema(v_abs_filtered_values, np.less, order=5)[0]
    
    print(f"Found {len(local_minima_indices)} local minima")
    
    # Always include local minima frames
    sampled_frames = []
    sampled_indices = []
    is_minima_flags = []  # Track which frames are local minima
    
    # If we have minima, sample between them
    if len(local_minima_indices) > 0:
        # Add first segment (from start to first minimum)
        start_idx = 0
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
            is_minima_flags.extend(segment_flags)
            
            # Add the local minimum frame (marked as special)
            sampled_frames.append(loader.frames[end_idx])
            sampled_indices.append(end_idx)
            is_minima_flags.append(True)
            
            start_idx = end_idx + 1
        
        # Add last segment (from last minimum to end)
        if start_idx < len(loader.frames):
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
            is_minima_flags.extend(segment_flags)
    else:
        # No minima found, sample the entire sequence
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
    
    print(f"\nTotal sampled frames: {len(sampled_frames)}")
    print(f"  - Local minima frames: {sum(is_minima_flags)}")
    print(f"  - Velocity-based sampled frames: {len(sampled_frames) - sum(is_minima_flags)}")
    
    # Create output directory
    output_dir = "output/velocity_sampled_frames"
    os.makedirs(output_dir, exist_ok=True)
    
    # Save images with different markers for minima vs regular samples
    print(f"\nSaving images to {output_dir}/...")
    for i, (frame, is_minima) in enumerate(zip(sampled_frames, is_minima_flags)):
        # Create filename
        frame_type = "MINIMA" if is_minima else "sample"
        filename = f"{i+1:03d}_{frame_type}_frame{frame.id:04d}_t{frame.timestamp:.2f}s_v{frame.v_abs_filtered:.4f}.png"
        filepath = os.path.join(output_dir, filename)
        
        # Add annotation to image
        img_annotated = frame.image.copy()
        
        # Different colors for minima vs regular frames
        if is_minima:
            color = (0, 0, 255)  # Red for local minima
            label = "LOCAL MINIMUM"
        else:
            color = (0, 255, 255)  # Yellow for regular samples
            label = "Velocity Sample"
        
        # Add text overlay
        text_lines = [
            f"{label}",
            f"Frame ID: {frame.id}",
            f"Time: {frame.timestamp:.2f}s",
            f"v_abs_filtered: {frame.v_abs_filtered:.4f} m/s"
        ]
        
        y_offset = 30
        for line in text_lines:
            cv2.putText(img_annotated, line, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
            y_offset += 30
        
        # Save image
        cv2.imwrite(filepath, img_annotated)
    
    print(f"Saved {len(sampled_frames)} images successfully!")
    
    # Prepare data for visualization
    timestamps = np.array([frame.timestamp for frame in loader.frames])
    v_abs_filtered_all = np.array([frame.v_abs_filtered for frame in loader.frames])
    
    sampled_times = np.array([frame.timestamp for frame in sampled_frames])
    sampled_velocities = np.array([frame.v_abs_filtered for frame in sampled_frames])
    
    # Separate minima and regular samples for plotting
    minima_times = sampled_times[is_minima_flags]
    minima_velocities = sampled_velocities[is_minima_flags]
    regular_times = sampled_times[~np.array(is_minima_flags)]
    regular_velocities = sampled_velocities[~np.array(is_minima_flags)]
    
    # Calculate FPS (frames per second) for sampled frames
    # FPS is calculated as 1 / time_difference between consecutive samples
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
    
    # Subplot 1: Velocity with sampled points
    ax1.plot(timestamps, v_abs_filtered_all, linewidth=2, color='blue', alpha=0.5, label='v_abs_filtered')
    ax1.scatter(regular_times, regular_velocities, color='yellow', s=80, zorder=5, 
                edgecolors='black', linewidths=1.5, label=f'Velocity Samples (n={len(regular_times)})')
    ax1.scatter(minima_times, minima_velocities, color='red', s=200, zorder=6, 
                marker='v', edgecolors='black', linewidths=2, 
                label=f'Local Minima (n={len(minima_times)})')
    ax1.set_ylabel('Velocity (m/s)', fontsize=12)
    ax1.set_title('Velocity-Based Adaptive Sampling', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=10, loc='best')
    
    # Subplot 2: FPS (sampling rate)
    if fps_values:
        ax2.plot(fps_times, fps_values, linewidth=2, color='green', marker='o', 
                 markersize=6, label='Sampling FPS', markeredgecolor='black', markeredgewidth=0.5)
        ax2.fill_between(fps_times, fps_values, alpha=0.3, color='green')
        
        # Also mark all sampled points on the FPS plot for clarity
        ax2.scatter(sampled_times, [0] * len(sampled_times), color='blue', s=20, 
                   alpha=0.5, marker='|', linewidths=2, label=f'Sample Points (n={len(sampled_times)})')
    
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
                    f'Total Samples: {len(sampled_times)}')
        ax2.text(0.02, 0.98, fps_stats, transform=ax2.transAxes,
                fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))
    
    plt.tight_layout()
    
    plot_path = "output/velocity_adaptive_sampling_plot.png"
    plt.savefig(plot_path, dpi=150)
    print(f"\nVisualization plot saved to: {plot_path}")
    plt.show()
    
    return sampled_frames, is_minima_flags



def _sample_segment_by_velocity(frames, start_idx, end_idx, v_abs,timestamps, base_samples=0.5,  max_samples=10, velocity_scale=20):
    """
    Sample frames in a segment based on velocity (higher velocity = more samples).
    
    Args:
        frames: All frames
        start_idx: Start index of segment
        end_idx: End index of segment (exclusive)
        v_abs_filtered_values: Array of all filtered velocity values
        base_samples: Minimum number of samples per segment
        velocity_scale: Scaling factor for velocity-based sampling
    
    Returns:
        Tuple of (sampled_frames, sampled_indices, is_minima_flags)
    """
    if end_idx - start_idx <= 1:
        return [], [], []
    
    segment_velocities = v_abs[start_idx:end_idx]
    segment_timestamps = timestamps[start_idx:end_idx]
    
    T, sampled_idx = sample_times_rate_proportional(
        segment_timestamps, segment_velocities, r_min=base_samples, k=velocity_scale, r_max=max_samples, include_t1=False
    )
    # 去頭尾切片
    sampled_idx = sampled_idx[1:-1]
    
    sampled_idx = sampled_idx + start_idx  # Adjust indices to global frame list

    sampled_frames = [frames[i] for i in sampled_idx]
    is_minima_flags = [False] * len(sampled_idx)
    
    return sampled_frames, sampled_idx, is_minima_flags

def main():
    """Run all examples."""
    # Example 1: Basic usage
    loader = example_basic_usage()
    
    # # Example 2: Access by ID
    # example_access_by_id(loader)
    
    # # Example 3: Access by timestamp
    # example_access_by_timestamp(loader)
    
    # # Example 4: Time range
    # example_time_range(loader)
    
    # # Example 5: Iterate frames
    # example_iterate_frames(loader)
    
    # # Example 6: Work with images
    # example_work_with_images(loader)
    
    # # Example 7: Extract as arrays
    # example_data_as_arrays(loader)
    
    # # Example 8: Create and view GIF
    # example_create_and_view_gif(loader) 
    
    # Example 9: Plot velocity
    example_plot_velocity(loader)
    
    # Example 9b: Plot gripper width
    example_plot_gripper_width(loader)
    
    # Example 10: Extract local minima frames
    # local_minima_frames = example_extract_local_minima(loader)
    
    # Example 11: Velocity-based adaptive sampling
    # sampled_frames, is_minima_flags = example_velocity_based_sampling(loader)
    
    print("\n\n=== All examples completed! ===")


if __name__ == "__main__":
    main()

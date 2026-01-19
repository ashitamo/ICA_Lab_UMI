"""
Visualization Tools for Robot Demonstration Data

This module provides tools to create and display GIF animations from image sequences.
"""

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import io
from typing import List, Optional, Union
import tempfile
import os


def add_text_to_image(image: np.ndarray, 
                      text_lines: List[str],
                      position: str = 'top',
                      font_scale: float = 0.7,
                      thickness: int = 2,
                      bg_alpha: float = 0.7) -> np.ndarray:
    """
    在影像上加入文字標籤
    
    Args:
        image: 輸入影像 (BGR format from cv2)
        text_lines: 文字行列表
        position: 文字位置 ('top' 或 'bottom')
        font_scale: 字體大小
        thickness: 字體粗細
        bg_alpha: 背景透明度 (0-1)
        
    Returns:
        加上文字的影像
    """
    img = image.copy()
    height, width = img.shape[:2]
    
    # 設定字體
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    # 計算每行文字的大小
    text_sizes = []
    for text in text_lines:
        (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        text_sizes.append((text_w, text_h, baseline))
    
    # 計算總高度
    line_spacing = 10
    total_height = sum(h + baseline for _, h, baseline in text_sizes) + line_spacing * (len(text_lines) + 1)
    max_width = max(w for w, _, _ in text_sizes)
    
    # 決定位置
    if position == 'top':
        y_start = line_spacing
    else:  # bottom
        y_start = height - total_height
    
    # 繪製半透明背景
    overlay = img.copy()
    cv2.rectangle(overlay, (0, y_start - line_spacing), 
                  (width, y_start + total_height), 
                  (0, 0, 0), -1)
    cv2.addWeighted(overlay, bg_alpha, img, 1 - bg_alpha, 0, img)
    
    # 繪製文字
    y_offset = y_start
    for i, (text, (text_w, text_h, baseline)) in enumerate(zip(text_lines, text_sizes)):
        y_offset += text_h + line_spacing
        
        # 白色文字，黑色邊框
        cv2.putText(img, text, (10, y_offset), font, font_scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(img, text, (10, y_offset), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
    
    return img


def images_to_gif(images: List[np.ndarray], 
                  duration: int = 100, 
                  loop: int = 0,
                  resize: Optional[tuple] = None) -> bytes:
    """
    Convert a list of images to a GIF animation.
    
    Args:
        images: List of images as numpy arrays (BGR format from cv2)
        duration: Duration of each frame in milliseconds (default: 100ms)
        loop: Number of loops (0 = infinite loop, default: 0)
        resize: Optional tuple (width, height) to resize images
        
    Returns:
        GIF animation as bytes
        
    Example:
        >>> images = [frame.image for frame in loader.frames[::10]]
        >>> gif_data = images_to_gif(images, duration=50)
    """
    if not images:
        raise ValueError("Image list is empty")
    
    # Convert BGR (OpenCV) to RGB (PIL)
    pil_images = []
    for img in images:
        # Convert BGR to RGB
        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Resize if requested
        if resize:
            rgb_img = cv2.resize(rgb_img, resize)
        
        # Convert to PIL Image
        pil_img = Image.fromarray(rgb_img)
        pil_images.append(pil_img)
    
    # Save to bytes buffer
    gif_buffer = io.BytesIO()
    pil_images[0].save(
        gif_buffer,
        format='GIF',
        save_all=True,
        append_images=pil_images[1:],
        duration=duration,
        loop=loop
    )
    
    gif_buffer.seek(0)
    return gif_buffer.getvalue()


def save_gif(images: List[np.ndarray], 
             output_path: str,
             duration: int = 30,
             loop: int = 0,
             resize: Optional[tuple] = None,
             text_lines: Optional[List[str]] = None,
             text_position: str = 'top',
             gif_fps: Optional[int] = None,
             frame_step: int = 1):
    """
    Save a list of images as a GIF file.
    
    Args:
        images: List of images as numpy arrays (BGR format from cv2)
        output_path: Path to save the GIF file
        duration: Duration of each frame in milliseconds (default: 100ms)
        loop: Number of loops (0 = infinite loop, default: 0)
        resize: Optional tuple (width, height) to resize images
        text_lines: Optional list of text lines to overlay on images
        text_position: Position of text ('top' or 'bottom')
        
    Example:
        >>> images = [frame.image for frame in loader.frames[::10]]
        >>> save_gif(images, "output/demo.gif", duration=50, 
        ...          text_lines=["PT: Pick Object", "AA: Reach"])
    """
    # Subsample frames if requested (frame_step > 1)
    if frame_step and frame_step > 1:
        images = images[::frame_step]

    # Add text overlay if requested
    if text_lines:
        images = [add_text_to_image(img, text_lines, text_position) for img in images]

    # If gif_fps is provided, override duration (ms per frame)
    if gif_fps and gif_fps > 0:
        duration = max(1, int(1000 / gif_fps))

    gif_data = images_to_gif(images, duration=duration, loop=loop, resize=resize)
    
    with open(output_path, 'wb') as f:
        f.write(gif_data)
    
    print(f"GIF saved to: {output_path}")


def view_gif(gif_data: Union[bytes, str], 
             window_name: str = "GIF Animation",
             fps: int = 10):
    """
    Display a GIF animation in a window.
    
    Args:
        gif_data: Either bytes of GIF data or path to GIF file
        window_name: Name of the display window
        fps: Frames per second for playback (default: 10)
        
    Note:
        Press 'q' or ESC to close the window.
        Press SPACE to pause/resume.
        
    Example:
        >>> images = [frame.image for frame in loader.frames[::10]]
        >>> gif_data = images_to_gif(images)
        >>> view_gif(gif_data)
    """
    # Load GIF
    if isinstance(gif_data, str):
        # It's a file path
        gif = Image.open(gif_data)
    else:
        # It's bytes data
        gif_buffer = io.BytesIO(gif_data)
        gif = Image.open(gif_buffer)
    
    # Extract all frames
    frames = []
    try:
        while True:
            # Convert PIL Image to numpy array (RGB)
            frame_rgb = np.array(gif.convert('RGB'))
            # Convert RGB to BGR for OpenCV
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            frames.append(frame_bgr)
            gif.seek(gif.tell() + 1)
    except EOFError:
        pass  # End of GIF
    
    if not frames:
        print("No frames found in GIF")
        return
    
    print(f"Loaded {len(frames)} frames from GIF")
    print("Controls:")
    print("  SPACE: Pause/Resume")
    print("  'q' or ESC: Quit")
    
    # Display frames in a loop
    frame_delay = int(1000 / fps)  # milliseconds
    current_frame = 0
    paused = False
    
    while True:
        if not paused:
            cv2.imshow(window_name, frames[current_frame])
            current_frame = (current_frame + 1) % len(frames)
        
        key = cv2.waitKey(frame_delay)
        
        # Check for key presses
        if key == ord('q') or key == 27:  # 'q' or ESC
            break
        elif key == ord(' '):  # SPACE
            paused = not paused
            status = "PAUSED" if paused else "PLAYING"
            print(f"Status: {status}")
    
    cv2.destroyAllWindows()


def create_comparison_gif(image_lists: List[List[np.ndarray]], 
                         labels: Optional[List[str]] = None,
                         duration: int = 100,
                         loop: int = 0) -> bytes:
    """
    Create a side-by-side comparison GIF from multiple image sequences.
    
    Args:
        image_lists: List of image sequences to compare
        labels: Optional labels for each sequence
        duration: Duration of each frame in milliseconds
        loop: Number of loops (0 = infinite)
        
    Returns:
        GIF animation as bytes
        
    Example:
        >>> seq1 = [frame.image for frame in loader1.frames[::10]]
        >>> seq2 = [frame.image for frame in loader2.frames[::10]]
        >>> gif_data = create_comparison_gif([seq1, seq2], labels=["Demo 1", "Demo 2"])
    """
    if not image_lists:
        raise ValueError("No image sequences provided")
    
    # Find minimum length
    min_length = min(len(seq) for seq in image_lists)
    if min_length == 0:
        raise ValueError("One or more image sequences are empty")
    
    # Truncate all sequences to same length
    image_lists = [seq[:min_length] for seq in image_lists]
    
    # Create combined frames
    combined_frames = []
    for frame_idx in range(min_length):
        # Get frames at this index from all sequences
        frames = [seq[frame_idx] for seq in image_lists]
        
        # Add labels if provided
        if labels:
            labeled_frames = []
            for frame, label in zip(frames, labels):
                labeled_frame = frame.copy()
                cv2.putText(labeled_frame, label, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                labeled_frames.append(labeled_frame)
            frames = labeled_frames
        
        # Concatenate horizontally
        combined = np.hstack(frames)
        combined_frames.append(combined)
    
    # Create GIF
    return images_to_gif(combined_frames, duration=duration, loop=loop)


def add_timestamp_overlay(images: List[np.ndarray], 
                         timestamps: List[float],
                         position: tuple = (10, 30),
                         color: tuple = (255, 255, 255),
                         font_scale: float = 1.0) -> List[np.ndarray]:
    """
    Add timestamp overlays to images.
    
    Args:
        images: List of images
        timestamps: List of timestamps (in seconds)
        position: Position of text (x, y)
        color: Text color (B, G, R)
        font_scale: Font size scale
        
    Returns:
        List of images with timestamp overlays
        
    Example:
        >>> images = [frame.image for frame in loader.frames[::10]]
        >>> timestamps = [frame.timestamp for frame in loader.frames[::10]]
        >>> images_with_time = add_timestamp_overlay(images, timestamps)
        >>> gif_data = images_to_gif(images_with_time)
    """
    if len(images) != len(timestamps):
        raise ValueError("Number of images and timestamps must match")
    
    overlay_images = []
    for img, ts in zip(images, timestamps):
        img_copy = img.copy()
        text = f"t={ts:.2f}s"
        cv2.putText(img_copy, text, position, cv2.FONT_HERSHEY_SIMPLEX,
                   font_scale, color, 2, cv2.LINE_AA)
        overlay_images.append(img_copy)
    
    return overlay_images


def add_velocity_overlay(images: List[np.ndarray], 
                        v_abs_values: List[float],
                        position: Optional[tuple] = None,
                        color: tuple = (255, 255, 0),
                        font_scale: float = 0.7,
                        background: bool = True) -> List[np.ndarray]:
    """
    Add velocity (v_abs) overlays to images.
    
    Args:
        images: List of images
        v_abs_values: List of velocity absolute values
        position: Position of text (x, y). If None, defaults to top-right corner
        color: Text color (B, G, R), default is cyan
        font_scale: Font size scale
        background: Whether to add a semi-transparent background for better readability
        
    Returns:
        List of images with velocity overlays
        
    Example:
        >>> images = [frame.image for frame in loader.frames[::10]]
        >>> v_abs = [frame.v_abs for frame in loader.frames[::10]]
        >>> images_with_velocity = add_velocity_overlay(images, v_abs)
    """
    if len(images) != len(v_abs_values):
        raise ValueError("Number of images and v_abs values must match")
    
    overlay_images = []
    for img, v_abs in zip(images, v_abs_values):
        img_copy = img.copy()
        h, w = img_copy.shape[:2]
        
        # Default position: top-right corner
        if position is None:
            text = f"v={v_abs:.4f} m/s"
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2)[0]
            pos = (w - text_size[0] - 10, 30)
        else:
            pos = position
            text = f"v={v_abs:.4f} m/s"
        
        # Add semi-transparent background if requested
        if background:
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2)[0]
            bg_top_left = (pos[0] - 5, pos[1] - text_size[1] - 5)
            bg_bottom_right = (pos[0] + text_size[0] + 5, pos[1] + 5)
            
            # Create overlay for transparency
            overlay = img_copy.copy()
            cv2.rectangle(overlay, bg_top_left, bg_bottom_right, (0, 0, 0), -1)
            # Blend with original image (alpha = 0.6 for background)
            cv2.addWeighted(overlay, 0.6, img_copy, 0.4, 0, img_copy)
        
        # Add text
        cv2.putText(img_copy, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                   font_scale, color, 2, cv2.LINE_AA)
        overlay_images.append(img_copy)
    
    return overlay_images


if __name__ == "__main__":
    # Simple test
    print("Visualization tools module loaded successfully")
    print("\nAvailable functions:")
    print("  - images_to_gif(): Convert image list to GIF bytes")
    print("  - save_gif(): Save image list as GIF file")
    print("  - view_gif(): Display GIF animation")
    print("  - create_comparison_gif(): Create side-by-side comparison GIF")
    print("  - add_timestamp_overlay(): Add timestamps to images")
    print("  - add_velocity_overlay(): Add velocity (v_abs) to images")

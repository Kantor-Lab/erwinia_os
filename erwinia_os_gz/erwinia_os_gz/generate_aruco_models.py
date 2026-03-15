#!/usr/bin/env python3
"""
Generate ArUco marker models (textures + SDF files) directly into package share.

Example:
  python3 tools/generate_aruco_models.py --count 10 --dict DICT_4X4_50 --size 0.05
  
This will create markers 0-9 with textures and model directories ready for Gazebo.
"""

from __future__ import annotations

import os
import sys
import argparse
from pathlib import Path
from typing import List

try:
    import cv2
    import numpy as np
except Exception as e:
    raise RuntimeError(
        "OpenCV not found. Install python3-opencv (apt) or opencv-contrib-python (pip)."
    ) from e

# Try to get package share directory if running as ROS2 node
try:
    from ament_index_python.packages import get_package_share_directory
    USE_AMENT = True
except ImportError:
    USE_AMENT = False


# Map friendly dict names to OpenCV constants
ARUCO_DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


MODEL_SDF_TEMPLATE = '''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="aruco_marker_{marker_id}">
    <static>true</static>

    <link name="link">
      <pose>0 0 0 0 0 0</pose>

      <visual name="visual">
        <cast_shadows>false</cast_shadows>

        <geometry>
          <box><size>{size} {size} 0.002</size></box>
        </geometry>

        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <emissive>0 0 0 1</emissive>
          <pbr>
            <metal>
              <albedo_map>../tree_fiducials/materials/textures/aruco_{dict_name}_id{marker_id:04d}.png</albedo_map>
              <metalness>0.0</metalness>
              <roughness>1.0</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''

MODEL_CONFIG_TEMPLATE = '''<?xml version="1.0"?>
<model>
  <name>aruco_marker_{marker_id}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>ArUco marker {dict_name} ID {marker_id} as a textured plate ({size}m).</description>
</model>
'''


def get_package_models_dir() -> Path:
    """Get the models directory in package share (install space)."""
    if USE_AMENT:
        try:
            pkg_share = get_package_share_directory('erwinia_os_gz')
            return Path(pkg_share) / 'models'
        except Exception:
            pass
    
    # Fallback: assume we're in the source tree
    script_dir = Path(__file__).parent.absolute()
    # Go from tools/ to package root, then find install
    pkg_root = script_dir.parent.parent.parent.parent
    install_share = pkg_root / 'install' / 'erwinia_os_gz' / 'share' / 'erwinia_os_gz' / 'models'
    
    if install_share.exists():
        return install_share
    
    # If install doesn't exist, use source models directory
    return script_dir.parent / 'models'


def ensure_aruco_available() -> None:
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "Your OpenCV build does not include cv2.aruco. "
            "Install opencv-contrib-python (pip) or a system OpenCV build with aruco."
        )


def generate_marker_texture(
    aruco_dict_name: str,
    marker_id: int,
    size_px: int,
    marker_border: float,
    textures_dir: Path,
    marker_size: float = 0.05,
) -> Path:
    """Generate a single ArUco marker texture PNG with white border.
    
    Args:
        aruco_dict_name: Name of the ArUco dictionary
        marker_id: ID of the marker to generate
        size_px: Total texture size in pixels (includes border)
        marker_border: Border size in meters
        textures_dir: Directory to save textures
        marker_size: Physical marker size in meters (used to calculate border ratio)
    """
    ensure_aruco_available()

    if aruco_dict_name not in ARUCO_DICT_MAP:
        raise ValueError(
            f"Unknown dict '{aruco_dict_name}'. Choose from: {', '.join(ARUCO_DICT_MAP.keys())}"
        )

    dict_id = ARUCO_DICT_MAP[aruco_dict_name]
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

    # Calculate marker size in pixels based on physical proportions
    # Total physical size = marker_size + 2*marker_border (border on each side)
    total_physical_size = marker_size + 2 * marker_border
    marker_ratio = marker_size / total_physical_size
    marker_size_px = int(size_px * marker_ratio)
    
    # Ensure marker size is at least reasonable
    marker_size_px = max(marker_size_px, 100)

    # Generate the ArUco marker at the calculated size
    marker_img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size_px, borderBits=1)

    # Create white background canvas at full texture size
    img = np.ones((size_px, size_px), dtype=np.uint8) * 255

    # Calculate offset to center the marker on the white background
    offset = (size_px - marker_size_px) // 2

    # Place marker in center of white background
    img[offset:offset + marker_size_px, offset:offset + marker_size_px] = marker_img

    # Write texture file
    textures_dir.mkdir(parents=True, exist_ok=True)
    texture_path = textures_dir / f"aruco_{aruco_dict_name}_id{marker_id:04d}.png"
    
    ok = cv2.imwrite(str(texture_path), img)
    if not ok:
        raise RuntimeError(f"Failed to write image to {texture_path}")
    
    return texture_path


def create_aruco_model_dir(
    marker_id: int,
    dict_name: str,
    size: float,
    models_dir: Path,
) -> Path:
    """Create model.sdf and model.config for a marker."""
    model_name = f"aruco_marker_{marker_id}"
    model_dir = models_dir / model_name
    model_dir.mkdir(parents=True, exist_ok=True)
    
    # Write model.sdf
    sdf_content = MODEL_SDF_TEMPLATE.format(
        marker_id=marker_id,
        dict_name=dict_name,
        size=size,
    )
    (model_dir / "model.sdf").write_text(sdf_content)
    
    # Write model.config
    config_content = MODEL_CONFIG_TEMPLATE.format(
        marker_id=marker_id,
        dict_name=dict_name,
        size=size,
    )
    (model_dir / "model.config").write_text(config_content)
    
    return model_dir


def generate_aruco_models(
    marker_ids: List[int],
    dict_name: str = "DICT_4X4_50",
    marker_size: float = 0.05,
    size_px: int = 800,
    marker_border: float = 0.005,
    target_dir: Path | None = None,
) -> None:
    """
    Generate complete ArUco marker models (textures + SDF files).
    
    Args:
        marker_ids: List of marker IDs to generate
        dict_name: ArUco dictionary name
        marker_size: Physical size in meters
        size_px: Texture size in pixels
        marker_border: Border size in meters
        target_dir: Target models directory (defaults to package share)
    """
    if target_dir is None:
        models_dir = get_package_models_dir()
    else:
        models_dir = target_dir
    
    textures_dir = models_dir / "tree_fiducials" / "materials" / "textures"
    
    print(f"Generating {len(marker_ids)} ArUco marker models")
    print(f"  Dictionary: {dict_name}")
    print(f"  Marker size: {marker_size}m")
    print(f"  Border size: {marker_border}m (on each side)")
    print(f"  Total physical size: {marker_size + 2 * marker_border}m")
    print(f"  Texture size: {size_px}px")
    print(f"  Target directory: {models_dir}")
    print()
    
    for marker_id in marker_ids:
        # Generate texture of the marker sitting on white background
        texture_path = generate_marker_texture(
            dict_name, marker_id, size_px, marker_border, textures_dir, marker_size
        )
        
        # Create model directory using the total size (marker + 2*border for both sides)
        model_dir = create_aruco_model_dir(
            marker_id, dict_name, (marker_size + 2 * marker_border), models_dir
        )
        
        print(f"✓ Created aruco_marker_{marker_id}")
    
    print(f"\n✓ Successfully generated {len(marker_ids)} marker models")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Generate ArUco marker models for Gazebo."
    )
    ap.add_argument(
        "--ids", type=int, nargs="*", default=None,
        help="Specific marker IDs to generate. If omitted, uses --start and --count."
    )
    ap.add_argument(
        "--start", type=int, default=0,
        help="Start ID if --ids not provided (default: 0)"
    )
    ap.add_argument(
        "--count", type=int, default=2,
        help="How many markers to generate if --ids not provided (default: 2)"
    )
    ap.add_argument(
        "--dict", dest="dict_name", default="DICT_4X4_50",
        help=f"ArUco dictionary (default: DICT_4X4_50). Options: {', '.join(ARUCO_DICT_MAP.keys())}"
    )
    ap.add_argument(
        "--marker_size", type=float, default=0.05,
        help="Physical marker size in meters (default: 0.05)"
    )
    ap.add_argument(
        "--size_px", type=int, default=800,
        help="Texture size in pixels (default: 800)"
    )
    ap.add_argument(
        "--border_size", type=float, default=0.005,
        help="White border size in meters on each side (default: 0.005)"
    )
    ap.add_argument(
        "--target_dir", type=str, default="",
        help="Target models directory (default: package share/models)"
    )
    args = ap.parse_args()
    
    # Determine marker IDs
    if args.ids and len(args.ids) > 0:
        marker_ids = args.ids
    else:
        marker_ids = list(range(args.start, args.start + args.count))
    
    target_dir = Path(args.target_dir) if args.target_dir else None
    
    try:
        generate_aruco_models(
            marker_ids,
            args.dict_name,
            args.marker_size,
            args.size_px,
            args.border_size,
            target_dir,
        )
        return 0
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

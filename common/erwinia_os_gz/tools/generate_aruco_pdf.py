#!/usr/bin/env python3
"""
Generate printable PDF of ArUco markers from generated textures.

Example:
  python3 tools/generate_aruco_pdf.py --dict DICT_4X4_50 --marker_size 0.05 --border_size 0.005
  
This will create a PDF with all available markers arranged on US Letter pages.
"""

from __future__ import annotations

import sys
import argparse
import re
from pathlib import Path
from typing import List

try:
    from reportlab.lib.pagesizes import letter
    from reportlab.lib.units import inch, mm
    from reportlab.pdfgen import canvas
    from reportlab.lib.utils import ImageReader
except ImportError as e:
    raise RuntimeError(
        "reportlab not found. Install with: pip install reportlab"
    ) from e

# Try to get package share directory if running as ROS2 node
try:
    from ament_index_python.packages import get_package_share_directory
    USE_AMENT = True
except ImportError:
    USE_AMENT = False


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


def meters_to_points(meters: float) -> float:
    """Convert meters to PDF points (1 inch = 72 points, 1 meter = 39.3701 inches)."""
    inches = meters * 39.3701
    return inches * 72


def find_marker_textures(textures_dir: Path, dict_name: str) -> List[int]:
    """Find all available marker textures in the directory and extract their IDs.
    
    Args:
        textures_dir: Directory containing marker textures
        dict_name: ArUco dictionary name to filter
    
    Returns:
        Sorted list of marker IDs found
    """
    marker_ids = []
    pattern = re.compile(rf"aruco_{re.escape(dict_name)}_id(\d+)\.png")
    
    if not textures_dir.exists():
        return marker_ids
    
    for texture_file in textures_dir.glob(f"aruco_{dict_name}_id*.png"):
        match = pattern.match(texture_file.name)
        if match:
            marker_id = int(match.group(1))
            marker_ids.append(marker_id)
    
    return sorted(marker_ids)


def generate_aruco_pdf(
    marker_ids: List[int] | None,
    dict_name: str,
    marker_size: float,
    border_size: float,
    output_path: Path,
    models_dir: Path | None = None,
    margin: float = 0.5,  # margin in inches
    cut_border_width: float = 1.5,  # black border width in points
) -> None:
    """
    Generate a printable PDF with ArUco markers arranged on US Letter pages.
    
    Args:
        marker_ids: List of marker IDs to include (None = auto-detect all)
        dict_name: ArUco dictionary name
        marker_size: Physical marker size in meters
        border_size: Border size in meters (on each side)
        output_path: Path to save the PDF
        models_dir: Models directory (defaults to package share)
        margin: Page margin in inches
        cut_border_width: Width of black cut line border in points
    """
    if models_dir is None:
        models_dir = get_package_models_dir()
    
    textures_dir = models_dir / "tree_fiducials" / "materials" / "textures"
    
    # Auto-detect markers if not specified
    if marker_ids is None or len(marker_ids) == 0:
        marker_ids = find_marker_textures(textures_dir, dict_name)
        if not marker_ids:
            raise RuntimeError(
                f"No marker textures found in {textures_dir}.\n"
                f"Run generate_aruco_models.py first to create marker textures."
            )
    
    # US Letter dimensions: 8.5 x 11 inches
    page_width, page_height = letter
    
    # Calculate total physical size (marker + borders on both sides)
    total_marker_size = marker_size + 2 * border_size
    
    # Convert to PDF points
    marker_size_pts = meters_to_points(total_marker_size)
    margin_pts = margin * inch
    
    # Calculate usable page area
    usable_width = page_width - 2 * margin_pts
    usable_height = page_height - 2 * margin_pts
    
    # Determine layout: 1, 2, 4, or 6 markers per page based on size
    spacing_pts = 0.75 * inch  # spacing between markers and for ID label (increased for more room)
    
    # Try 6 markers (3x2 or 2x3)
    if (3 * marker_size_pts + 2 * spacing_pts) <= usable_width and \
       (2 * marker_size_pts + spacing_pts) <= usable_height:
        markers_per_row = 3
        markers_per_col = 2
    elif (2 * marker_size_pts + spacing_pts) <= usable_width and \
         (3 * marker_size_pts + 2 * spacing_pts) <= usable_height:
        markers_per_row = 2
        markers_per_col = 3
    # Try 4 markers (2x2)
    elif (2 * marker_size_pts + spacing_pts) <= usable_width and \
         (2 * marker_size_pts + spacing_pts) <= usable_height:
        markers_per_row = 2
        markers_per_col = 2
    # Try 2 markers (2x1 or 1x2)
    elif (2 * marker_size_pts + spacing_pts) <= usable_width:
        markers_per_row = 2
        markers_per_col = 1
    elif (2 * marker_size_pts + spacing_pts) <= usable_height:
        markers_per_row = 1
        markers_per_col = 2
    # Single marker per page
    else:
        markers_per_row = 1
        markers_per_col = 1
    
    markers_per_page = markers_per_row * markers_per_col
    
    print(f"Generating ArUco marker PDF")
    print(f"  Dictionary: {dict_name}")
    print(f"  Marker physical size: {marker_size}m")
    print(f"  Border size: {border_size}m (each side)")
    print(f"  Total physical size: {total_marker_size}m")
    print(f"  PDF size: {marker_size_pts:.1f} points ({total_marker_size * 39.3701:.2f} inches)")
    print(f"  Layout: {markers_per_page} marker(s) per page ({markers_per_row} x {markers_per_col})")
    print(f"  Total markers: {len(marker_ids)}")
    print(f"  Pages needed: {(len(marker_ids) + markers_per_page - 1) // markers_per_page}")
    print(f"  Output: {output_path}")
    print()
    
    # Create PDF
    c = canvas.Canvas(str(output_path), pagesize=letter)
    
    # Calculate spacing to center the grid on the page
    # Add extra space for ID labels
    label_space = 20  # points for ID label
    total_marker_height = marker_size_pts + label_space
    
    total_grid_width = markers_per_row * marker_size_pts + (markers_per_row - 1) * spacing_pts
    total_grid_height = markers_per_col * total_marker_height + (markers_per_col - 1) * spacing_pts
    
    start_x = (page_width - total_grid_width) / 2
    start_y = page_height - (page_height - total_grid_height) / 2 - total_marker_height
    
    marker_count = 0
    for marker_id in marker_ids:
        # Find the texture file
        texture_path = textures_dir / f"aruco_{dict_name}_id{marker_id:04d}.png"
        
        if not texture_path.exists():
            print(f"Warning: Texture not found for marker {marker_id}: {texture_path}")
            print(f"         Run generate_aruco_models.py first to create marker textures.")
            continue
        
        # Calculate position on current page
        row = (marker_count % markers_per_page) // markers_per_row
        col = (marker_count % markers_per_page) % markers_per_row
        
        # Start new page if needed
        if marker_count > 0 and marker_count % markers_per_page == 0:
            c.showPage()
        
        # Calculate position
        x = start_x + col * (marker_size_pts + spacing_pts)
        y = start_y - row * (total_marker_height + spacing_pts)
        
        # Draw the marker image
        try:
            img = ImageReader(str(texture_path))
            c.drawImage(img, x, y, width=marker_size_pts, height=marker_size_pts)
            
            # Draw black border around the marker (cut line)
            c.setStrokeColorRGB(0, 0, 0)
            c.setLineWidth(cut_border_width)
            c.rect(x, y, marker_size_pts, marker_size_pts, stroke=1, fill=0)
            
            # Add marker ID label to the right of the marker
            label_x = x + marker_size_pts + 10  # 10 points to the right
            label_y = y + marker_size_pts / 2  # Centered vertically
            c.setFont("Helvetica-Bold", 12)
            c.setFillColorRGB(0, 0, 0)
            c.drawString(label_x, label_y, f"ID: {marker_id}")
            
            # Also add smaller label below for reference
            label_below_y = y - 15  # 15 points below marker
            c.setFont("Helvetica", 8)
            c.drawCentredString(x + marker_size_pts / 2, label_below_y, 
                               f"{dict_name} | {total_marker_size * 1000:.1f}mm")
            
            print(f"✓ Added marker {marker_id} (page {marker_count // markers_per_page + 1})")
        except Exception as e:
            print(f"Error drawing marker {marker_id}: {e}")
            continue
        
        marker_count += 1
    
    # Save the PDF
    c.save()
    
    print(f"\n✓ Successfully generated PDF with {marker_count} markers")
    print(f"  Saved to: {output_path}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Generate printable PDF of ArUco markers. "
                    "Auto-detects all available markers if --ids not specified."
    )
    ap.add_argument(
        "--ids", type=int, nargs="*", default=None,
        help="Marker IDs to include in PDF (default: auto-detect all available)"
    )
    ap.add_argument(
        "--dict", dest="dict_name", default="DICT_4X4_50",
        help="ArUco dictionary name (default: DICT_4X4_50)"
    )
    ap.add_argument(
        "--marker_size", type=float, default=0.05,
        help="Physical marker size in meters (default: 0.05)"
    )
    ap.add_argument(
        "--border_size", type=float, default=0.005,
        help="White border size in meters on each side (default: 0.005)"
    )
    ap.add_argument(
        "--output", type=str, default="aruco_markers.pdf",
        help="Output PDF filename (default: aruco_markers.pdf)"
    )
    ap.add_argument(
        "--models_dir", type=str, default="",
        help="Models directory (default: package share/models)"
    )
    ap.add_argument(
        "--margin", type=float, default=0.5,
        help="Page margin in inches (default: 0.5)"
    )
    ap.add_argument(
        "--cut_border", type=float, default=1.5,
        help="Black cut line border width in points (default: 1.5)"
    )
    args = ap.parse_args()
    
    models_dir = Path(args.models_dir) if args.models_dir else None
    output_path = Path(args.output)
    
    try:
        generate_aruco_pdf(
            args.ids,
            args.dict_name,
            args.marker_size,
            args.border_size,
            output_path,
            models_dir,
            args.margin,
            args.cut_border,
        )
        return 0
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

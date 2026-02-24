"""
Point Cloud Visualization Tool

Simple command-line tool to visualize point clouds.
"""

import sys
import argparse
import open3d as o3d
from pathlib import Path


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Visualize point cloud files')
    parser.add_argument('file', type=str, help='Path to point cloud file (.pcd, .ply)')
    parser.add_argument('--normals', action='store_true', help='Show normals')
    parser.add_argument('--voxel', type=float, default=0.0, help='Voxel size for downsampling')

    args = parser.parse_args()

    file_path = Path(args.file)
    if not file_path.exists():
        print(f"Error: File not found: {file_path}")
        return 1

    print(f"Loading point cloud: {file_path}")
    pcd = o3d.io.read_point_cloud(str(file_path))

    print(f"  Points: {len(pcd.points)}")
    print(f"  Has normals: {pcd.has_normals()}")
    print(f"  Has colors: {pcd.has_colors()}")

    # Preprocess if requested
    if args.voxel > 0:
        print(f"Downsampling with voxel size: {args.voxel}")
        pcd = pcd.voxel_down_sample(args.voxel)
        print(f"  Points after downsampling: {len(pcd.points)}")

    # Estimate normals
    if args.normals and not pcd.has_normals():
        print("Estimating normals...")
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1, max_nn=30
            )
        )
        pcd.orient_normals_consistent_tangent_plane(30)
        print("  Normals estimated")

    # Visualize
    print("Starting visualizer...")
    print("Controls:")
    print("  - Mouse drag: Rotate")
    print("  - Ctrl + drag: Pan")
    print("  - Scroll wheel: Zoom")
    print("  - N: Toggle normals")
    print("  - Q: Exit")

    o3d.visualization.draw_geometries(
        [pcd],
        window_name="Point Cloud Visualizer",
        width=1920,
        height=1080
    )

    print("Visualization closed")
    return 0


if __name__ == '__main__':
    sys.exit(main())

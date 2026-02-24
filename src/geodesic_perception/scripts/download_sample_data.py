#!/usr/bin/env python3
"""
Download Sample Point Cloud Data

下载用于测试的示例点云数据。
"""

import os
import sys
import urllib.request
from pathlib import Path


# 下载数据集
DATASETS = {
    'fragment.pcd': 'https://raw.githubusercontent.com/isl-org/Open3D/master/examples/test_data/fragment.pcd',
    'scene.pcd': 'https://raw.githubusercontent.com/isl-org/Open3D/master/examples/test_data/scene.pcd',
    'chair.ply': 'https://raw.githubusercontent.com/isl-org/Open3D/master/examples/test_data/chair.ply',
}


def download_file(url, dest_path):
    """下载文件"""
    print(f"  下载 {os.path.basename(dest_path)}...")

    try:
        urllib.request.urlretrieve(url, dest_path)
        print(f"  ✅ {os.path.basename(dest_path)} 下载完成")
        return True
    except Exception as e:
        print(f"  ❌ 下载失败: {e}")
        return False


def create_synthetic_data(output_dir):
    """创建合成测试数据"""
    print("\n📝 创建合成测试数据...")

    try:
        import open3d as o3d
        import numpy as np

        # 创建平面点云（模拟工件表面）
        print("  创建平面点云...")
        x = np.random.uniform(-0.3, 0.3, 5000)
        y = np.random.uniform(-0.2, 0.2, 5000)
        z = np.random.normal(0.5, 0.001, 5000)  # 平面在z=0.5

        plane_pcd = o3d.geometry.PointCloud()
        plane_pcd.points = o3d.utility.Vector3dVector(np.column_stack([x, y, z]))

        # 添加一些"孔"（低点）
        for i in range(5):
            cx, cy = np.random.uniform(-0.2, 0.2, 2)
            hole_points = np.random.normal([cx, cy, 0.48], 0.01, (100, 3))
            hole_pcd = o3d.geometry.PointCloud()
            hole_pcd.points = o3d.utility.Vector3dVector(hole_points)
            plane_pcd += hole_pcd

        output_path = output_dir / "synthetic_plane.pcd"
        o3d.io.write_point_cloud(str(output_path), plane_pcd)
        print(f"  ✅ 合成数据已保存: {output_path}")

        # 创建带噪声的版本
        print("  创建带噪声的版本...")
        noisy_pcd = o3d.geometry.PointCloud(plane_pcd)
        points = np.asarray(noisy_pcd.points)
        noise = np.random.normal(0, 0.005, points.shape)  # 5mm噪声
        noisy_pcd.points = o3d.utility.Vector3dVector(points + noise)

        output_path = output_dir / "synthetic_plane_noisy.pcd"
        o3d.io.write_point_cloud(str(output_path), noisy_pcd)
        print(f"  ✅ 噪声数据已保存: {output_path}")

        return True

    except ImportError as e:
        print(f"  ❌ 需要Open3D: {e}")
        return False
    except Exception as e:
        print(f"  ❌ 创建合成数据失败: {e}")
        return False


def main():
    """主函数"""
    print("=" * 60)
    print("ProjectGeodesic - 下载示例数据")
    print("=" * 60)

    # 创建数据目录
    script_dir = Path(__file__).parent.parent.parent
    data_dir = script_dir / "data" / "sample_data"
    data_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n📁 数据目录: {data_dir}")
    print("\n📥 下载示例数据...\n")

    # 下载数据集
    success_count = 0
    for filename, url in DATASETS.items():
        dest_path = data_dir / filename
        if dest_path.exists():
            print(f"  ⏭️  {filename} 已存在，跳过")
            success_count += 1
            continue

        if download_file(url, dest_path):
            success_count += 1

    # 创建合成数据
    if create_synthetic_data(data_dir):
        success_count += 1

    # 总结
    print("\n" + "=" * 60)
    if success_count == len(DATASETS) + 1:
        print("✅ 所有数据下载完成！")
        print(f"\n数据位置: {data_dir}")
        print("\n可用的测试数据:")
        for file in sorted(data_dir.glob("*")):
            size_mb = file.stat().st_size / 1024 / 1024
            print(f"  - {file.name} ({size_mb:.2f} MB)")
        return 0
    else:
        print("⚠️  部分数据下载失败")
        return 1


if __name__ == '__main__':
    sys.exit(main())

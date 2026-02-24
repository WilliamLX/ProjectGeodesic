#!/usr/bin/env python3
"""
Registration Demo

演示完整的点云配准流程。
"""

import sys
import numpy as np
from pathlib import Path

# 添加模块路径
script_dir = Path(__file__).parent.parent
sys.path.insert(0, str(script_dir))

import open3d as o3d
from open3d.visualization import draw_geometries

from geodesic_perception.pointcloud_proc import PointCloudProcessor
from geodesic_perception.registration import Registration


def create_synthetic_workpiece():
    """创建合成工件点云（模拟前顶板）"""
    print("📝 创建合成工件点云...")

    # 创建矩形板
    x = np.random.uniform(-0.3, 0.3, 8000)
    y = np.random.uniform(-0.2, 0.2, 8000)
    z = np.random.normal(0, 0.002, 8000)  # 2mm厚度噪声

    points = np.column_stack([x, y, z])

    # 添加13个"螺丝孔"（凹陷）
    hole_positions = [
        (-0.25, -0.15), (-0.25, 0), (-0.25, 0.15),
        (0, -0.15), (0, 0), (0, 0.15),
        (0.25, -0.15), (0.25, 0), (0.25, 0.15),
        (-0.15, -0.18), (0.15, -0.18),
        (-0.15, 0.18), (0.15, 0.18),
    ]

    for hx, hy in hole_positions:
        # 孔周围的点更低
        mask = (np.abs(x - hx) < 0.01) & (np.abs(y - hy) < 0.01)
        z[mask] -= 0.01  # 10mm深

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.column_stack([x, y, z]))

    # 添加颜色（灰色）
    colors = np.ones_like(points) * 0.7
    pcd.colors = o3d.utility.Vector3dVector(colors)

    print(f"  ✅ 创建工件点云: {len(pcd.points)} 点")
    print(f"  ✅ 添加了 {len(hole_positions)} 个螺丝孔")

    return pcd


def transform_workpiece(pcd, translation, rotation_axis='z', rotation_deg=0):
    """变换工件点云"""
    T = np.eye(4)

    # 旋转
    if rotation_deg != 0:
        theta = np.deg2rad(rotation_deg)
        c, s = np.cos(theta), np.sin(theta)

        if rotation_axis == 'z':
            R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        elif rotation_axis == 'y':
            R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        else:  # x
            R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

        T[:3, :3] = R

    # 平移
    T[:3, 3] = translation

    return pcd.transform(T), T


def visualize_result(target, source, aligned):
    """可视化配准结果"""
    print("\n📊 可视化结果...")

    # 给点云上色
    target_copy = o3d.geometry.PointCloud(target)
    source_copy = o3d.geometry.PointCloud(source)
    aligned_copy = o3d.geometry.PointCloud(aligned)

    # 目标：蓝色
    if not target_copy.has_colors():
        target_copy.paint_uniform_color([0, 0, 1])

    # 源点云：红色
    source_copy.paint_uniform_color([1, 0, 0])

    # 配准后：绿色
    aligned_copy.paint_uniform_color([0, 1, 0])

    print("  显示窗口:")
    print("  - 蓝色: 目标点云（标准位置）")
    print("  - 红色: 源点云（变换后，配准前）")
    print("  - 绿色: 配准后的点云")
    print("\n  按 'Q' 退出...")

    draw_geometries([target_copy, source_copy, aligned_copy])


def main():
    """主函数"""
    print("=" * 60)
    print("ProjectGeodesic - 配准演示")
    print("=" * 60)

    # 1. 创建标准工件（模板）
    print("\n" + "=" * 60)
    print("步骤1: 创建标准工件模板")
    print("=" * 60)

    template = create_synthetic_workpiece()

    # 2. 模拟随机放置的工件
    print("\n" + "=" * 60)
    print("步骤2: 模拟随机放置的工件")
    print("=" * 60)

    import random
    tx = random.uniform(-0.1, 0.1)
    ty = random.uniform(-0.1, 0.1)
    tz = random.uniform(-0.05, 0.05)
    rdeg = random.uniform(-45, 45)

    print(f"  随机变换: 平移=[{tx:.3f}, {ty:.3f}, {tz:.3f}], 旋转={rdeg:.1f}°")

    workpiece, T_ground_truth = transform_workpiece(
        template, [tx, ty, tz], 'z', rdeg
    )

    # 3. 预处理
    print("\n" + "=" * 60)
    print("步骤3: 点云预处理")
    print("=" * 60)

    processor = PointCloudProcessor(voxel_size=0.005)

    template_proc = processor.preprocess(template, enable_normals=True)
    workpiece_proc = processor.preprocess(workpiece, enable_normals=True)

    print(f"  模板: {len(template_proc.points)} 点")
    print(f"  工件: {len(workpiece_proc.points)} 点")

    # 4. PCA粗配准
    print("\n" + "=" * 60)
    print("步骤4: PCA粗配准")
    print("=" * 60)

    registration = Registration(voxel_size=0.005)

    T_coarse = registration.pca_alignment(workpiece_proc, template_proc)

    # 计算粗配准误差
    workpiece_coarse = workpiece.transform(T_coarse)
    coarse_points = np.asarray(workpiece_coarse.points)
    template_points = np.asarray(template_proc.points)

    from scipy.spatial import cKDTree
    tree = cKDTree(template_points)
    distances, _ = tree.query(coarse_points, k=1)
    coarse_error = np.mean(distances)

    print(f"  粗配准误差: {coarse_error*1000:.2f} mm")

    # 5. ICP精配准
    print("\n" + "=" * 60)
    print("步骤5: ICP精配准")
    print("=" * 60)

    result = registration.icp_registration(
        workpiece_proc,
        template_proc,
        init_transform=T_coarse,
        max_correspondence_distance=0.02,
        point_to_plane=True
    )

    print(f"  方法: {result.method}")
    print(f"  RMS误差: {result.rmse*1000:.3f} mm")
    print(f"  适应度: {result.fitness:.2%}")
    print(f"  迭代次数: {result.num_iterations}")

    # 6. 最终评估
    print("\n" + "=" * 60)
    print("步骤6: 配准结果评估")
    print("=" * 60)

    workpiece_aligned = workpiece.transform(result.transformation)
    aligned_points = np.asarray(workpiece_aligned.points)
    tree = cKDTree(template_points)
    distances, _ = tree.query(aligned_points, k=1)
    final_error = np.mean(distances)

    print(f"\n  粗配准误差: {coarse_error*1000:.2f} mm")
    print(f"  精配准误差: {final_error*1000:.3f} mm")
    print(f"  改善: {(1 - final_error/coarse_error)*100:.1f}%")

    # 判断是否成功
    if final_error < 0.002:  # 2mm
        print("\n  ✅ 配准成功！精度满足要求（<2mm）")
    elif final_error < 0.005:  # 5mm
        print("\n  ⚠️  配准一般，精度尚可（<5mm）")
    else:
        print("\n  ❌ 配准失败，精度不足")

    # 7. 可视化
    print("\n" + "=" * 60)
    print("步骤7: 可视化")
    print("=" * 60)

    visualize_result(template_proc, workpiece_proc, workpiece_aligned)

    print("\n✅ 演示完成！")
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""
Algorithm Unit Tests

测试配准算法和点云处理功能。
"""

import sys
import numpy as np
from pathlib import Path

# 添加模块路径
script_dir = Path(__file__).parent.parent
sys.path.insert(0, str(script_dir))

import open3d as o3d

from geodesic_perception.pointcloud_proc import PointCloudProcessor
from geodesic_perception.registration import Registration, transform_points


def create_test_pointcloud(shape='plane', noise_level=0.001):
    """创建测试点云"""
    if shape == 'plane':
        # 平面点云
        x = np.random.uniform(-0.3, 0.3, 5000)
        y = np.random.uniform(-0.2, 0.2, 5000)
        z = np.random.normal(0.5, noise_level, 5000)
        points = np.column_stack([x, y, z])

    elif shape == 'box':
        # 盒子点云
        points = []
        for _ in range(2000):
            # 随机选择面
            face = np.random.randint(0, 6)
            if face < 2:  # x = ±0.3
                x = 0.3 if face == 0 else -0.3
                y = np.random.uniform(-0.2, 0.2)
                z = np.random.uniform(0, 0.4)
            elif face < 4:  # y = ±0.2
                x = np.random.uniform(-0.3, 0.3)
                y = 0.2 if face == 2 else -0.2
                z = np.random.uniform(0, 0.4)
            else:  # z = 0 or 0.4
                x = np.random.uniform(-0.3, 0.3)
                y = np.random.uniform(-0.2, 0.2)
                z = 0.4 if face == 4 else 0
            points.append([x, y, z] + np.random.normal(0, noise_level, 3))
        points = np.array(points)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def transform_pointcloud(pcd, translation, rotation_deg):
    """变换点云"""
    # 创建变换矩阵
    T = np.eye(4)

    # 旋转
    theta = np.deg2rad(rotation_deg)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])
    T[:3, :3] = R

    # 平移
    T[:3, 3] = translation

    # 应用变换
    return pcd.transform(T)


def test_preprocessing():
    """测试点云预处理"""
    print("\n" + "=" * 60)
    print("测试: 点云预处理")
    print("=" * 60)

    processor = PointCloudProcessor(voxel_size=0.01)

    # 创建测试点云
    source = create_test_pointcloud('plane', noise_level=0.002)
    print(f"  原始点云: {len(source.points)} 点")

    # 测试降采样
    downsampled = processor.voxel_downsample(source)
    print(f"  ✅ 体素降采样: {len(downsampled.points)} 点")
    assert len(downsampled.points) < len(source.points), "降采样应该减少点数"

    # 测试去噪
    denoised = processor.remove_statistical_outliers(source)
    print(f"  ✅ 统计去噪: {len(denoised.points)} 点")

    # 测试完整预处理流程
    processed = processor.preprocess(source, enable_normals=True)
    print(f"  ✅ 完整预处理: {len(processed.points)} 点")
    assert processed.has_normals(), "应该有法向量"

    print("\n  ✅ 点云预处理测试通过")
    return True


def test_pca_registration():
    """测试PCA配准"""
    print("\n" + "=" * 60)
    print("测试: PCA粗配准")
    print("=" * 60)

    registration = Registration(voxel_size=0.01)

    # 创建源点云和目标点云
    target = create_test_pointcloud('plane')

    # 源点云：平移和旋转后
    source = transform_pointcloud(target, [0.1, 0.05, 0], 30)

    print(f"  目标点云: {len(target.points)} 点")
    print(f"  源点云: {len(source.points)} 点（已变换）")

    # PCA配准
    T_estimated = registration.pca_alignment(source, target)

    print(f"  ✅ PCA配准完成")
    print(f"  估计变换矩阵:\n{T_estimated}")

    # 评估配准误差
    source_aligned = source.transform(T_estimated)
    source_points = np.asarray(source_aligned.points)
    target_points = np.asarray(target.points)

    # 计算平均距离
    from scipy.spatial import cKDTree
    tree = cKDTree(target_points)
    distances, _ = tree.query(source_points, k=1)
    avg_error = np.mean(distances)

    print(f"  配准误差: {avg_error*1000:.2f} mm")

    if avg_error < 0.02:  # 20mm
        print(f"  ✅ PCA配准精度良好")
        return True
    else:
        print(f"  ⚠️  配准误差较大")
        return False


def test_icp_registration():
    """测试ICP配准"""
    print("\n" + "=" * 60)
    print("测试: ICP精配准")
    print("=" * 60)

    registration = Registration(voxel_size=0.005)

    # 创建点云
    target = create_test_pointcloud('box', noise_level=0.001)

    # 小幅变换
    source = transform_pointcloud(target, [0.02, 0.01, 0], 10)

    print(f"  目标点云: {len(target.points)} 点")
    print(f"  源点云: {len(source.points)} 点")

    # 预处理
    processor = PointCloudProcessor(voxel_size=0.005)
    source_proc = processor.preprocess(source, enable_normals=True)
    target_proc = processor.preprocess(target, enable_normals=True)

    # PCA初值
    T_coarse = registration.pca_alignment(source_proc, target_proc)
    print(f"  ✅ PCA粗配准完成")

    # ICP精配准
    result = registration.icp_registration(
        source_proc,
        target_proc,
        init_transform=T_coarse,
        max_correspondence_distance=0.02,
        point_to_plane=True
    )

    print(f"  ✅ ICP精配准完成")
    print(f"  方法: {result.method}")
    print(f"  RMS误差: {result.rmse*1000:.3f} mm")
    print(f"  适应度: {result.fitness:.2%}")
    print(f"  迭代次数: {result.num_iterations}")

    if result.rmse < 0.005:  # 5mm
        print(f"  ✅ ICP配准精度良好")
        return True
    else:
        print(f"  ⚠️  配准误差较大")
        return False


def test_template_manager():
    """测试模板管理器"""
    print("\n" + "=" * 60)
    print("测试: 模板管理器")
    print("=" * 60)

    from geodesic_perception.template_manager import TemplateManager

    # 创建临时目录
    import tempfile
    temp_dir = tempfile.mkdtemp()
    manager = TemplateManager(base_path=temp_dir)

    # 创建测试模板
    pcd = create_test_pointcloud('plane')
    template = manager.create_empty_template('test_template', 'Test Workpiece')

    # 添加孔位标注
    template = manager.add_hole_annotation(
        template, hole_id=1,
        position=[0.1, 0.2, 0.5],
        hole_type='corner'
    )
    template = manager.add_hole_annotation(
        template, hole_id=2,
        position=[-0.1, 0.2, 0.5],
        hole_type='edge'
    )

    print(f"  创建模板: {template.template_id}")
    print(f"  孔位数: {template.num_holes}")

    # 保存模板
    saved_path = manager.save_template(template, pcd)
    print(f"  ✅ 模板已保存: {saved_path}")

    # 加载模板
    loaded_template = manager.load_template('test_template')
    print(f"  ✅ 模板已加载")
    print(f"  孔位数: {loaded_template.num_holes}")

    assert loaded_template.num_holes == template.num_holes, "孔位数应该匹配"

    # 获取孔位坐标
    positions = manager.get_hole_positions(loaded_template)
    print(f"  孔位坐标形状: {positions.shape}")
    assert positions.shape == (2, 3), "应该有2个孔，每个3D坐标"

    print(f"\n  ✅ 模板管理器测试通过")

    # 清理
    import shutil
    shutil.rmtree(temp_dir)

    return True


def main():
    """主函数"""
    print("=" * 60)
    print("ProjectGeodesic - 算法单元测试")
    print("=" * 60)

    # 检查依赖
    try:
        import scipy
    except ImportError:
        print("\n❌ 缺少依赖: scipy")
        print("   安装方法: pip install scipy")
        return 1

    tests = [
        ("点云预处理", test_preprocessing),
        ("PCA配准", test_pca_registration),
        ("ICP配准", test_icp_registration),
        ("模板管理器", test_template_manager),
    ]

    results = []

    for name, test_func in tests:
        try:
            result = test_func()
            results.append((name, result))
        except Exception as e:
            print(f"\n  ❌ 测试失败: {e}")
            import traceback
            traceback.print_exc()
            results.append((name, False))

    # 总结
    print("\n" + "=" * 60)
    print("测试总结")
    print("=" * 60)

    passed = sum(1 for _, result in results if result)
    total = len(results)

    for name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"  {name}: {status}")

    print(f"\n通过率: {passed}/{total} ({passed/total*100:.1f}%)")

    if passed == total:
        print("\n🎉 所有测试通过！")
        return 0
    else:
        print("\n⚠️  部分测试失败")
        return 1


if __name__ == '__main__':
    sys.exit(main())

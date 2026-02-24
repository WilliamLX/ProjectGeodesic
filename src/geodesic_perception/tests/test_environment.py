#!/usr/bin/env python3
"""
Environment Verification Script

验证所有依赖项是否正确安装。
"""

import sys


def check_python_version():
    """检查Python版本"""
    print("🔍 检查Python版本...")
    version = sys.version_info
    if version.major >= 3 and version.minor >= 8:
        print(f"   ✅ Python {version.major}.{version.minor}.{version.micro}")
        return True
    else:
        print(f"   ❌ Python版本过低: {version.major}.{version.minor}.{version.micro}")
        print("      需要Python 3.8或更高版本")
        return False


def check_open3d():
    """检查Open3D安装"""
    print("\n🔍 检查Open3D...")
    try:
        import open3d as o3d
        print(f"   ✅ Open3D {o3d.__version__}")
        return True
    except ImportError as e:
        print(f"   ❌ Open3D未安装: {e}")
        print("      安装方法: pip install open3d")
        return False


def check_numpy():
    """检查NumPy安装"""
    print("\n🔍 检查NumPy...")
    try:
        import numpy as np
        print(f"   ✅ NumPy {np.__version__}")
        return True
    except ImportError as e:
        print(f"   ❌ NumPy未安装: {e}")
        print("      安装方法: pip install numpy")
        return False


def check_pyqt():
    """检查PyQt6安装"""
    print("\n🔍 检查PyQt6...")
    try:
        from PyQt6 import QtWidgets
        print(f"   ✅ PyQt6 已安装")
        return True
    except ImportError as e:
        print(f"   ❌ PyQt6未安装: {e}")
        print("      安装方法: pip install PyQt6")
        return False


def check_ros2():
    """检查ROS 2"""
    print("\n🔍 检查ROS 2...")
    try:
        import rclpy
        print(f"   ✅ ROS 2 Python客户端 (rclpy)")
        return True
    except ImportError as e:
        print(f"   ❌ ROS 2未安装: {e}")
        print("      安装方法: sudo apt install ros-humble-desktop")
        return False


def check_pcl():
    """检查PCL"""
    print("\n🔍 检查PCL ROS...")
    try:
        import pcl
        print(f"   ✅ Python PCL绑定")
        return True
    except ImportError:
        print("   ⚠️  Python PCL未安装 (可选)")
        print("      安装方法: pip install pcl")
        return True  # PCL不是必需的，Open3D已足够


def test_open3d_basic():
    """测试Open3D基本功能"""
    print("\n🔍 测试Open3D基本功能...")
    try:
        import open3d as o3d
        import numpy as np

        # 创建测试点云
        points = np.random.rand(100, 3)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        print(f"   ✅ 创建点云: {len(pcd.points)} 点")

        # 测试体素降采样
        downsampled = pcd.voxel_down_sample(voxel_size=0.1)
        print(f"   ✅ 体素降采样: {len(downsampled.points)} 点")

        return True
    except Exception as e:
        print(f"   ❌ Open3D功能测试失败: {e}")
        return False


def main():
    """主函数"""
    print("=" * 60)
    print("ProjectGeodesic - 环境验证")
    print("=" * 60)

    results = []

    # 运行所有检查
    results.append(("Python版本", check_python_version()))
    results.append(("Open3D", check_open3d()))
    results.append(("NumPy", check_numpy()))
    results.append(("PyQt6", check_pyqt()))
    results.append(("ROS 2", check_ros2()))
    results.append(("PCL (可选)", check_pcl()))
    results.append(("Open3D基本功能", test_open3d_basic()))

    # 总结
    print("\n" + "=" * 60)
    print("验证总结")
    print("=" * 60)

    passed = sum(1 for _, result in results if result)
    total = len(results)

    for name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"  {name}: {status}")

    print(f"\n通过率: {passed}/{total} ({passed/total*100:.1f}%)")

    if passed == total:
        print("\n🎉 所有检查通过！环境已就绪。")
        return 0
    else:
        print("\n⚠️  部分检查失败，请安装缺失的依赖。")
        return 1


if __name__ == '__main__':
    sys.exit(main())

# Geodesic Perception Module

## 概述

这是 ProjectGeodesic 的3D感知与配准模块，负责工件的点云采集、处理、配准和孔位检测。

## 模块组成

### 核心模块

| 模块 | 功能 | 文件 |
|------|------|------|
| **PointcloudProc** | 点云预处理（降采样、去噪、法向量估计） | `pointcloud_proc.py` |
| **Registration** | 点云配准（PCA、FPFH、ICP） | `registration.py` |
| **TemplateManager** | 模板管理（保存/加载标注） | `template_manager.py` |
| **GlobalAlignmentNode** | ROS 2配准节点 | `global_alignment_node.py` |
| **TeachingGUI** | 示教界面 | `teaching_gui.py` |
| **VisualizePointCloud** | 点云可视化工具 | `visualize_pointcloud.py` |

## 安装

### 1. 系统依赖

```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop
sudo apt install ros-humble-pcl-ros
sudo apt install ros-humble-vision-opencv

# Python依赖
pip install open3d PyQt6 numpy
```

### 2. 构建包

```bash
cd ProjectGeodesic/src/geodesic_perception

# 创建虚拟环境（推荐）
python3 -m venv venv
source venv/bin/activate

# 安装Python依赖
pip install -r requirements.txt

# 构建ROS 2包
cd ../../..
colcon build --packages-select geodesic_perception
source install/setup.bash
```

## 使用方法

### 1. 示教模式 - 创建模板

```bash
# 启动示教界面
ros2 run geodesic_perception teaching_gui
```

**操作流程：**
1. 点击 "Load Point Cloud" 加载标准工件的点云
2. 输入模板ID（如 `nio_front_roof_v1`）
3. 点击 "New Annotation Session"
4. 在3D视图中点击螺丝孔位置进行标注
5. 完成后点击 "Save Template"

### 2. 配准模式 - 自动定位

```bash
# 启动配准节点
ros2 launch geodesic_perception geodesic_perception_launch.py
```

**订阅话题：**
- `/camera/pointcloud2` - 输入点云

**发布话题：**
- `/perception/transformation_matrix` - 工件变换矩阵（4x4）
- `/perception/hole_positions` - 螺丝孔位置数组
- `/perception/alignment_status` - 配准状态

### 3. 可视化工具

```bash
# 查看点云文件
ros2 run geodesic_perception visualize_pointcloud /path/to/pointcloud.pcd

# 显示法向量
ros2 run geodesic_perception visualize_pointcloud /path/to/pointcloud.pcd --normals
```

### 4. 测试模式（无真实相机）

```bash
# 启动测试相机节点（发布合成点云）
ros2 run geodesic_perception test_camera_node.py
```

## 配置参数

配置文件：`config/alignment_params.yaml`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `voxel_size` | 0.005 | 体素滤波大小（米） |
| `max_correspondence_distance` | 0.02 | ICP对应点最大距离（米） |
| `coarse_method` | 'pca' | 粗配准方法（'pca' 或 'fpfh'） |
| `point_to_plane` | true | 使用Point-to-Plane ICP |
| `template_path` | 'data/templates' | 模板存储路径 |

## 数据格式

### 模板JSON格式

```json
{
  "template_id": "nio_front_roof_v1",
  "timestamp": "2025-02-24T10:30:00",
  "pointcloud_path": "data/templates/nio_front_roof_v1/pointcloud.pcd",
  "num_holes": 13,
  "holes": [
    {
      "id": 1,
      "position": [0.123, 0.456, 0.789],
      "normal": [0, 0, -1],
      "hole_type": "corner"
    }
  ],
  "metadata": {
    "workpiece_name": "NIO Front Roof Panel",
    "notes": "Standard workpiece"
  }
}
```

## ROS 2 接口

### 话题

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/camera/pointcloud2` | sensor_msgs/PointCloud2 | 订阅 | 相机点云输入 |
| `/perception/transformation_matrix` | std_msgs/Float64MultiArray | 发布 | 4x4变换矩阵 |
| `/perception/hole_positions` | geometry_msgs/PoseArray | 发布 | 螺丝孔位姿数组 |
| `/perception/alignment_status` | std_msgs/String | 发布 | 配准状态 |

### 服务

| 名称 | 类型 | 说明 |
|------|------|------|
| `/perception/trigger_alignment` | std_srvs/Trigger | 手动触发配准 |

## 性能指标

| 指标 | 目标值 | 实测值 |
|------|--------|--------|
| 配准时间 | < 600ms | - |
| 配准精度（RMS） | < 2mm | - |
| 配准成功率 | > 95% | - |

## 开发测试

```bash
# 运行单元测试（需要先创建）
pytest tests/

# 检查代码风格
flake8 geodesic_perception/
pylint geodesic_perception/
```

## 故障排除

### 问题1：Open3D导入失败
```bash
pip install --upgrade open3d
```

### 问题2：PyQt6显示问题
```bash
# 对于Wayland用户
export QT_QPA_PLATFORM=xcb
```

### 问题3：点云为空
检查相机驱动是否正常发布到 `/camera/pointcloud2`

## 下一步开发

- [ ] 集成真实MecMind相机驱动
- [ ] 添加FPFH特征配准选项
- [ ] 实现配准质量评估
- [ ] 添加多工件支持
- [ ] 优化实时性能

## 相关文档

- [完整开发计划](../docs/Perception_Development_Plan.md)
- [技术架构](../docs/LLM_BehaviorTree_Design.md)

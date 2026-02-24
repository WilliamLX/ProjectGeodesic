# 故障排除

本文档记录常见问题和解决方案。

---

## 📋 目录

- [环境问题](#环境问题)
- [依赖问题](#依赖问题)
- [运行时问题](#运行时问题)
- [性能问题](#性能问题)
- [平台特定问题](#平台特定问题)

---

## 环境问题

### ROS 2找不到

**症状**:
```
ModuleNotFoundError: No module named 'rclpy'
```

**解决方案**:

```bash
# Linux
source /opt/ros/humble/setup.bash

# 添加到.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Conda环境未激活

**症状**:
```
conda: command not found
```

**解决方案**:

```bash
# 初始化conda
~/miniconda3/bin/conda init bash
source ~/.bashrc

# 激活环境
conda activate geodesy
```

### Python版本不兼容

**症状**:
```
Open3D does not support Python 3.13
```

**解决方案**:

```bash
# 创建Python 3.11环境
conda create -n geodesy python=3.11 -y
conda activate geodesy
pip install open3d==0.19.0
```

---

## 依赖问题

### Open3D导入失败

**症状**:
```
ImportError: No module named 'open3d'
```

**解决方案**:

```bash
# pip安装
pip install open3d==0.19.0

# 或conda安装
conda install -c conda-forge open3d
```

### PyQt6显示问题

**症状**:
```
Qt platform plugin issues
```

**解决方案**:

```bash
# macOS
export QT_QPA_PLATFORM=xcb

# Linux
sudo apt install libx11-dev libxext-dev libgl1-mesa-dev
```

### SciPy版本冲突

**症状**:
```
ImportError: cannot import name 'cKDTree'
```

**解决方案**:

```bash
pip install --upgrade scipy
```

---

## 运行时问题

### 点云为空

**症状**:
```
RuntimeWarning: No points in point cloud
```

**原因**: 相机未正确连接或话题未发布

**解决方案**:

```bash
# 检查话题
ros2 topic list
ros2 topic echo /camera/pointcloud2

# 测试相机
ros2 run geodesic_perception test_camera_node.py
```

### 配准失败

**症状**:
```
ICP failed to converge
```

**原因**: 初值不好或点云质量差

**解决方案**:

```python
# 1. 检查点云质量
pcd = load_pointcloud("data.pcd")
print(f"Points: {len(pcd.points)}")

# 2. 预处理
processor = PointCloudProcessor(voxel_size=0.005)
pcd_clean = processor.preprocess(pcd)

# 3. 检查初值
T_coarse = registration.pca_alignment(source, target)

# 4. 调整参数
result = registration.icp_registration(
    source, target,
    max_correspondence_distance=0.02  # 增大阈值
)
```

### GUI无法启动

**症状**:
```
Could not connect to display
```

**解决方案**:

```bash
# 检查DISPLAY
echo $DISPLAY

# macOS
export QT_QPA_PLATFORM=xcb

# Docker
xhost +local:docker
docker run --env DISPLAY=$DISPLAY ...
```

---

## 性能问题

### 配准速度慢

**症状**: 配准时间 > 1秒

**解决方案**:

```python
# 1. 增大体素尺寸
processor = PointCloudProcessor(voxel_size=0.01)  # 原来是0.005

# 2. 减少ICP迭代次数
result = registration.icp_registration(
    source, target,
    max_correspondence_distance=0.02,
    max_iterations=30  # 减少迭代
)
```

### 内存占用高

**症状**: 进程内存 > 1GB

**解决方案**:

```python
# 1. 降采样
pcd_down = pcd.voxel_down_sample(voxel_size=0.01)

# 2. 清理中间结果
import gc
del temp_pointcloud
gc.collect()
```

---

## 平台特定问题

### macOS

#### ROS 2不可用

**说明**: ROS 2官方不支持macOS

**解决方案**: 跳过ROS 2功能，专注于算法测试

#### GUI显示问题

```bash
export QT_QPA_PLATFORM=xcb
```

### Ubuntu

#### 相机权限问题

```bash
# 添加用户到video组
sudo usermod -aG video $USER

# 重新登录
```

#### 实时调度

```bash
# 设置实时优先级
sudo chrt -f 50 python your_script.py
```

### Docker

#### X11转发失败

```bash
# 允许本地连接
xhost +local:docker

# 检查DISPLAY
echo $DISPLAY

# 使用host网络
docker run --network=host ...
```

---

## 调试技巧

### 启用详细日志

```python
import rclpy
from rclpy.logging import get_logger

logger = get_logger('geodesy')
logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

### 可视化点云

```bash
python scripts/visualize_pointcloud.py data.pcd --normals
```

### 检查话题

```bash
ros2 topic list
ros2 topic info /camera/pointcloud2
ros2 topic hz /camera/pointcloud2
```

### 查看TF树

```bash
ros2 run tf2_tools view_frames
```

---

## 获取帮助

### 日志位置

- ROS 2日志: `~/.ros/log/`
- Python日志: 控制台输出
- 系统日志: `journalctl -u geodesy`

### 提交问题

[GitHub Issues](https://github.com/WilliamLX/ProjectGeodesy/issues)

**信息模板**:

```markdown
## 问题描述
[简要描述问题]

## 环境信息
- OS: Ubuntu 22.04
- Python: 3.11.13
- Open3D: 0.19.0

## 复现步骤
1. ...
2. ...

## 错误信息
```
[粘贴错误输出]
```

## 已尝试的解决方案
- [ ] 尝试方案1
- [ ] 尝试方案2
```

---

**相关文档**:
- [快速开始](01_Getting_Started.md)
- [部署指南](06_Deployment_Guide.md)
- [测试指南](07_Testing_Guide.md)

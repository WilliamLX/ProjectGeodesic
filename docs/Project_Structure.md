# ProjectGeodesic 项目结构

## 目录结构

```
ProjectGeodesic/
├── README.md                          # 项目主README
├── Geodesic.md                        # 完整技术方案
├── README_PERCEPTION.md               # 3D感知模块使用指南
│
├── docs/                              # 技术文档
│   ├── Perception_Development_Plan.md # 3D感知开发计划
│   ├── LLM_BehaviorTree_Design.md     # LLM+行为树架构设计
│   └── Project_Structure.md           # 本文件
│
├── data/                              # 数据目录
│   ├── templates/                     # 工件模板存储
│   ├── pointclouds/                   # 原始点云数据
│   └── calibration/                   # 标定数据
│
└── src/                               # ROS 2 源码
    └── geodesic_perception/           # 3D感知包
        ├── geodesic_perception/       # Python模块
        │   ├── __init__.py
        │   ├── pointcloud_proc.py     # 点云预处理
        │   ├── registration.py         # 配准算法
        │   ├── template_manager.py     # 模板管理
        │   ├── global_alignment_node.py # ROS 2配准节点
        │   ├── teaching_gui.py         # 示教界面
        │   ├── visualize_pointcloud.py # 可视化工具
        │   └── test_camera_node.py     # 测试相机节点
        │
        ├── launch/                     # Launch文件
        │   ├── geodesic_perception_launch.py
        │   └── camera_test.launch.py
        │
        ├── config/                     # 配置文件
        │   └── alignment_params.yaml
        │
        ├── resource/                   # ROS 2资源标记
        ├── requirements.txt            # Python依赖
        ├── setup.py                    # Python包设置
        └── package.xml                 # ROS 2包描述
```

## 核心模块说明

### 1. 点云预处理模块 (pointcloud_proc.py)

**功能：**
- 体素降采样 (Voxel Downsampling)
- 统计离群点移除 (Statistical Outlier Removal)
- 法向量估计 (Normal Estimation)
- FPFH特征计算
- ROS与Open3D格式转换

**关键类：**
- `PointCloudProcessor`: 主要处理器类

**使用示例：**
```python
processor = PointCloudProcessor(voxel_size=0.005)
processed = processor.preprocess(pointcloud, enable_normals=True)
```

---

### 2. 配准算法模块 (registration.py)

**功能：**
- PCA粗配准 (快速初始化)
- FPFH特征配准 (更鲁棒但较慢)
- ICP精配准 (Point-to-Point / Point-to-Plane)
- 多尺度ICP (渐进式优化)
- Colored ICP (利用颜色信息)

**关键类：**
- `Registration`: 配准算法类
- `RegistrationResult`: 配准结果数据类

**使用示例：**
```python
registration = Registration(voxel_size=0.005)

# PCA粗配准
T_coarse = registration.pca_alignment(source, target)

# ICP精配准
result = registration.icp_registration(
    source, target,
    init_transform=T_coarse,
    point_to_plane=True
)
```

---

### 3. 模板管理模块 (template_manager.py)

**功能：**
- 保存/加载工件模板
- 孔位标注管理
- 点云数据存储
- 模板序列化（JSON格式）

**关键类：**
- `TemplateManager`: 模板管理器
- `WorkpieceTemplate`: 模板数据类
- `HoleAnnotation`: 孔位标注类
- `AnnotationSession`: 示教会话管理

**模板数据格式：**
```json
{
  "template_id": "nio_front_roof_v1",
  "num_holes": 13,
  "holes": [
    {"id": 1, "position": [x, y, z], "type": "corner"}
  ]
}
```

---

### 4. 全局配准节点 (global_alignment_node.py)

**功能：**
- ROS 2节点实现
- 订阅相机点云话题
- 实时配准处理
- 发布变换矩阵和孔位坐标

**话题接口：**

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/camera/pointcloud2` | PointCloud2 | 订阅 | 相机输入 |
| `/perception/hole_positions` | PoseArray | 发布 | 孔位坐标 |
| `/perception/alignment_status` | String | 发布 | 配准状态 |

**启动方式：**
```bash
ros2 launch geodesic_perception geodesic_perception_launch.py
```

---

### 5. 示教界面 (teaching_gui.py)

**功能：**
- PyQt6 GUI应用
- 点云可视化（Open3D）
- 交互式孔位标注
- 模板保存/加载

**启动方式：**
```bash
ros2 run geodesic_perception teaching_gui
```

**操作流程：**
1. 加载标准工件点云
2. 创建示教会话
3. 点击标注13个螺丝孔位置
4. 保存模板

---

### 6. 可视化工具 (visualize_pointcloud.py)

**功能：**
- 命令行点云查看器
- 支持法向量显示
- 支持降采样

**使用方式：**
```bash
# 基础查看
ros2 run geodesic_perception visualize_pointcloud data.pcd

# 显示法向量
ros2 run geodesic_perception visualize_pointcloud data.pcd --normals

# 降采样后查看
ros2 run geodesic_perception visualize_pointcloud data.pcd --voxel 0.01
```

---

### 7. 测试工具 (test_camera_node.py)

**功能：**
- 发布合成点云数据
- 用于无相机情况下的测试

**启动方式：**
```bash
ros2 run geodesic_perception test_camera_node.py
```

---

## 数据流

### 示教模式数据流
```
标准工件 → MecMind相机 → 点云采集 → 示教GUI → 人工标注 → 模板保存
```

### 配准模式数据流
```
工件 → 相机 → 点云 → 预处理 → PCA粗配准 → ICP精配准 → 变换矩阵 + 孔位坐标
```

---

## 配置参数

主要配置文件：`config/alignment_params.yaml`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| voxel_size | 0.005m | 体素滤波大小 |
| max_correspondence_distance | 0.02m | ICP对应距离阈值 |
| coarse_method | 'pca' | 粗配准方法 |
| point_to_plane | true | 使用Point-to-Plane ICP |

---

## 开发状态

### ✅ 已完成
- [x] 项目结构搭建
- [x] 3D感知核心算法实现
- [x] 示教界面开发
- [x] ROS 2节点集成
- [x] 配置和启动文件
- [x] 测试工具

### 🚧 待开发
- [ ] 真实相机驱动集成
- [ ] 视觉伺服模块
- [ ] 运动规划模块
- [ ] 力控模块
- [ ] LLM行为树集成

---

## 依赖关系

```
geodesic_perception/
├── Open3D >= 0.18.0      # 点云处理
├── PyQt6 >= 6.6.0        # GUI界面
├── numpy >= 1.24.0       # 数值计算
├── rclpy >= 4.0.0        # ROS 2 Python客户端
├── sensor_msgs >= 4.0.0  # 传感器消息类型
└── geometry_msgs >= 4.0.0 # 几何消息类型
```

---

## 扩展指南

### 添加新的配准算法

在 `registration.py` 的 `Registration` 类中添加新方法：

```python
def my_registration_method(self, source, target):
    # 实现你的配准算法
    T = np.eye(4)
    return RegistrationResult(T, rmse, fitness, iterations, "MyMethod")
```

### 添加新的ROS 2服务

在 `global_alignment_node.py` 中：

```python
self.create_service(
    MyService,
    '/perception/my_service',
    self.my_service_callback
)
```

### 自定义GUI功能

在 `teaching_gui.py` 中扩展 `TeachingGUI` 类。

---

## 性能基准

| 操作 | 目标 | 说明 |
|------|------|------|
| 点云预处理 | <100ms | 50K点 → 10K点 |
| PCA粗配准 | <50ms | 初始对齐 |
| ICP精配准 | <500ms | 收敛到2mm以内 |
| 总配准时间 | <600ms | 完整流程 |
| 配准精度 | <2mm RMS | 工件定位 |
| 配准成功率 | >95% | 正常工况 |

---

## 故障排除

### 导入错误
```bash
# 确保安装了所有依赖
pip install -r src/geodesic_perception/requirements.txt
```

### ROS 2构建错误
```bash
# 清理并重新构建
cd ProjectGeodesic
rm -rf build/ install/ log/
colcon build --packages-select geodesic_perception
```

### Open3D可视化问题
```bash
# 对于Wayland用户
export QT_QPA_PLATFORM=xcb
```

---

## 联系方式

- 项目维护者: WilliamLX
- 技术支持: 在GitHub Issues提交问题

---

**最后更新:** 2025-02-24

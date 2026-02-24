# 3D感知与配准模块开发计划

## 模块概述

负责工件的3D点云采集、处理、配准和孔位检测，输出工件的世界坐标系变换矩阵和所有螺丝孔的3D坐标。

---

## 核心功能分解

### 1. 示教模式 (Teaching Mode)
- 采集标准工件的3D点云模板
- 可视化点云并人工标注螺丝孔位置
- 保存模板数据和孔位标注

### 2. 全局配准 (Global Registration)
- 实时采集工件点云
- 预处理（降采样、去噪）
- 粗配准（PCA/FPFH）
- 精配准（ICP）
- 输出 4x4 变换矩阵 T

### 3. 孔位检测 (Hole Detection)
- 基于标注的模板孔位，应用变换矩阵
- 可选：视觉辅助的孔位验证

---

## 技术栈选型

| 功能 | 库/工具 | 说明 |
|------|---------|------|
| 点云处理 | **Open3D** + PCL | Open3D API更友好，PCL作为底层支持 |
| 配准算法 | Open3D.registration | ICP, Colored ICP, Point-to-Plane ICP |
| 可视化 | Open3D.visualization | 实时点云显示和交互 |
| 相机驱动 | MecMind SDK | 厂家提供的ROS 2驱动 |
| UI框架 | PyQt6 | 示教界面 |
| 数据存储 | Pickle / HDF5 | 点云模板和标注数据 |

**推荐 Open3D 的原因：**
- Python原生支持，开发效率高
- ICP配准性能优秀（C++内核）
- 可视化功能强大
- 与NumPy无缝集成

---

## 开发阶段

### Phase 1: 环境搭建与数据采集 (Week 1)

#### 目标
- 搭建开发环境
- 连接MecMind相机，成功采集点云
- 基础可视化

#### 任务清单
- [ ] 创建ROS 2工作空间和包结构
- [ ] 安装依赖：Open3D, PCL, PyQt6, MecMind SDK
- [ ] 测试相机连接和数据采集
- [ ] 实现点云采集节点 `/camera/get_pointcloud`
- [ ] 实现基础点云可视化脚本

#### 交付物
```bash
# 测试相机连接
ros2 launch geodesic_perception camera_test.launch.py

# 查看实时点云
ros2 run geodesic_perception visualize_pointcloud
```

#### 数据流
```
MecMind Camera → ROS 2 Topic (/camera/pointcloud2) → Open3D Visualization
```

---

### Phase 2: 点云预处理与粗配准 (Week 2)

#### 目标
- 实现点云预处理流程（体素滤波、去噪）
- 实现基于PCA的粗配准
- 验证配准精度

#### 任务清单
- [ ] 实现体素降采样 (Voxel Downsampling)
- [ ] 实现统计去噪 (Statistical Outlier Removal)
- [ ] 实现PCA粗配准算法
- [ ] 实现FPFH特征提取和匹配（可选，作为PCA的备选）
- [ ] 配准精度评估工具

#### 核心算法
```python
# 伪代码
def preprocess_pointcloud(pointcloud, voxel_size=0.005):
    """点云预处理"""
    # 1. 体素降采样
    downsampled = pointcloud.voxel_down_sample(voxel_size)

    # 2. 统计去噪
    denoised, _ = downsampled.remove_statistical_outlier(
        nb_neighbors=20, std_ratio=2.0
    )

    return denoised

def coarse_registration(source, target, method="PCA"):
    """粗配准"""
    if method == "PCA":
        # 基于主成分分析的初始对齐
        return pca_alignment(source, target)
    elif method == "FPFH":
        # 基于特征的配准
        return fpfh_registration(source, target)
```

#### 性能指标
- 预处理时间： < 100ms
- 粗配准误差： < 20mm
- 点云规模： ~50,000 点 → ~10,000 点（降采样后）

---

### Phase 3: ICP精配准 (Week 3)

#### 目标
- 实现ICP点云配准
- 实现Point-to-Plane ICP（利用法向量）
- 优化配准参数，达到毫米级精度

#### 任务清单
- [ ] 实现标准ICP配准
- [ ] 实现Point-to-Plane ICP
- [ ] 法向量估计优化
- [ ] 配准质量评估（收敛性、RMS误差）
- [ ] 参数调优（最大对应距离、迭代次数）

#### 核心算法
```python
def fine_registration(source, target, transformation_init):
    """ICP精配准"""

    # 1. 估计法向量
    source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30
    ))
    target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30
    ))

    # 2. Point-to-Plane ICP
    reg_p2l = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance=0.02,
        init=transformation_init,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=50
        )
    )

    return reg_p2l.transformation, reg_p2l.inlier_rmse
```

#### 性能指标
- 精配准时间： < 500ms
- 配准精度（RMS）： < 2mm
- 成功率： > 95%

---

### Phase 4: 示教界面开发 (Week 3-4)

#### 目标
- 开发PyQt示教界面
- 支持点云可视化和交互标注
- 保存/加载模板数据

#### 界面功能模块
```
┌─────────────────────────────────────────────────────────┐
│  ProjectGeodesic - 示教模式                            │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  [采集点云] [加载模板] [保存模板] [清空标注]           │
│                                                         │
│  ┌─────────────────────┐    ┌─────────────────────┐   │
│  │                     │    │  孔位列表           │   │
│  │   3D 点云可视化      │    │  ─────────────      │   │
│  │   (Open3D Widget)   │    │  1. (x, y, z)       │   │
│  │                     │    │  2. (x, y, z)       │   │
│  │   [可交互点击标注]  │    │  3. (x, y, z)       │   │
│  │                     │    │  ...                │   │
│  └─────────────────────┘    └─────────────────────┘   │
│                                                         │
│  当前标注孔位: 5 / 13                                    │
│  坐标: [123.4, 56.7, 89.0]                             │
│                                                         │
│  [导出模板数据]                                         │
└─────────────────────────────────────────────────────────┘
```

#### 任务清单
- [ ] PyQt主窗口框架
- [ ] 集成Open3D可视化控件
- [ ] 实现鼠标点击标注功能
- [ ] 孔位列表显示和编辑
- [ ] 模板数据保存/加载（Pickle/HDF5）
- [ ] 标注导出为JSON/YAML

#### 数据格式
```json
{
  "template_id": "nio_front_roof_v1",
  "timestamp": "2025-02-24T10:30:00",
  "pointcloud_path": "data/templates/nio_front_roof.pcd",
  "num_holes": 13,
  "holes": [
    {
      "id": 1,
      "position": [100.234, 200.456, 50.123],
      "normal": [0, 0, -1],
      "type": "corner"
    },
    // ... 共13个孔
  ],
  "metadata": {
    "camera_serial": "MM-Pro-001",
    "voxel_size": 0.005,
    "notes": "标准工件，无遮挡"
  }
}
```

---

### Phase 5: 实时配准节点 (Week 4)

#### 目标
- 整合所有模块，实现完整的配准流程
- 发布变换矩阵和孔位坐标到ROS话题
- 集成测试

#### 任务清单
- [ ] 实现 `global_alignment_node`
- [ ] 整合预处理 → 粗配准 → 精配准流程
- [ ] 发布 `/perception/transformation_matrix` (Float64Array 4x4)
- [ ] 发布 `/perception/hole_positions` (PoseArray)
- [ ] 添加配准状态反馈
- [ ] 参数服务器配置（config/alignment_params.yaml）

#### ROS 2 节点接口

```python
# 订阅话题
/camera/pointcloud2     # SensorMsgs/PointCloud2

# 发布话题
/perception/transformation_matrix    # Float64Array[16] (列优先4x4矩阵)
/perception/hole_positions          # GeometryMsgs/PoseArray
/perception/alignment_status        # String (IDLE, ALIGNING, SUCCESS, FAILED)
/perception/aligned_pointcloud      # SensorMsgs/PointCloud2 (可视化)

# 服务
/perception/load_template           # Load saved template
/perception/save_template           # Save current as template

# 参数
~voxel_size: 0.005
~max_correspondence_distance: 0.02
~template_path: "data/templates/default.json"
```

#### 节点实现框架
```python
class GlobalAlignmentNode(Node):
    def __init__(self):
        super().__init__('global_alignment_node')

        # 状态
        self.template = None
        self.state = "IDLE"

        # 订阅者
        self.create_subscription(
            PointCloud2,
            '/camera/pointcloud2',
            self.pointcloud_callback,
            10
        )

        # 发布者
        self.transform_pub = self.create_publisher(
            Float64MultiArray,
            '/perception/transformation_matrix',
            10
        )
        self.holes_pub = self.create_publisher(
            PoseArray,
            '/perception/hole_positions',
            10
        )

        # 服务
        self.create_service(
            LoadTemplate,
            '/perception/load_template',
            self.load_template_callback
        )

    def pointcloud_callback(self, msg):
        """处理输入点云，执行配准"""
        if self.template is None:
            self.get_logger().warn("No template loaded")
            return

        self.state = "ALIGNING"

        # 1. 转换ROS消息为Open3D格式
        source = self.ros_to_open3d(msg)

        # 2. 预处理
        source_processed = self.preprocess(source)

        # 3. 粗配准
        T_coarse = self.coarse_registration(source_processed, self.template.pointcloud)

        # 4. 精配准
        T_final, rmse = self.fine_registration(
            source_processed,
            self.template.pointcloud,
            T_coarse
        )

        # 5. 变换模板孔位到当前工件坐标系
        current_holes = self.transform_holes(self.template.holes, T_final)

        # 6. 发布结果
        self.publish_results(T_final, current_holes, rmse)

        self.state = "SUCCESS"
```

---

## 测试计划

### 单元测试

| 模块 | 测试内容 | 预期结果 |
|------|---------|---------|
| 点云预处理 | 不同体素大小的降采样 | 点数减少，形状保持 |
| PCA配准 | 30°旋转 + 100mm平移 | 角度误差 < 5° |
| ICP配准 | 标准件 vs 稍微偏移件 | RMS < 2mm |
| 法向量估计 | 平面点云 | 法向量偏差 < 5° |

### 集成测试

| 场景 | 测试方法 | 成功标准 |
|------|---------|---------|
| 完整配准流程 | 标准件 → 采集 → 配准 | 变换矩阵正确 |
| 不同光照条件 | 调整光照强度测试 | 配准成功率 > 90% |
| 工件位置随机 | 5个不同位置/姿态 | 误差均 < 5mm |
| 边缘情况 | 无工件 / 多个工件 | 正确识别并报错 |

### 性能测试

```
测试场景: 工件随机放置，连续配准50次

指标:
- 平均配准时间: _____ ms (目标: < 600ms)
- 配准成功率: _____ % (目标: > 95%)
- RMS误差分布: [μ=_____, σ=_____] mm
- 内存占用: _____ MB
```

---

## 风险与应对

| 风险 | 影响 | 概率 | 应对措施 |
|------|------|------|---------|
| 点云质量差（反光、黑色件） | 配准失败 | 高 | 多角度采集融合；补光优化 |
| ICP陷入局部最优 | 精度不足 | 中 | 好的初值（PCA/FPFH）；多尺度ICP |
| 实时性不满足 | 系统延迟 | 低 | GPU加速；降采样优化 |
| 相机标定误差 | 累积误差 | 中 | 定期手眼标定；误差补偿 |

---

## 数据流图

```
┌──────────────┐
│ MecMind Pro  │
│   Camera     │
└──────┬───────┘
       │ PointCloud2
       ↓
┌───────────────────────────────────────┐
│   global_alignment_node               │
│  ┌─────────────────────────────────┐  │
│  │ 1. ROS → Open3D 转换            │  │
│  └─────────────────────────────────┘  │
│  ┌─────────────────────────────────┐  │
│  │ 2. 预处理 (体素滤波 + 去噪)     │  │
│  └─────────────────────────────────┘  │
│  ┌─────────────────────────────────┐  │
│  │ 3. 粗配准 (PCA)                 │  │
│  └─────────────────────────────────┘  │
│  ┌─────────────────────────────────┐  │
│  │ 4. 精配准 (ICP Point-to-Plane)  │  │
│  └─────────────────────────────────┘  │
│  ┌─────────────────────────────────┐  │
│  │ 5. 变换孔位坐标                 │  │
│  └─────────────────────────────────┘  │
└───────────┬───────────────────────────┘
            │
            ├→ /perception/transformation_matrix (4x4)
            ├→ /perception/hole_positions (PoseArray)
            └→ /perception/alignment_status (String)
```

---

## 开发环境要求

### 硬件
- NVIDIA Jetson AGX Orin 或 RTX 5090 工控机
- MecMind Pro 3D相机

### 软件
```bash
# ROS 2
ROS 2 Humble / Jazzy

# Python依赖
pip install open3d==0.18.0
pip install PyQt6==6.6.1
pip install numpy==1.24.0
pip install pyyaml==6.0.1

# ROS依赖
sudo apt install ros-humble-pcl-ros
sudo apt install ros-humble-vision-opencv
```

### 项目结构（建议）
```
ProjectGeodesic/
├── src/
│   ├── geodesic_perception/          # 感知包
│   │   ├── geodesic_perception/
│   │   │   ├── __init__.py
│   │   │   ├── pointcloud_proc.py   # 点云预处理
│   │   │   ├── registration.py       # 配准算法
│   │   │   ├── global_alignment_node.py
│   │   │   └── teaching_gui.py       # 示教界面
│   │   ├── launch/
│   │   ├── config/
│   │   ├── test/
│   │   └── package.xml
│   │
│   └── geodesic_bringup/             # 启动配置
│
├── data/
│   ├── templates/                    # 模板数据
│   │   └── nio_front_roof.json
│   ├── pointclouds/                  # 原始点云
│   └── calibration/                  # 标定数据
│
└── docs/
    ├── Perception_Development_Plan.md
    └── LLM_BehaviorTree_Design.md
```

---

## 里程碑

| 周次 | 里程碑 | 交付物 |
|------|--------|--------|
| Week 1 | 相机集成完成 | 能采集并可视化点云 |
| Week 2 | 粗配准验证 | PCA配准误差 < 20mm |
| Week 3 | ICP配准完成 | 配准精度 < 2mm |
| Week 4 | 示教界面完成 | 能标注并保存13个孔位 |
| Week 5 | 集成测试通过 | 全流程配准成功率 > 95% |

---

## 后续工作

完成本模块后，进入：

**Phase 6: 与视觉伺服集成**
- 配准输出作为视觉伺服的初值
- 全局定位 + 局部精对准的完整链路

**Phase 7: 与行为树集成**
- `GlobalAlignment` 作为行为树第一个节点
- 配准成功后才执行后续拧紧动作

---

## 参考资料

- [Open3D Tutorials - Point Cloud Registration](http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html)
- [PCL ICP Tutorial](https://pointclouds.org/documentation/tutorials/iterative_closest_point.html)
- [ROS 2 Perception Tutorials](https://docs.ros.org/en/humble/Tutorials/Perception/)
- [MecMind Camera Documentation](http://www.mecmind.com/manual) (假设)

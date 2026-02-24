# ProjectGeodesic

> **固定基座智能拧紧机器人系统** - 基于视觉伺服与力控的单臂自动化装配解决方案

## 📋 项目概述

ProjectGeodesic 是一套面向工厂制造的智能机器人拧紧系统，通过**3D视觉定位**、**视觉伺服**和**柔顺力控**实现高精度的自动化螺丝拧紧作业。

### 核心特点

- **自适应视觉定位** - 通过ICP点云配准 + IBVS视觉伺服，适应工件位置的随机性
- **双算法路线** - 支持传统图像处理（Blob Detection + 霍夫圆）与深度学习（YOLOv8）切换
- **AI驱动的任务规划** - 大模型动态生成行为树，根据工件情况智能调整拧紧顺序
- **柔顺入孔控制** - 螺旋搜索 + 阻抗控制，保护塑料件螺纹
- **ROS 2架构** - 模块化设计，易于扩展和维护

## 🎯 应用场景

- 汽车制造（如蔚来前顶板装配，13个螺丝孔）
- 3C产品装配
- 其他需要高精度拧紧的制造场景

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────┐
│   Decision Layer (LLM + Behavior Tree)          │  ← 大模型生成任务序列
├─────────────────────────────────────────────────┤
│   Skill Layer (动作原语)                         │
│   - GlobalAlignment  (全局粗定位)               │
│   - ApproachMove     (进场轨迹规划)             │
│   - VisualServo      (视觉伺服精对准)           │
│   - ScrewIn          (螺旋入孔 + 拧紧)          │
├─────────────────────────────────────────────────┤
│   Perception Layer (感知)                       │
│   - PointCloud Registration (ICP)               │
│   - Hole Detection (YOLOv8 / Blob + Hough)      │
│   - Pose Refinement (IBVS)                      │
├─────────────────────────────────────────────────┤
│   Control Layer (控制)                          │
│   - Spiral Search Entry (螺旋搜索入孔)          │
│   - Impedance Control (阻抗控制拧紧)            │
├─────────────────────────────────────────────────┤
│   Hardware Interface (ROS 2)                    │
│   - Robot Arm (Tianji Marvin)                   │
│   - Cameras (MecMind Pro + Nano)                │
│   - Screwdriver (Atlas Copco)                   │
└─────────────────────────────────────────────────┘
```

## 🛠️ 技术栈

### 硬件
| 组件 | 型号 | 用途 |
|------|------|------|
| 计算核心 | Jetson AGX Orin + RTX 5090 工控机 | 边缘推理 + 视觉处理 |
| 机械臂 | 天机 Marvin 上半身 | 执行拧紧动作 |
| 3D相机 | MecMind Pro (头顶) + Nano (手腕) | 粗定位 + 精对准 |
| 拧紧枪 | Atlas Copco 直柄电枪 | 螺丝拧紧 |
| 送钉机 | 自动送钉系统 | 供钉 |

### 软件栈
- **机器人框架**: ROS 2 Humble
- **运动规划**: MoveIt 2
- **视觉感知**:
  - 点云处理: PCL, Open3D
  - 图像检测: OpenCV, YOLOv8
  - 点云配准: ICP, FPFH
- **控制算法**:
  - 视觉伺服: IBVS (Image-Based Visual Servo)
  - 力控: 阻抗控制
- **任务编排**: Behavior Tree CPP + LLM集成
- **仿真**: NVIDIA Isaac Sim
- **UI**: PyQt6 (示教界面)

## 🚀 工作流程

1. **示教阶段** (一次配置)
   - 拍摄标准件3D点云模板
   - 人工标注所有螺丝孔位置

2. **全局粗定位** (每工件一次)
   - 体素滤波 → PCA粗配准 → ICP精配准
   - 输出工件位姿变换矩阵 T

3. **智能任务规划** (每工件一次)
   - LLM分析当前工件状态（遮挡、可达性等）
   - 动态生成最优拧紧顺序的行为树

4. **逐孔拧紧** (循环)
   - 生成进场位姿 → MoveIt轨迹规划
   - IBVS视觉伺服收敛 (<0.5mm误差)
   - 螺旋搜索入孔 + 阻抗控制拧紧
   - 异常处理：失败则退出进入下一孔

## 📅 开发进度

### ✅ 已完成
- [x] 项目初始化与GitHub仓库设置
- [x] 技术方案设计（Geodesic.md）
- [x] 3D感知模块框架实现
  - 点云预处理（体素滤波、去噪）
  - PCA粗配准 + ICP精配准
  - 模板管理系统
  - 示教GUI界面
  - ROS 2配准节点

### 🚧 进行中
- [ ] 3D感知模块集成测试
- [ ] 相机驱动集成
- [ ] 视觉伺服模块开发

### 📅 计划中 (4个月)
- **Month 1**: 仿真环境搭建 + 硬件集成
- **Month 2**: 视觉算法开发（ICP + IBVS）
- **Month 3**: 力控调试 + 拧紧工艺
- **Month 4**: LLM行为树集成 + 系统测试

详见 [Geodesic.md](./Geodesic.md) 完整技术方案

## 🚀 快速开始

### 安装依赖

```bash
# 系统依赖
sudo apt install ros-humble-desktop ros-humble-pcl-ros

# Python依赖
pip install open3d PyQt6 numpy
```

### 构建项目

```bash
# 克隆仓库
cd ProjectGeodesic

# 构建ROS 2包
colcon build --packages-select geodesic_perception
source install/setup.bash
```

### 运行示教界面

```bash
ros2 run geodesic_perception teaching_gui
```

### 启动配准节点

```bash
ros2 launch geodesic_perception geodesic_perception_launch.py
```

## 📖 文档

- [技术方案详解](./Geodesic.md) - 完整算法流程和开发计划
- [3D感知模块开发计划](./docs/Perception_Development_Plan.md) - 点云配准模块详细设计
- [LLM+行为树架构设计](./docs/LLM_BehaviorTree_Design.md) - AI任务规划系统
- [感知模块使用说明](./README_PERCEPTION.md) - 3D感知模块快速开始

## 👥 团队

8-10人团队，包括机器人、视觉、算法工程师

## 📄 License

TBD

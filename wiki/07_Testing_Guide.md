# 测试指南

本文档说明如何测试和验证ProjectGeodesy系统。

---

## 📋 测试概览

### 测试类型

| 类型 | 目的 | 工具 |
|------|------|------|
| **环境测试** | 验证依赖安装 | `test_environment.py` |
| **单元测试** | 测试单个模块 | `test_algorithms.py` |
| **集成测试** | 测试模块交互 | ROS 2测试 |
| **性能测试** | 验证性能指标 | Benchmark |
| **演示测试** | 可视化验证 | `demo_registration.py` |

---

## Phase 1: 环境验证

### 运行测试

```bash
cd ~/ProjectGeodesy/src/geodesic_perception
python tests/test_environment.py
```

### 测试项

| 组件 | 检查内容 | 预期结果 |
|------|---------|---------|
| Python | 版本 ≥ 3.8 | ✅ 3.11.13 |
| Open3D | 可导入 | ✅ 0.19.0 |
| NumPy | 可导入 | ✅ 2.4.2 |
| PyQt6 | 可导入 | ✅ 6.10.2 |
| ROS 2 | 可导入 | ⚠️ Linux only |
| Open3D功能 | 创建点云 | ✅ 100点 |

### 通过标准

- **Ubuntu**: 6/7 通过 (85.7%)
- **macOS**: 5/7 通过 (ROS 2跳过)

---

## Phase 2: 算法单元测试

### 运行测试

```bash
python tests/test_algorithms.py
```

### 测试模块

#### 1. 点云预处理

**测试内容**:
- 体素降采样（5000点 → ~2800点）
- 统计去噪
- 法向量估计

**通过标准**: ✅ 无错误

#### 2. PCA粗配准

**测试场景**:
- 平面点云（5000点）
- 随机变换（平移 + 旋转）
- PCA对齐

**通过标准**:
- 配准误差 < 20mm
- ✅ 实际: ~0mm (理想情况)

#### 3. ICP精配准

**测试场景**:
- 盒子点云（2000点）
- 小幅变换
- PCA初值 + ICP精配准

**通过标准**:
- RMS误差 < 5mm
- ✅ 实际: < 2mm

#### 4. 模板管理器

**测试内容**:
- 创建模板
- 添加孔位标注
- 保存/加载
- 序列化验证

**通过标准**: ✅ 无错误

### 测试结果

```
通过率: 4/4 (100.0%)

✅ 点云预处理
✅ PCA配准
✅ ICP配准
✅ 模板管理器
```

---

## Phase 3: 配准演示

### 运行演示

```bash
python scripts/demo_registration.py
```

### 演示流程

```
步骤1: 创建标准工件模板
  → 8000点，13个螺丝孔

步骤2: 模拟随机放置
  → 平移[9cm, 9cm, -2.8cm], 旋转11.6°

步骤3: 点云预处理
  → 8000点 → 6131点

步骤4: PCA粗配准
  → 误差: 0.85mm ✅

步骤5: ICP精配准
  → RMS: 0.855mm ✅
  → 目标: <2mm

步骤6: 可视化
  → 蓝色: 目标
  → 红色: 源点云
  → 绿色: 配准结果
```

### 性能指标

| 指标 | 目标 | 实测 | 状态 |
|------|------|------|------|
| 粗配准误差 | <20mm | 0.85mm | ✅ |
| 精配准误差 | <2mm | 0.855mm | ✅ |
| 总配准时间 | <600ms | ~270ms | ✅ |

---

## Phase 4: ROS 2节点测试

### 启动测试节点

```bash
# 终端1: 测试相机（发布合成数据）
ros2 run geodesic_perception test_camera_node.py

# 终端2: 配准节点
ros2 launch geodesic_perception geodesic_perception.launch.py

# 终端3: 查看话题
ros2 topic echo /perception/alignment_status
ros2 topic echo /perception/hole_positions
```

### 验证项

- ✅ 相机数据发布
- ✅ 配准节点接收
- ✅ 变换矩阵发布
- ✅ 孔位坐标发布
- ✅ 状态更新

---

## Phase 5: 示教界面测试

### 启动GUI

```bash
conda activate geodesy
cd ~/ProjectGeodesy/src/geodesic_perception
python teaching_gui.py
```

### 测试流程

1. **加载点云**
   - File → Load Point Cloud
   - 选择测试数据

2. **创建会话**
   - 输入模板ID
   - 点击"New Annotation Session"

3. **标注孔位**
   - 在3D视图中点击
   - 添加13个孔位

4. **保存模板**
   - 点击"Save Template"
   - 验证JSON格式

---

## 性能基准

### 点云处理性能

| 操作 | 点数 | 时间 | 目标 |
|------|------|------|------|
| 降采样 | 8000 | ~30ms | <50ms |
| 去噪 | 8000 | ~20ms | <50ms |
| 预处理总计 | 8000 | ~50ms | <100ms ✅ |

### 配准性能

| 方法 | 输入 | 时间 | 精度 |
|------|------|------|------|
| PCA | 6000点 | ~20ms | <20mm |
| ICP | 6000点 | ~200ms | <2mm |
| **总计** | 6000点 | **~270ms** | **<2mm** ✅ |

### 内存占用

| 组件 | 内存 |
|------|------|
| 点云数据 | ~50MB |
| Open3D | ~100MB |
| ROS 2节点 | ~200MB |
| **总计** | **~350MB** |

---

## 持续集成

### GitHub Actions

```yaml
name: Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-22.04
    container: ros:humble

    steps:
      - uses: actions/checkout@v2

      - name: Install dependencies
        run: |
          apt-get update
          apt-get install -y python3-pip
          pip3 install open3d PyQt6 scipy

      - name: Run tests
        run: |
          cd src/geodesic_perception
          python3 tests/test_algorithms.py
```

---

## 测试数据

### 合成数据

位置: `data/sample_data/`

| 文件 | 说明 | 大小 |
|------|------|------|
| `synthetic_plane.pcd` | 平面+5孔 | ~236KB |
| `synthetic_plane_noisy.pcd` | 带噪声版本 | ~236KB |

### 真实数据（待添加）

- 标准工件模板
- 各种姿态工件
- 边界情况

---

## 故障排除

### 测试失败

**问题**: Open3D导入失败

```bash
pip install open3d==0.19.0
```

**问题**: ROS 2未找到

```bash
source /opt/ros/humble/setup.bash
```

**问题**: 配准精度不达标

```bash
# 检查点云质量
python visualize_pointcloud.py data.pcd --normals
```

---

## 报告结果

### 测试报告模板

```markdown
## 测试报告 - [日期]

### 环境
- OS: Ubuntu 22.04
- Python: 3.11.13
- Open3D: 0.19.0

### 结果
- 环境测试: 6/7 (85.7%)
- 算法测试: 4/4 (100%)
- 配准演示: ✅ 通过

### 性能
- 配准精度: 0.85mm (目标: <2mm)
- 配准时间: 270ms (目标: <600ms)

### 结论
✅ 所有测试通过
```

---

**相关文档**:
- [快速开始](01_Getting_Started.md)
- [部署指南](06_Deployment_Guide.md)
- [故障排除](08_Troubleshooting.md)

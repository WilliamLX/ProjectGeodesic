# 快速开始指南

本文档帮助您在5分钟内快速启动ProjectGeodesy项目。

---

## 🎯 选择您的环境

| 环境 | 支持程度 | 推荐场景 |
|------|---------|---------|
| **Ubuntu 20.04/22.04** | ✅ 完整支持 | 生产开发 |
| **macOS** | ⚠️ 部分支持 | 算法开发 |
| **Docker** | ✅ 完整支持 | 跨平台测试 |
| **WSL2** | ✅ 完整支持 | Windows用户 |

---

## 🚀 方式1: Linux一键安装（推荐）

适用于：Ubuntu 20.04/22.04

### 步骤1: 下载并运行安装脚本

```bash
# 下载脚本
wget https://raw.githubusercontent.com/WilliamLX/ProjectGeodesy/master/scripts/install_linux.sh

# 运行（选择选项1：完整安装）
chmod +x install_linux.sh
./install_linux.sh
```

### 步骤2: 激活环境

```bash
# 添加到shell配置
echo "source ~/ProjectGeodesy/setup_env.sh" >> ~/.bashrc
source ~/.bashrc
```

### 步骤3: 验证安装

```bash
cd ~/ProjectGeodesy/src/geodesic_perception
python tests/test_algorithms.py
```

**预期输出**: 🎉 所有测试通过！

---

## 🐳 方式2: Docker部署（跨平台）

适用于：所有支持Docker的系统

### 步骤1: 构建镜像

```bash
git clone https://github.com/WilliamLX/ProjectGeodesy.git
cd ProjectGeodesy
docker build -f scripts/Dockerfile -t projectgeodesy:dev .
```

### 步骤2: 运行容器

```bash
# 无GUI模式（算法测试）
docker run -it --volume $(pwd):/root/ProjectGeodesy projectgeodesy:dev

# 带GUI模式（需要X11）
xhost +local:docker
docker run -it \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $(pwd):/root/ProjectGeodesy \
  projectgeodesy:dev
```

---

## 💻 方式3: 手动安装

适用于：需要自定义配置的场景

### 系统要求

| 组件 | 版本要求 |
|------|---------|
| Python | 3.11 |
| ROS 2 | Humble (Ubuntu) |
| Open3D | 0.19.0 |
| 内存 | ≥8GB |
| 存储 | ≥20GB |

### 步骤1: 安装ROS 2 (仅Linux)

```bash
# 添加ROS 2 apt仓库
sudo apt update && sudo apt install -y \
  software-properties-common
sudo add-apt-repository universe -y
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop python3-pip
```

### 步骤2: 安装Miniconda

```bash
# 下载
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
  -O miniconda.sh

# 安装
bash miniconda.sh -b -p $HOME/miniconda3

# 初始化
$HOME/miniconda3/bin/conda init bash
source ~/.bashrc
```

### 步骤3: 创建Python环境

```bash
# 创建环境
conda create -n geodesy python=3.11 -y
conda activate geodesy

# 安装依赖
pip install open3d==0.19.0 PyQt6 scipy
```

### 步骤4: 克隆并构建项目

```bash
# 克隆项目
git clone https://github.com/WilliamLX/ProjectGeodesy.git ~/ProjectGeodesy
cd ~/ProjectGeodesy

# 构建ROS 2包
source /opt/ros/humble/setup.bash
cd src/geodesic_perception
colcon build --packages-select geodesic_perception --symlink-install
source install/setup.bash
```

---

## 🧪 验证安装

### 环境测试

```bash
cd ~/ProjectGeodesy/src/geodesy_perception
python tests/test_environment.py
```

**预期输出**:
```
通过率: 6/7 (85.7%)
✅ Python 3.11.13
✅ Open3D 0.19.0
✅ NumPy
✅ PyQt6
✅ Open3D基本功能
```

### 算法测试

```bash
python tests/test_algorithms.py
```

**预期输出**:
```
通过率: 4/4 (100.0%)
✅ 点云预处理
✅ PCA配准
✅ ICP配准
✅ 模板管理器
```

---

## 🎯 第一个示例：配准演示

### 运行演示

```bash
cd ~/ProjectGeodesy/src/geodesic_perception
python scripts/demo_registration.py
```

**输出**:
```
步骤1: 创建标准工件模板
  ✅ 创建工件点云: 8000 点
  ✅ 添加了 13 个螺丝孔

步骤5: ICP精配准
  RMS误差: 0.855 mm
  ✅ 配准成功！精度满足要求（<2mm）
```

---

## 📖 下一步

安装完成后，建议按以下顺序阅读：

1. **[系统架构](02_Architecture_Overview.md)** - 了解整体设计
2. **[3D感知模块](04_Perception_Module.md)** - 学习核心功能
3. **[测试指南](07_Testing_Guide.md)** - 运行更多测试
4. **[部署指南](06_Deployment_Guide.md)** - 深入配置

---

## 🔧 环境管理

### 激活环境

每次打开新终端：

```bash
# 方式1: 使用自动脚本
source ~/ProjectGeodesy/setup_env.sh

# 方式2: 手动激活
conda activate geodesy
source /opt/ros/humble/setup.bash  # Linux
cd ~/ProjectGeodesy/src/geodesic_perception
source install/setup.bash
```

### 退出环境

```bash
conda deactivate
```

### 更新环境

```bash
# 更新代码
cd ~/ProjectGeodesy
git pull origin master

# 更新依赖
conda activate geodesy
pip install --upgrade open3d PyQt6 scipy

# 重新构建
cd src/geodesic_perception
colcon build --packages-select geodesic_perception --symlink-install
```

---

## 🆘 常见问题

### Q: ROS 2找不到？
```bash
source /opt/ros/humble/setup.bash
```

### Q: Conda环境未激活？
```bash
conda init bash
source ~/.bashrc
conda activate geodesy
```

### Q: Open3D导入失败？
```bash
pip install open3d==0.19.0
```

### Q: GUI无法显示（macOS）？
```bash
export QT_QPA_PLATFORM=xcb
```

更多问题请参考：[故障排除](08_Troubleshooting.md)

---

## 📞 获取帮助

- 📖 [完整文档](00_Wiki_Index.md)
- 🐛 [问题反馈](https://github.com/WilliamLX/ProjectGeodesy/issues)
- 💬 [讨论区](https://github.com/WilliamLX/ProjectGeodesy/discussions)

---

**下一步**: [系统架构](02_Architecture_Overview.md) →

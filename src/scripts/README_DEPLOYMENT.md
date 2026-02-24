# ProjectGeodesy 部署指南

## 快速部署（推荐）

### Linux系统（Ubuntu 20.04/22.04）

#### 方式1: 自动化脚本（推荐）

```bash
# 下载并运行部署脚本
cd /tmp
wget https://raw.githubusercontent.com/WilliamLX/ProjectGeodesy/master/scripts/install_linux.sh
chmod +x install_linux.sh
./install_linux.sh
```

**选择安装选项**:
- 选项1: 完整安装（ROS 2 + Python + 项目）
- 选项2: 仅Python环境（用于算法测试）
- 选项3: 仅ROS 2

#### 方式2: 手动安装

```bash
# 1. 安装ROS 2 Humble
sudo apt update
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop -y

# 2. 安装Miniconda
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh
bash miniconda.sh -b -p $HOME/miniconda3
source $HOME/miniconda3/bin/activate
conda init

# 3. 创建Python环境
conda create -n geodesy python=3.11 -y
conda activate geodesy
pip install open3d PyQt6 scipy

# 4. 克隆项目
git clone https://github.com/WilliamLX/ProjectGeodesic.git ~/ProjectGeodesy
cd ~/ProjectGeodesy/src/geodesic_perception

# 5. 运行测试
python tests/test_algorithms.py
```

---

## Docker部署（跨平台）

### 构建镜像

```bash
cd ProjectGeodesy
docker build -f scripts/Dockerfile -t projectgeodesy:dev .
```

### 运行容器

#### 无GUI模式（算法测试）

```bash
docker run -it \
  --volume $(pwd):/root/ProjectGeodesy \
  projectgeodesy:dev
```

#### 带GUI模式（示教界面）

```bash
# 允许X11连接
xhost +local:docker

# 运行容器
docker run -it \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $(pwd):/root/ProjectGeodesy \
  projectgeodesy:dev

# 在容器中运行GUI
conda activate geodesy
cd /root/ProjectGeodesy/src/geodesy_perception
python teaching_gui.py
```

---

## 环境配置

### Python依赖

**requirements.txt**:
```
# 核心依赖
open3d==0.19.0
PyQt6==6.10.2
scipy==1.17.1
numpy>=1.24.0

# ROS 2 (需要ROS 2环境)
rclpy>=4.0.0
sensor-msgs>=4.0.0
geometry-msgs>=4.0.0

# 工具
pyyaml>=6.0.1
matplotlib>=3.0
```

**安装方式**:
```bash
# 使用pip
pip install -r requirements.txt

# 使用conda
conda env create -f environment.yml
```

### Conda环境文件

**environment.yml**:
```yaml
name: geodesy
channels:
  - conda-forge
  - defaults
dependencies:
  - python=3.11
  - pip
  - numpy
  - scipy
  - matplotlib
  - pyyaml
  - pip:
    - open3d==0.19.0
    - PyQt6==6.10.2
```

---

## 系统要求

### 硬件

| 组件 | 最低要求 | 推荐配置 |
|------|---------|---------|
| CPU | 4核 | 8核+ |
| 内存 | 8GB | 16GB+ |
| 存储 | 20GB | 50GB+ SSD |
| GPU | 无 | NVIDIA GPU (可选) |

### 软件

- **操作系统**: Ubuntu 20.04/22.04 LTS
- **Python**: 3.11 (通过conda管理)
- **ROS 2**: Humble (Ubuntu) 或其他发行版
- **Docker**: 20.10+ (可选)

---

## 验证安装

### 1. 激活环境

```bash
# 如果使用自动化脚本
source ~/ProjectGeodesy/setup_env.sh

# 或者手动激活
conda activate geodesy
source /opt/ros/humble/setup.bash  # Linux
cd ~/ProjectGeodesy/src/geodesic_perception
source install/setup.bash
```

### 2. 运行测试

```bash
# 环境验证
python tests/test_environment.py

# 算法测试
python tests/test_algorithms.py

# 配准演示
python scripts/demo_registration.py
```

### 3. 测试GUI（仅Linux/带X11的Docker）

```bash
python teaching_gui.py
```

---

## 故障排除

### 问题1: ROS 2找不到

```bash
# 检查安装
dpkg -l | grep ros-humble

# 手动source
source /opt/ros/humble/setup.bash

# 添加到.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 问题2: Conda环境未激活

```bash
# 初始化conda
conda init bash
source ~/.bashrc

# 激活环境
conda activate geodesy
```

### 问题3: Open3D导入失败

```bash
# 重新安装
pip uninstall open3d
pip install open3d==0.19.0

# 或使用conda
conda install -c conda-forge open3d
```

### 问题4: GUI无法显示

```bash
# 检查DISPLAY
echo $DISPLAY

# X11转发（SSH）
ssh -X user@hostname

# Docker
xhost +local:docker
```

---

## 平台特定说明

### Ubuntu/Debian
- ✅ 完整支持（ROS 2 + GUI）
- 推荐用于开发

### macOS
- ⚠️ ROS 2不可用
- 可以运行算法测试
- GUI支持正常

### Windows
- 使用WSL2 + Ubuntu
- 或使用Docker

### Docker
- ✅ 跨平台
- 需要X11转发用于GUI
- 适合CI/CD

---

## 持续集成

### GitHub Actions示例

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
          pip3 install open3d PyQt6 scipy numpy

      - name: Run tests
        run: |
          cd src/geodesic_perception
          python3 tests/test_algorithms.py
```

---

## 更新环境

### 更新项目代码

```bash
cd ~/ProjectGeodesy
git pull origin master
```

### 更新Python依赖

```bash
conda activate geodesy
pip install --upgrade open3d PyQt6 scipy
```

### 更新ROS 2

```bash
sudo apt update
sudo apt upgrade --with-new-pkgs
```

---

## 卸载

### 完全卸载

```bash
# 删除项目目录
rm -rf ~/ProjectGeodesy

# 删除conda环境
conda env remove -n geodesy

# 卸载ROS 2
sudo apt remove ros-humble-* && sudo apt autoremove
```

---

## 获取帮助

- 📖 [完整文档](../README_PERCEPTION.md)
- 📋 [开发计划](../docs/Perception_Development_Plan.md)
- 🐛 [问题报告](https://github.com/WilliamLX/ProjectGeodesy/issues)

---

**最后更新**: 2025-02-24

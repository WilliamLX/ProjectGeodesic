# 部署指南

本文档详细说明如何在各种平台上部署ProjectGeodesy系统。

---

## 📋 目录

- [快速部署](#快速部署)
- [Linux部署](#linux部署)
- [Docker部署](#docker部署)
- [环境配置](#环境配置)
- [故障排除](#故障排除)

---

## 快速部署

### 一键安装（Ubuntu）

```bash
wget https://raw.githubusercontent.com/WilliamLX/ProjectGeodesy/master/scripts/install_linux.sh
chmod +x install_linux.sh
./install_linux.sh
```

选择 **选项1: 完整安装**

---

## Linux部署

### 系统要求

- **操作系统**: Ubuntu 20.04/22.04 LTS
- **内存**: ≥8GB (推荐16GB)
- **存储**: ≥50GB SSD
- **网络**: 稳定的互联网连接

### 自动化安装

详见：[scripts/install_linux.sh](../scripts/install_linux.sh)

**功能**:
- ✅ 安装ROS 2 Humble
- ✅ 安装Miniconda
- ✅ 创建Python 3.11环境
- ✅ 安装所有依赖
- ✅ 克隆并构建项目
- ✅ 运行测试验证

### 手动安装

#### 1. 安装ROS 2 Humble

```bash
sudo apt update
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop -y
```

#### 2. 安装Miniconda

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh
bash miniconda.sh -b -p $HOME/miniconda3
$HOME/miniconda3/bin/conda init bash
source ~/.bashrc
```

#### 3. 创建Python环境

```bash
conda create -n geodesy python=3.11 -y
conda activate geodesy
pip install open3d==0.19.0 PyQt6 scipy
```

#### 4. 克隆项目

```bash
git clone https://github.com/WilliamLX/ProjectGeodesy.git ~/ProjectGeodesy
cd ~/ProjectGeodesy/src/geodesic_perception
```

#### 5. 构建工作空间

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select geodesic_perception --symlink-install
source install/setup.bash
```

---

## Docker部署

### 构建镜像

```bash
cd ProjectGeodesy
docker build -f scripts/Dockerfile -t projectgeodesy:dev .
```

### 运行容器

#### 无GUI（算法测试）

```bash
docker run -it \
  --volume $(pwd):/root/ProjectGeodesy \
  projectgeodesy:dev
```

#### 带GUI（示教界面）

```bash
# 允许X11连接
xhost +local:docker

# 运行容器
docker run -it \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $(pwd):/root/ProjectGeodesy \
  projectgeodesy:dev
```

---

## 环境配置

### 激活脚本

**位置**: `~/ProjectGeodesy/setup_env.sh`

```bash
#!/bin/bash
# ROS 2
source /opt/ros/humble/setup.bash

# Conda
conda activate geodesy

# 工作空间
source ~/ProjectGeodesy/src/geodesic_perception/install/setup.bash

echo "✅ ProjectGeodesy 环境已激活"
```

**添加到shell**:

```bash
echo "source ~/ProjectGeodesy/setup_env.sh" >> ~/.bashrc
```

### Python依赖

**requirements.txt**:

```
open3d==0.19.0
PyQt6==6.10.2
scipy>=1.17.0
numpy>=1.24.0
scikit-learn>=1.0.0
```

**environment.yml**:

```yaml
name: geodesy
channels:
  - conda-forge
dependencies:
  - python=3.11
  - pip
  - numpy
  - scipy
  - pip:
    - open3d==0.19.0
    - PyQt6==6.10.2
```

---

## 验证安装

### 1. 环境测试

```bash
cd ~/ProjectGeodesy/src/geodesy_perception
python tests/test_environment.py
```

**预期**: 85.7% 通过率（macOS上ROS 2跳过）

### 2. 算法测试

```bash
python tests/test_algorithms.py
```

**预期**: 100% 通过率

### 3. 配准演示

```bash
python scripts/demo_registration.py
```

**预期**: 配准精度 <2mm

---

## 平台特定说明

### Ubuntu/Debian
- ✅ 完整支持
- ROS 2原生支持
- GUI完整支持

### macOS
- ⚠️ ROS 2不可用
- Python功能正常
- GUI支持正常

### Windows (WSL2)
- ✅ 完整支持
- 需要配置X11
- 推荐WSL2 + Ubuntu

---

## 更新部署

### 更新代码

```bash
cd ~/ProjectGeodesy
git pull origin master
```

### 更新依赖

```bash
conda activate geodesy
pip install --upgrade open3d PyQt6 scipy
```

### 重新构建

```bash
cd src/geodesic_perception
colcon build --packages-select geodesic_perception --symlink-install
```

---

## 故障排除

详见：[故障排除](08_Troubleshooting.md)

### 常见问题

**ROS 2找不到**
```bash
source /opt/ros/humble/setup.bash
```

**Conda环境未激活**
```bash
conda init bash
source ~/.bashrc
conda activate geodesy
```

**GUI无法显示**
```bash
export DISPLAY=:0
# 或 macOS
export QT_QPA_PLATFORM=xcb
```

---

## 卸载

### 完全卸载

```bash
# 删除项目
rm -rf ~/ProjectGeodesy

# 删除conda环境
conda env remove -n geodesy

# 卸载ROS 2
sudo apt remove ros-humble-* && sudo apt autoremove
```

---

## 生产部署

### 系统服务

创建systemd服务自动启动：

```bash
sudo vim /etc/systemd/system/geodesy.service
```

```ini
[Unit]
Description=ProjectGeodesy Service
After=network.target

[Service]
Type=simple
User=your_user
WorkingDirectory=/home/your_user/ProjectGeodesy
Environment="PATH=/home/your_user/miniconda3/bin:/usr/local/bin:/usr/bin:/bin"
ExecStart=/home/your_user/miniconda3/envs/geodesy/bin/python main.py
Restart=always

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable geodesy
sudo systemctl start geodesy
```

---

**相关文档**:
- [快速开始](01_Getting_Started.md)
- [故障排除](08_Troubleshooting.md)
- [测试指南](07_Testing_Guide.md)

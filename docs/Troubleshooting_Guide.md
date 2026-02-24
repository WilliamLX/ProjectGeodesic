# 环境配置故障排除

## 当前问题

**系统信息:**
- 架构: arm64 (Apple Silicon)
- Python版本: 3.13.5
- 问题: Open3D暂不支持Python 3.13

## 解决方案

### 方案1: 使用conda环境（推荐）

```bash
# 安装miniconda（如果尚未安装）
# 下载: https://docs.conda.io/en/latest/miniconda.html

# 创建Python 3.11环境
conda create -n geodesic python=3.11 -y
conda activate geodesic

# 安装Open3D
pip install open3d

# 安装其他依赖
pip install PyQt6 scipy numpy

# 如果需要ROS 2
sudo apt install ros-humble-desktop
```

### 方案2: 使用pyenv

```bash
# 安装pyenv
brew install pyenv

# 安装Python 3.11
pyenv install 3.11.10

# 在项目目录设置本地Python版本
cd /Users/server1/Work/ProjectNexus
pyenv local 3.11.10

# 安装依赖
pip install open3d PyQt6 scipy
```

### 方案3: Docker容器

```dockerfile
FROM ros:humble

RUN pip install open3d PyQt6 scipy

# 挂载项目目录
# docker run -v $(pwd):/workspace -it geodesic_dev
```

### 方案4: 暂时跳过Open3D测试

对于今天，可以先进行：
1. 代码审查
2. 文档阅读
3. 算法理解
4. 准备测试数据集

## 推荐步骤

1. **立即行动**: 创建conda环境
   ```bash
   conda create -n geodesic python=3.11 -y
   conda activate geodesy
   pip install open3d PyQt6 scipy
   ```

2. **验证环境**:
   ```bash
   cd src/geodesic_perception
   python tests/test_environment.py
   ```

3. **运行测试**:
   ```bash
   bash scripts/quickstart.sh
   ```

## 如果没有conda

macOS上安装并使用conda:

```bash
# 下载Miniconda for macOS ARM64
curl -O https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-arm64.sh
bash Miniconda3-latest-MacOSX-arm64.sh

# 初始化shell
conda init zsh  # 或 bash
source ~/.zshrc

# 创建环境
conda create -n geodesic python=3.11
conda activate geodesic
```

## 联系

如遇到其他问题，请记录:
- Python版本: `python --version`
- pip版本: `pip --version`
- 系统版本: `sw_vers`
- 完整错误信息

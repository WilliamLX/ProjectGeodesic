#!/bin/bash
################################################################################
# ProjectGeodesic - Linux Deployment Script
#
# 用途: 在Linux系统上自动部署开发环境
# 支持: Ubuntu 20.04/22.04, ROS 2 Humble
################################################################################

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印函数
print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# 检查Linux发行版
check_distro() {
    print_info "检查系统发行版..."

    if [ -f /etc/os-release ]; then
        . /etc/os-release
        OS=$ID
        OS_VERSION=$VERSION_ID
        print_info "检测到系统: $OS $OS_VERSION"
    else
        print_error "无法检测系统发行版"
        exit 1
    fi

    if [[ "$OS" != "ubuntu" ]] && [[ "$OS" != "debian" ]]; then
        print_warning "此脚本主要针对Ubuntu/Debian系统"
        read -p "是否继续? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
}

# 检查并安装基础依赖
install_base_packages() {
    print_info "安装基础依赖包..."

    sudo apt-get update

    # 基础工具
    sudo apt-get install -y \
        build-essential \
        cmake \
        git \
        wget \
        curl \
        vim \
        htop \
        tree \
        python3-pip \
        python3-venv

    print_success "基础依赖安装完成"
}

# 安装ROS 2 Humble
install_ros2() {
    if command -v ros2 &> /dev/null; then
        print_warning "ROS 2已安装，跳过"
        return
    fi

    print_info "安装ROS 2 Humble..."

    # 设置locale
    sudo apt update && sudo apt install locales -y
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # 添加ROS 2 apt仓库
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # 安装ROS 2
    sudo apt update
    sudo apt install ros-humble-desktop -y

    # 开发工具
    sudo apt install -y \
        ros-humble-ros-base \
        ros-humble-pcl-ros \
        ros-humble-vision-opencv \
        ros-humble-rviz2 \
        ros-humble-rqt-common-plugins

    # ROS 2开发工具
    sudo apt install -y \
        python3-colcon-common-extensions \
        python3-rosdep

    # 初始化rosdep
    sudo rosdep init
    rosdep update

    # 设置环境
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi

    print_success "ROS 2 Humble安装完成"
    print_warning "请运行 'source ~/.bashrc' 或重新登录以使ROS 2生效"
}

# 安装Miniconda
install_miniconda() {
    if [ -d "$HOME/miniconda3" ]; then
        print_warning "Miniconda已安装，跳过"
        return
    fi

    print_info "安装Miniconda..."

    # 下载Miniconda
    cd /tmp
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh

    # 静默安装
    bash miniconda.sh -b -p $HOME/miniconda3

    # 初始化conda
    $HOME/miniconda3/bin/conda init bash

    # 清理
    rm miniconda.sh

    print_success "Miniconda安装完成"
    print_warning "请运行 'source ~/.bashrc' 或重新登录以使conda生效"
}

# 创建Python环境
create_python_env() {
    print_info "创建Python环境..."

    # 检查conda是否可用
    if [ -f "$HOME/miniconda3/bin/conda" ]; then
        CONDA="$HOME/miniconda3/bin/conda"
    elif command -v conda &> /dev/null; then
        CONDA="conda"
    else
        print_error "未找到conda，请先安装Miniconda"
        exit 1
    fi

    # 创建环境
    print_info "创建conda环境: geodesy (Python 3.11)"
    $CONDA create -n geodesy python=3.11 -y

    # 激活环境并安装依赖
    print_info "安装Python依赖..."
    $HOME/miniconda3/envs/geodesy/bin/pip install --upgrade pip

    # 核心依赖
    $HOME/miniconda3/envs/geodesy/bin/pip install \
        open3d==0.19.0 \
        PyQt6==6.10.2 \
        scipy==1.17.1 \
        numpy

    print_success "Python环境创建完成"
}

# 克隆项目仓库
clone_project() {
    PROJECT_DIR="$HOME/ProjectGeodesic"

    if [ -d "$PROJECT_DIR" ]; then
        print_warning "项目目录已存在: $PROJECT_DIR"
        read -p "是否更新项目? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            cd $PROJECT_DIR
            git pull origin master
        fi
    else
        print_info "克隆项目仓库..."
        git clone https://github.com/WilliamLX/ProjectGeodesic.git $PROJECT_DIR
    fi

    print_success "项目准备完成: $PROJECT_DIR"
}

# 构建ROS 2工作空间
build_workspace() {
    PROJECT_DIR="$HOME/ProjectGeodesic"

    if [ ! -d "$PROJECT_DIR" ]; then
        print_error "项目目录不存在: $PROJECT_DIR"
        exit 1
    fi

    print_info "构建ROS 2工作空间..."

    cd $PROJECT_DIR

    # 激活ROS 2
    source /opt/ros/humble/setup.bash

    # 构建感知包
    print_info "构建geodesic_perception包..."
    cd $PROJECT_DIR/src/geodesic_perception
    colcon build --packages-select geodesic_perception --symlink-install

    # 源工作空间
    source $PROJECT_DIR/src/geodesic_perception/install/setup.bash

    print_success "工作空间构建完成"
}

# 运行测试
run_tests() {
    print_info "运行环境验证测试..."

    PROJECT_DIR="$HOME/ProjectGeodesic"
    PYTHON="$HOME/miniconda3/envs/geodesy/bin/python"

    cd $PROJECT_DIR/src/geodesic_perception

    # 环境测试
    print_info "测试1: 环境依赖验证"
    $PYTHON tests/test_environment.py
    if [ $? -eq 0 ]; then
        print_success "环境验证通过"
    else
        print_warning "部分依赖未安装（可能正常）"
    fi

    # 算法测试
    print_info "测试2: 算法单元测试"
    $PYTHON tests/test_algorithms.py
    if [ $? -eq 0 ]; then
        print_success "算法测试通过"
    else
        print_error "算法测试失败"
        return 1
    fi

    print_success "所有测试完成"
}

# 生成激活脚本
generate_setup_script() {
    cat > $HOME/ProjectGeodesy/setup_env.sh << 'EOF'
#!/bin/bash
# ProjectGeodesic 环境激活脚本

# ROS 2
source /opt/ros/humble/setup.bash

# Conda
if [ -f "$HOME/miniconda3/bin/activate" ]; then
    source "$HOME/miniconda3/bin/activate" geodesy
fi

# 工作空间
if [ -f "$HOME/ProjectGeodesy/src/geodesic_perception/install/setup.bash" ]; then
    source "$HOME/ProjectGeodesy/src/geodesic_perception/install/setup.bash"
fi

echo "✅ ProjectGeodesy 环境已激活"
echo "   ROS 2: /opt/ros/humble"
echo "   Python: geodesy (Python 3.11)"
echo "   工作空间: $HOME/ProjectGeodesy"
EOF

    chmod +x $HOME/ProjectGeodesy/setup_env.sh

    print_success "环境激活脚本已创建: ~/ProjectGeodesy/setup_env.sh"
}

# 打印使用说明
print_usage() {
    cat << EOF

╔══════════════════════════════════════════════════════════════════╗
║                  ProjectGeodesic 部署完成                        ║
╚══════════════════════════════════════════════════════════════════╝

📁 项目位置: $HOME/ProjectGeodesy
🐍 Python环境: geodesy (Python 3.11)
🤖 ROS 2版本: Humble

🚀 快速开始:

   1. 激活环境（每次打开新终端）:
      source ~/ProjectGeodesy/setup_env.sh

   2. 运行测试:
      cd ~/ProjectGeodesy/src/geodesy_perception
      python tests/test_algorithms.py

   3. 启动示教界面:
      cd ~/ProjectGeodesy/src/geodesy_perception
      python teaching_gui.py

   4. 查看点云:
      cd ~/ProjectGeodesy/src/geodesy_perception
      python visualize_pointcloud.py <file.pcd>

📖 文档:
   - README_PERCEPTION.md (使用说明)
   - docs/Perception_Development_Plan.md (开发计划)
   - docs/PlanA_Test_Report.md (测试报告)

🔧 环境管理:

   # 激活conda环境
   conda activate geodesy

   # 退出环境
   conda deactivate

   # 更新代码
   cd ~/ProjectGeodesy
   git pull origin master

💡 提示: 将 'source ~/ProjectGeodesy/setup_env.sh' 添加到 ~/.bashrc
   以自动激活环境

EOF
}

################################################################################
# 主函数
################################################################################

main() {
    echo "============================================================"
    echo "  ProjectGeodesic - Linux 自动部署脚本"
    echo "============================================================"
    echo ""

    # 检查是否为root
    if [ "$EUID" -eq 0 ]; then
        print_error "请不要使用root运行此脚本"
        exit 1
    fi

    # 询问安装选项
    echo "请选择安装选项:"
    echo "  1) 完整安装 (ROS 2 + Conda + 项目)"
    echo "  2) 仅安装Python环境 (跳过ROS 2)"
    echo "  3) 仅安装ROS 2"
    echo "  4) 退出"
    read -p "请输入选项 (1-4): " choice

    case $choice in
        1)
            check_distro
            install_base_packages
            install_ros2
            install_miniconda
            create_python_env
            clone_project
            build_workspace
            run_tests
            generate_setup_script
            print_usage
            ;;
        2)
            install_miniconda
            create_python_env
            clone_project
            run_tests
            print_warning "未安装ROS 2，部分功能将不可用"
            ;;
        3)
            check_distro
            install_base_packages
            install_ros2
            print_success "ROS 2安装完成"
            ;;
        4)
            print_info "退出"
            exit 0
            ;;
        *)
            print_error "无效选项"
            exit 1
            ;;
    esac

    print_success "部署完成！"
}

# 运行主函数
main

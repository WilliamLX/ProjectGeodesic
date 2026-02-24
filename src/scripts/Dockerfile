# ProjectGeodesic Development Environment
# 基于 ROS 2 Humble 的 Docker 镜像

FROM ros:humble

# 设置工作目录
WORKDIR /root/ProjectGeodesy

# 安装系统依赖
RUN apt-get update && apt-get install -y \
    # Python 包管理
    python3-pip \
    python3-venv \
    # 构建工具
    build-essential \
    cmake \
    git \
    wget \
    curl \
    # GUI相关（如果需要X11转发）
    libx11-dev \
    libxext-dev \
    libgl1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# 安装Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p /opt/miniconda3 && \
    rm /tmp/miniconda.sh

# 添加conda到PATH
ENV PATH="/opt/miniconda3/bin:${PATH}"

# 创建conda环境
RUN conda create -n geodesy python=3.11 -y

# 激活环境并安装Python依赖
SHELL ["conda", "run", "-n", "geodesy", "/bin/bash", "-c"]

RUN pip install --upgrade pip && \
    pip install \
    open3d==0.19.0 \
    PyQt6==6.10.2 \
    scipy==1.17.1 \
    numpy

# 克隆项目（如果需要）或者挂载卷
# RUN git clone https://github.com/WilliamLX/ProjectGeodesic.git /root/ProjectGeodesic

# 复制项目文件（使用Docker上下文）
# COPY . /root/ProjectGeodesy

# 设置工作目录
WORKDIR /root/ProjectGeodesy

# 构建ROS 2包
RUN source /opt/ros/humble/setup.bash && \
    if [ -d src/geodesic_perception ]; then \
        cd src/geodesic_perception && \
        colcon build --packages-select geodesic_perception --symlink-install; \
    fi

# 激活脚本
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'conda activate geodesy' >> /root/.bashrc

# 默认命令
CMD ["/bin/bash"]

# 使用说明:
#
# 构建镜像:
#   docker build -t projectgeodesy:dev .
#
# 运行容器（带X11转发，用于GUI）:
#   xhost +local:docker
#   docker run -it \
#     --env DISPLAY=$DISPLAY \
#     --volume /tmp/.X11-unix:/tmp/.X11-unix \
#     --volume $(pwd):/root/ProjectGeodesy \
#     projectgeodesy:dev
#
# 运行容器（无GUI）:
#   docker run -it \
#     --volume $(pwd):/root/ProjectGeodesy \
#     projectgeodesy:dev

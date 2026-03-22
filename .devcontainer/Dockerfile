# 使用 Ubuntu 24.04 对应的 ROS 2 Jazzy 镜像
FROM osrf/ros:jazzy-desktop

# 设置环境变量，避免安装过程中的交互弹窗
ENV DEBIAN_FRONTEND=noninteractive

# 【必须保留】允许 pip 在系统全局安装包
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# 1. 替换 Ubuntu 系统源 (Ubuntu 24.04 专用)
RUN sed -i 's/http:\/\/archive.ubuntu.com\/ubuntu\//https:\/\/mirrors.tuna.tsinghua.edu.cn\/ubuntu\//g' /etc/apt/sources.list.d/ubuntu.sources && \
    sed -i 's/http:\/\/security.ubuntu.com\/ubuntu\//https:\/\/mirrors.tuna.tsinghua.edu.cn\/ubuntu\//g' /etc/apt/sources.list.d/ubuntu.sources

# 2. 替换 ROS 2 源为清华镜像站
RUN sed -i 's/packages.ros.org/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list.d/ros2.list || true

# 3. 安装系统基础工具和图形库依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    vim \
    wget \
    curl \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# 4. 安装 ROS 2 Jazzy 仿真相关功能包
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# 5. 配置 pip 镜像源并直接安装 MuJoCo
# 【修正：去掉了 pip3 install --upgrade pip 这一行】
RUN pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple && \
    pip3 install mujoco dm_control

# 6. 设置工作目录
WORKDIR /workspace

# 7. 渲染与环境配置
ENV MUJOCO_GL=glfw
# 自动 source 环境
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# 默认启动指令
CMD ["bash"]

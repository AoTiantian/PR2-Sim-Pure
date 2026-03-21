FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG APT_MIRROR=mirrors.tuna.tsinghua.edu.cn
ARG ROS_MIRROR=mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu

SHELL ["/bin/bash", "-lc"]

# 1) Faster mirror sources for Ubuntu apt
RUN set -eux; \
    if [ -f /etc/apt/sources.list ]; then \
      sed -i "s|http://archive.ubuntu.com/ubuntu/|https://${APT_MIRROR}/ubuntu/|g" /etc/apt/sources.list || true; \
      sed -i "s|http://security.ubuntu.com/ubuntu/|https://${APT_MIRROR}/ubuntu/|g" /etc/apt/sources.list || true; \
    fi; \
    if [ -f /etc/apt/sources.list.d/ubuntu.sources ]; then \
      sed -i "s|http://archive.ubuntu.com/ubuntu|https://${APT_MIRROR}/ubuntu|g" /etc/apt/sources.list.d/ubuntu.sources || true; \
      sed -i "s|http://security.ubuntu.com/ubuntu|https://${APT_MIRROR}/ubuntu|g" /etc/apt/sources.list.d/ubuntu.sources || true; \
    fi

# 2) Base tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    tzdata \
    sudo \
    git \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV TZ=Asia/Shanghai

# 3) ROS 2 Jazzy via mirror source
RUN set -eux; \
    curl -sSL https://${APT_MIRROR}/rosdistro/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://${ROS_MIRROR} $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-control-msgs \
    ros-jazzy-hardware-interface \
    ros-jazzy-controller-interface \
    ros-jazzy-realtime-tools \
    ros-jazzy-pluginlib \
    ros-jazzy-generate-parameter-library \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=jazzy
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV PATH=/opt/ros/jazzy/bin:${PATH}

# 4) Pip mirror for faster python installs in CN
RUN python3 -m pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

# 5) Non-root dev user
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc

WORKDIR /workspaces/PR2-Sim-Pure
USER ${USERNAME}

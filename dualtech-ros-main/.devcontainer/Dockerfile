FROM ubuntu:24.04

SHELL ["/bin/bash", "-c"]

ENV RCUTILS_COLORIZED_OUTPUT=1
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt update && apt install -y \
    build-essential \
    cmake \
    git \
    curl \
    lsb-release \
    gnupg \
    wget \
    nano \
    python3-pip \
    python3-setuptools

# Install and configure tzdata
RUN apt update && \
    apt install -y --no-install-recommends tzdata && \
    ln -fs /usr/share/zoneinfo/Europe/Warsaw /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata

# Ensure Ubuntu Universe repository is enabled
RUN apt install -y software-properties-common && \
    add-apt-repository universe

# Install ROS 2 Jazzy
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb

RUN apt update && apt upgrade -y

RUN apt update && \
    apt install -y \
    ros-dev-tools \
    ros-${ROS_DISTRO}-desktop \
    python3-colcon-common-extensions 

# Create developer user with root-like privileges
RUN existing_user=$(getent passwd 1000 | cut -d: -f1) && \
    if [ -n "$existing_user" ]; then userdel -r "$existing_user"; fi && \
    useradd -m -d /home/developer -s /bin/bash -u 1000 -G sudo developer && \
    echo 'developer:developer' | chpasswd && \
    echo "developer ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer
USER developer

# Setup ROS2 workspace
RUN mkdir -p /home/developer/ros2_ws/src 

# Copy workspace files
COPY setup.sh /home/developer/ros2_ws/setup.sh
COPY build.sh /home/developer/ros2_ws/build.sh
WORKDIR /home/developer/ros2_ws

# Update locales
USER root
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 

# Additional configs for developer user
USER developer    
RUN echo "export LANG=en_US.UTF-8" >> /home/developer/.bashrc \
    && echo "export LANGUAGE=en_US:en" >> /home/developer/.bashrc \
    && echo "export LC_ALL=en_US.UTF-8" >> /home/developer/.bashrc \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/developer/.bashrc \
    && echo "source /home/developer/ros2_ws/install/setup.bash" >> /home/developer/.bashrc \
    && echo 'export PATH="$HOME/.local/bin:$PATH"' >> /home/developer/.bashrc

CMD [ "sleep", "infinity" ]
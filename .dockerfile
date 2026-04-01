# We have to use ubuntu and then install ROS 2 because we have to symulate our raspberry pi enviroment the best as we can
#==================================
# BASE
#==================================
FROM ubuntu:24.04 AS base-ros


ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Warsaw
ENV LANG=en_US.UTF-8


RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8


RUN apt-get update && apt-get install -y locales software-properties-common curl \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


RUN apt-get update && apt-get install -y \
    ros-kilted-desktop \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /ros2_ws


RUN echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc


COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

#==========================
# UGV
#==========================

FROM base-ros AS ugv-env


RUN apt-get update && apt-get install -y \
    ros-kilted-navigation2 \
    ros-kilted-nav2-bringup \
    ros-kilted-slam-toolbox \
    ros-kilted-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*



#=======================
# UAV
#=======================
FROM base-ros AS uav-env


RUN apt-get update && apt-get install -y \
    ros-kilted-mavros \
    ros-kilted-mavros-extras \
    && rm -rf /var/lib/apt/lists/*
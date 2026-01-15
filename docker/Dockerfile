##############
# Base image #
##############
FROM nvidia/cuda:12.6.0-runtime-ubuntu22.04 as base

ARG ROS_VERSION="humble"
# Disable dialog
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# ===========
# Install ROS
# ===========
RUN apt update \
    && apt-get install -y --no-install-recommends \
        curl \
        gnupg \
        lsb-release \
        git \
        software-properties-common \
        build-essential \
        nano \
        wget

# ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros2.list


# Ignition Fortress
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    ignition-tools \
    libignition-fuel-tools7-dev \
    && rm -rf /var/lib/apt/lists/*

# ROS and python tools
RUN apt-get update \
    && apt install --no-install-recommends -y ros-${ROS_VERSION}-desktop-full \
    && apt install --no-install-recommends -y \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-osrf-pycommon \
        python3-numpy \
    &&  python3 -m pip install --upgrade --extra-index-url https://PySimpleGUI.net/install PySimpleGUI \
    && rosdep init \
    && rosdep update \
    && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN pip install getch
RUN rm -rf /var/lib/apt/lists/* && apt clean

CMD ["/bin/bash"]
WORKDIR /root/ros2_humble_ws

ENV XDG_RUNTIME_DIR=/tmp/runtime-${UID}
RUN mkdir -p -m 700 $XDG_RUNTIME_DIR

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#####################
# Development image #
#####################
FROM base as dev
    
RUN mkdir -p /root/ros2_humble_ws/src \
    && rm -rf /var/lib/apt/lists/* \
    && source /opt/ros/humble/setup.bash \
    && source "/root/.bashrc" \
    && cd /root/ros2_humble_ws && colcon build --symlink-install

RUN apt-get update \
    && apt-get install -y \
        # aerostack
        ros-humble-aerostack2 \
        python3-vcstool \
        tmux \
        tmuxinator \
        iputils-ping \
        # lidarslam
        libeigen3-dev \
        ros-humble-libg2o \
        # orb slam 3
        libopencv-dev \
        ninja-build \
        wayland-protocols \
        libc++-dev \
        libepoxy-dev \
        # ORBSLAMROS2
        ros-humble-vision-opencv \
        ros-humble-message-filters \
        # pangolin
        libpython2.7-dev \
        libglew-dev \
        libgl1-mesa-dev \
        libgtk-3-dev \
        pkg-config \
        libavcodec-dev libavformat-dev libswscale-dev \
        libjpeg-dev libpng-dev libtiff-dev libopenexr-dev \
        libxkbcommon-dev wayland-protocols \
        libboost-all-dev \
        libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y \
        # map creation - OCTOMAP
        libatlas-base-dev \
        ros-humble-octomap \
        ros-humble-octomap-server \
        ros-humble-pcl-ros \
        ros-humble-pcl-conversions \
        ros-humble-perception-pcl \
        libpcl-dev \
        pcl-tools \
        libspdlog-dev \
        qtdeclarative5-dev \
        qt5-qmake \
        libqglviewer-dev-qt5 \
        # libgoogle-glog-dev \
        ros-humble-cv-bridge \
        ros-humble-rmw-cyclonedds-cpp
# RUN echo "source /root/ros2_humble_ws/devel/setup.bash" >> ~/.bashrc

# RUN git clone https://github.com/aerostack2/project_gazebo

# COPY docker/third_parties.sh third_parties.sh
# RUN chmod +x third_parties.sh

# RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
# RUN ./third_parties.sh  # ????????????? NO FUNCIONA, HAY QUE EJECUTARLO A PARTE

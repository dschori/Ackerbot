# this dockerfile roughly follows the 'Installing from source' from:
#   http://wiki.ros.org/noetic/Installation/Source
#
ARG BASE_IMAGE=arm64v8/ros:noetic
FROM ${BASE_IMAGE}

ARG ROS_PKG=ros_base
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

# add the ROS deb repo to the apt sources list
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          git \
          nano \
	  cmake \
	  build-essential \
	  curl \
	  wget \ 
	  gnupg2 \
	  lsb-release \
	  python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y software-properties-common && \
    apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key &&\
    add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u &&\
    apt-get install apt-utils -y &&\
    apt-get install librealsense2-utils librealsense2-dev -y &&\
    apt-get install ros-noetic-joint-state-publisher -y &&\
    apt-get install ros-noetic-teleop-twist-keyboard -y &&\
    apt-get install ros-noetic-ackermann-msgs -y &&\
    apt-get install ros-noetic-map-server -y &&\
    apt-get install ros-noetic-move-base -y &&\
    apt-get install ros-noetic-global-planner -y &&\
    apt-get install ros-noetic-amcl -y &&\
    apt-get install ros-noetic-teb-local-planner -y

# Catkin Tools (catkin build)
WORKDIR /
RUN git clone https://github.com/catkin/catkin_tools.git
WORKDIR /catkin_tools
RUN pip3 install -r requirements.txt --upgrade && \
    python3 setup.py install --record install_manifest.txt

RUN mkdir -p /workspace/catkin_ws/src

# Realsense + Velodyne
WORKDIR /workspace/catkin_ws/src
RUN git clone https://github.com/dschori/realsense-ros.git
RUN git clone https://github.com/dschori/velodyne.git

# Cartographer
RUN apt-get update && \
    apt-get install -y python3-wstool python3-rosdep ninja-build stow
WORKDIR /workspace/catkin_ws
RUN wstool init src && \
    wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall && \
    wstool update -t src

# Install Dependencies and Build
RUN rosdep install --from-paths src --ignore-src -r -y
RUN src/cartographer/scripts/install_abseil.sh
RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.bash; cd /workspace/catkin_ws; catkin build -j2'

# PCA9685 and ServoKit Libraries + PID Controller
RUN pip3 install adafruit-circuitpython-pca9685 && \
    pip3 install adafruit-circuitpython-servokit && \
    pip3 install Jetson.GPIO && \
    pip3 install simple-pid
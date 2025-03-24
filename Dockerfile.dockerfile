# Base image
FROM nvcr.io/nvidia/isaac-sim:4.5.0

ENV DEBIAN_FRONTEND noninteractive

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    wget \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble
RUN apt-get update && apt-get install -y locales

RUN locale-gen en_US en_US.UTF-8

RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN apt-get install -y software-properties-common

RUN add-apt-repository universe
   
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN sh -c 'echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN apt-get update && apt-get install -y ros-humble-desktop

RUN apt-get clean

RUN rm -rf /var/lib/apt/lists/*

# Install ROS2 bridge for Isaac Sim
RUN apt-get update && apt-get install -y \
    ros-humble-ros-ign-bridge \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables
ENV ACCEPT_EULA=Y
ENV PRIVACY_CONSENT=Y
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3

# Set up volumes for caches and data
VOLUME ["/isaac-sim/kit/cache", "/root/.cache/ov", "/root/.cache/pip", "/root/.cache/nvidia/GLCache", "/root/.nv/ComputeCache", "/root/.nvidia-omniverse/logs", "/root/.local/share/ov/data", "/root/Documents"]

# Source ROS2
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# README!: the statements below, especially the second one, should run Isaac Sim every time the container is initialized. However, it does not work.
# Command to run Isaac Sim in headless mode
# CMD ["./runheadless.native.sh", "-v"]

# Command to run Isaac Sim with GUI
# CMD ["./runapp.sh", "-v"]
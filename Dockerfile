FROM nvcr.io/nvidia/isaac-sim:4.5.0

# Set the Debian frontend to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary dependencies
RUN apt-get update && apt-get install -y --allow-downgrades  \
    curl \
    gnupg2 \
    libbrotli1=1.0.9-2build6 \
    lsb-release \
    tzdata \
    libfreetype6-dev \
    libfontconfig1-dev \
    libfreetype6-dev

# Add the ROS 2 GPG key and repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

# Install ROS 2 Humble desktop version
RUN apt-get update && apt-get install -y ros-humble-desktop

# Install other ROS2 packages

RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-nav2-mppi-controller \
    ros-humble-nav2-graceful-controller \
    ros-humble-navigation2 \
    ros-humble-vision-opencv \
    ros-humble-vision-msgs \
    ros-humble-vision-msgs-rviz-plugins

# Install text editor 
RUN apt-get update && apt-get install -y nano

# Configure the ROS 2 environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Set necessary environment variables
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV ROS_DISTRO=humble

# Reset the Debian frontend
ENV DEBIAN_FRONTEND=dialog

# Set the default entry point
ENTRYPOINT ["/bin/bash"]
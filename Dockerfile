# Docker deployment for ROS 2 CAN Bridge
# Based on ROS 2 Humble with CAN support

FROM ros:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/ros2_ws

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # CAN utilities
    can-utils \
    iproute2 \
    kmod \
    usbutils \
    # ROS 2 dependencies
    ros-${ROS_DISTRO}-can-msgs \
    ros-${ROS_DISTRO}-diagnostic-msgs \
    # Python dependencies
    python3-pip \
    python3-colcon-common-extensions \
    # Development tools
    git \
    vim \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir \
    python-can \
    netifaces

# Create workspace
WORKDIR ${WORKSPACE}
RUN mkdir -p ${WORKSPACE}/src

# Copy package source
COPY . ${WORKSPACE}/src/ros_can_bridge_native/

# Install dependencies using rosdep
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the package
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Source ROS setup in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source ${WORKSPACE}/install/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR ${WORKSPACE}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ros_can_bridge_native", "can_bridge.launch.py"]

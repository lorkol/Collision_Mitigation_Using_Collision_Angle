FROM osrf/ros:jazzy-desktop-full

# Install development tools and ROS 2 Control Stack
RUN apt-get update && apt-get install -y \
    build-essential \
    git wget curl unzip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # Navigation & Simulation Base
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb3-sim \
    ros-jazzy-turtlebot3-description \
    # ROS 2 Control (The Core)
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2controlcli \
    # The "Missing Links" for MuJoCo Bridge
    ros-jazzy-ros2-control-cmake \
    ros-jazzy-controller-manager \
    ros-jazzy-hardware-interface \
    ros-jazzy-mujoco-vendor \
    ros-jazzy-transmission-interface \
    ros-jazzy-realtime-tools \
    ros-jazzy-control-toolbox \
    libglfw3-dev \
    libgles2-mesa-dev \
    && rm -rf /var/lib/apt/lists/*
    
# Ensure FetchContent uses HTTPS (avoid SSH rewrite rules inside containers)
RUN git config --global --unset-all url."git@github.com:".insteadof || true && \
    git config --global --unset-all url."ssh://git@github.com/".insteadof || true && \
    git config --global --unset-all url."https://github.com/".insteadof || true && \
    git config --global --get-regexp '^url\..*\.insteadof$' || true
    
# Set up the workspace
WORKDIR /ros2_ws

# 2. Automate the Model and Simulator
ENV TURTLEBOT3_MODEL=waffle
ENV ROBOT_SIMULATOR=gz

# 3. Initialize rosdep (if not already initialized)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && rosdep update
    
# Copy your source code folders so rosdep can "see" what it needs to install
COPY . /ros2_ws/src/Mobile-Robot-Collision

# Install EVERY dependency your source code needs (e.g., test-msgs, geographic-msgs)
RUN apt-get update && \
    rosdep install --from-paths src/Mobile-Robot-Collision/src --ignore-src -y -r && \
    rm -rf /var/lib/apt/lists/*

# Create a robust entrypoint for automated sourcing
RUN echo '#!/bin/bash \n\
source /opt/ros/jazzy/setup.bash \n\
if [ -f "/ros2_ws/install/setup.bash" ]; then source "/ros2_ws/install/setup.bash"; fi \n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh	

# This ensures that even 'docker exec' sessions are automatically sourced
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo 'if [ -f "/ros2_ws/install/setup.bash" ]; then source "/ros2_ws/install/setup.bash"; fi' >> /root/.bashrc	

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

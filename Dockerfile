FROM osrf/ros:jazzy-desktop-full

# Install development tools
RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb3-sim \
    ros-jazzy-turtlebot3-description \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*
    
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
    rosdep install --from-paths src --ignore-src -y -r && \
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

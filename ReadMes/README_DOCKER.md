# Docker Architecture: Mobile-Robot-Collision

This file documents every instruction in the \`Dockerfile\` used to build the ROS 2 Jazzy development environment.

## 1. Base Image
- **\`FROM osrf/ros:jazzy-desktop-full\`**
  - **Logic**: Inherits the official OSRF image for ROS 2 Jazzy Jalisco on Ubuntu 24.04. 
  - **Detail**: The "desktop-full" version includes the necessary libraries for RViz2 and Gazebo Harmonic to run out of the box.

## 2. Dependency Management
- **\`RUN apt-get update\`**: Refreshes the local package index to ensure the latest binary versions are retrieved.
- **\`ros-jazzy-navigation2\` & \`nav2-bringup\`**: Installs the high-level navigation stack and the official launch scripts.
- **\`ros-jazzy-nav2-minimal-tb3-sim\`**: Provides the core logic required to run a TurtleBot3 in a simulated world.
- **\`ros-jazzy-turtlebot3-description\`**
Provides the physical 3D mesh files (.dae/.stl) for the robot.
- **\`python3-colcon-common-extensions\`**: The primary build tool for ROS 2, used to compile your C++ workspace.
- **python3-rosdep**: Tools for managing system dependencies.
- **python3-vcstool**: Version control tool for managing multiple repositories.
- **git & wget**: Essential utilities for cloning code and downloading assets.
- **\`rm -rf /var/lib/apt/lists/*\`**: Deletes the package index after installation to keep the final image size small.

## 3. Workspace & State
- **\`WORKDIR /ros2_ws\`**: Defines the root directory for all subsequent commands, standardizing the workspace structure.
- **\`ENV TURTLEBOT3_MODEL=waffle\`**: Automatically tells Nav2 to use the Waffle model, matching the entity found in your Gazebo simulation.
- **\`ENV ROBOT_SIMULATOR=gz\`**: Specifically directs the bringup scripts to use the new Gazebo Sim (Harmonic) instead of the old Gazebo Classic.

## 4. Automated Source Dependency Resolution
* **`COPY . /ros2_ws/src/Mobile-Robot-Collision`**
    * **Logic**: Injects your local source code metadata into the Docker image during the build phase.
    * **Detail**: This is required so the `rosdep` tool can scan your `package.xml` files inside the image to identify every system library your code requires before you even start the container.
* **`RUN apt-get update && rosdep install --from-paths src --ignore-src -y -r && rm -rf /var/lib/apt/lists/*`**
    * **Logic**: Automatically resolves and installs all missing system dependencies.
    * **Explanation**: This command scans your entire source tree and uses the global `rosdep` database to install every specific development package (e.g., `test-msgs`, `geographic_msgs`) required to compile Nav2 from scratch.
    * **The Risk**: Without this automation, any attempt to modify or build the Nav2 source code would result in a "Package not found" error, forcing you to manually hunt down and add `apt-get install` lines one by one.

## 5. Dependency Database
- **\`rosdep init && update\`**
  - **Logic**: Initializes the database that translates ROS-specific dependencies (from \`package.xml\`) into actual Linux system install commands.
  **Explanation**: The command `rosdep init` can only be run once per system. This check verifies if the initialization file already exists. 
  **The Risk**: Without this `if` block, the Docker build would fail with a "system already initialized" error if a previous layer or base image already ran this command.

## 6. Automated Sourcing (Entrypoint)
- **\`ENTRYPOINT ["/entrypoint.sh"]\`**
  - **Logic**: Forces every terminal you open to run a sourcing script first. 
  - **Detail**: It runs \`source /opt/ros/jazzy/setup.bash\` and your local \`install/setup.bash\` if it exists, ensuring your custom code overrides system defaults automatically.
- **\`exec "\$@"\`**: Critical shell logic that hands over the process to your final command (like bash), ensuring signals like Ctrl+C are handled properly.
**Explanation**: This checks if you have successfully built your workspace code. 
* **Purpose**: If a build exists, it "overlays" your local C++ changes on top of the base ROS 2 environment so your custom controller code is what actually runs.
* **The Risk**: Without the `if` check, a brand-new container would crash immediately upon opening because the `install/setup.bash` file hasn't been created yet by a build command.

## 7. Interactive Shell Sourcing (.bashrc)
* **`RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc`**
    * **Logic**: Appends the primary ROS 2 environment setup command to the user's bash configuration file.
    * **Detail**: This ensures that every time you use `docker exec` to open a new terminal window, the ROS 2 commands are immediately available without manual sourcing.
* **`RUN echo 'if [ -f "/ros2_ws/install/setup.bash" ]; then source "/ros2_ws/install/setup.bash"; fi' >> /root/.bashrc`**
    * **Logic**: Automatically overlays your custom workspace (including your modified Nav2 source code) onto the environment.
    * **Explanation**: This check looks for the local `install/setup.bash` created after a successful `colcon build`.
    * **The Benefit**: It guarantees that your experimental MPPI critic or Navigation2 modifications are the "active" versions being used by the system whenever you start a new terminal session.
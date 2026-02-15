# Technical Breakdown: start_nav2.sh

### Host Access
- **`xhost +local:docker`**: Grants the container permission to use the host's X11 server to display GUI windows.

### Lifecycle Management
- **`docker build -t $IMAGE_NAME .`**: Automatically builds the image if it doesn't exist locally.

 `if [[ "$(docker images -q $IMAGE_NAME ...)" == "" ]]`.
* **Explanation**: Checks your local Docker library for the `nav2_jazzy` image.
* **Action**: If missing, it automatically triggers `docker build` to create your environment from the Dockerfile.
* **Benefit**: Ensures a "One-Click" setup for fresh installations.


The script uses nested `if` statements to manage the `nav2_dev` container automatically:

* **`if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]`**: Checks if the container is already running. If it is, it uses `docker exec` to open a new terminal tab inside it instead of launching a duplicate.
* **`elif [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]`**: Checks if the container exists but is stopped. If so, it uses `docker start` to revive it, preserving any temporary files or state.
* **The `else` block**: If no container is found, it performs a fresh `docker run`.

### Container Execution Flags (`docker run`)
- **`--init`**: Uses a tiny init process (tini) to reap "zombie" processes and ensure Ctrl+C shuts down Gazebo cleanly.
- **`--network host`**: Shares the host's network stack for maximum communication efficiency between ROS 2 nodes.
- **`--privileged`**: Allows the container to access host hardware devices (like cameras or sensors).

### Hardware Acceleration (The RTX 3070 Bridge)
- **`--gpus all`**: Maps the NVIDIA GPU into the container.
- **\`--env="NVIDIA_VISIBLE_DEVICES=all"\`**: Explicitly tells the NVIDIA Docker runtime which GPUs to map. Setting this to "all" ensures that all available CUDA cores and rendering engines on your RTX 3070 are accessible.
- **`--env="NVIDIA_DRIVER_CAPABILITIES=all"`**: Enables the GPU for both 3D rendering (OpenGL) and AI compute (CUDA).
- **`--env="QT_X11_NO_MITSHM=1"`**: Disables shared memory for the UI to prevent "black screen" rendering bugs in Docker.

### Data Persistence
- **`--volume="$(pwd):/ros2_ws/src/Mobile-Robot-Collision"`**: This is the "Bind Mount." It maps your host code directory directly into the container. Any C++ edits made on your host are instantly available in the container.
- **`--volume="$(pwd)/build:/ros2_ws/build"`**: his is the "Bind Mount." It maps your host code directory directly into the container. Makes it so what colcon build makes stays.
- **`--volume="$(pwd)/build:/ros2_ws/install"`**: his is the "Bind Mount." It maps your host code directory directly into the container. Makes it so what is installed with sudo stays.
- **`--volume="$(pwd)/build:/ros2_ws/log"`**: his is the "Bind Mount." It maps your host code directory directly into the container. Makes it so logs stay.
- **`--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"`**: Mounts the actual socket file required for X11 GUI windows to appear on your screen.
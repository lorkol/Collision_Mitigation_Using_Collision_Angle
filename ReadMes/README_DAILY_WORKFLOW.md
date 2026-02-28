# Daily Development Workflow

## 1. Launch Environment
Run the start script from your host:
```bash
./start_nav2.sh
```

## 2. Start Simulation
Source the workspace:

```bash
source install/setup.bash
```

Set the model to Waffle and launch the bringup:
```bash
ros2 launch collision_angle_mitigation circular_simulation_launch.py headless:=False
```

## 3. Initialize RViz
- Click **2D Pose Estimate**.
- Drag the green arrow on the robot's location in the map.
- Confirm the `RobotModel` display shows the Waffle shape (ensure "Collision Enabled" is checked if meshes fail).



# Opening an additional terminal
from **HOST** run 

### New method: 

Made a shortcut using adding an alias to my ~/.bashrc:

alias nav2_docker='docker exec -it nav2_dev /bin/bash'

Now I just need to run:
```bash
nav2_docker
```

Source the workspace:

```bash
source install/setup.bash
```


### Old method: 
```bash
docker ps
```
To get a list of running containers.

Then run **With the relevant container ID**
```bash
docker exec -it 50a1c7276078 bash
```
Then in the container run
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
``` 
# While working on EDT:
keep building using:
```bash
colcon build --packages-select collision_angle_mitigation --parallel-workers 2
```

Running the node:
```bash
source install/setup.bash
ros2 run collision_angle_mitigation edt_publisher_node
```

# Misc
Make sure when opening and connecting to VSCode to download the c/c++ and IntelliJ extensions


# TroubleShooting

## 🖥️ Troubleshooting GUI & Display Issues

If you encounter errors like `qt.qpa.xcb: could not connect to display :0` or if **Gazebo** and **RViz2** fail to open after a system reboot:

### 1. Remove the container
**On HOST**
```bash
docker rm -f nav2_dev
```
**On HOST**
```bash
echo $DISPLAY
```
If this returns :1 or any value that differs from your previous session, it confirms why the internal application (which was looking for :0) failed.

### 2. Reset X11 Permissions
Permissions for Docker to access your host's display server are reset upon every logout or reboot. Run this on your **host terminal**:
```bash
xhost +local:docker
```

### 3. 
To ensure the container is created with the exact environment variables and hardware access required, run a fresh instance using this command:
```bash
docker run -it \
    --name nav2_dev \
    --network host \
    --privileged \
    --gpus all \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/ros2_ws/src/Mobile-Robot-Collision" \
    --volume="$(pwd)/build:/ros2_ws/build" \
    --volume="$(pwd)/install:/ros2_ws/install" \
    --volume="$(pwd)/log:/ros2_ws/log" \    nav2_jazzy
```
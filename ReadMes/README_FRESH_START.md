# Fresh Setup from Scratch

## 1. Directory Structure
Create your development root and clone your source:
```bash
mkdir -p ~/dev && cd ~/dev
```
```bash
git clone https://github.com/ros-navigation/navigation2.git src/Mobile-Robot-Collision
```

## 2. Image Creation
Ensure the `Dockerfile` and `start_nav2.sh` are in the root. Run the start script; it will automatically detect the missing image and trigger a build.
```bash
cd Mobile-Robot-Collision
chmod +x start_nav2.sh && ./start_nav2.sh
```

## 3. Host Prerequisites
Ensure the NVIDIA Container Toolkit is installed on the host:
```bash
sudo apt install -y nvidia-container-toolkit && sudo systemctl restart docker
```

## Misc
In the repo, under .vscode, add a c_cpp_properties.json: (After adding the c/c++ extensions)

```bash
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/src/**",
                "${workspaceFolder}/src/collision_angle_mitigation/include",
                "${workspaceFolder}/install/**",
                "/opt/ros/jazzy/include",
                "/opt/ros/jazzy/include/**",
                "/opt/ros/jazzy/include/rclcpp",
                "/opt/ros/jazzy/include/rcl_interfaces",
                "/opt/ros/jazzy/include/nav_msgs",
                "/usr/include",
                "/usr/include/opencv4",
                "/ros2_ws/src/Mobile-Robot-Collision/build/collision_angle_mitigation/rosidl_generator_cpp"
            ],
            "defines": [
                "DEFAULT_RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
                "_FILE_OFFSET_BITS=64"
            ],
            "compilerPath": "/usr/bin/c++",
            "cStandard": "c17",
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```

and a settings.json(after adding the python extensions):
```bash
{
    "python.analysis.extraPaths": [
        "${workspaceFolder}/install/nav2_common/lib/python3.12/site-packages",
        "${workspaceFolder}/install/nav2_msgs/lib/python3.12/site-packages",
        "${workspaceFolder}/install/dwb_msgs/lib/python3.12/site-packages",
        "${workspaceFolder}/install/nav_2d_msgs/lib/python3.12/site-packages",
        "/opt/ros/jazzy/lib/python3.12/site-packages"
    ],
    "python.defaultInterpreterPath": "/usr/bin/python3",
    "python.linting.enabled": false,
    "python.linting.pylintEnabled": false,
    "[python]": {
        "editor.defaultFormatter": "ms-python.python",
        "editor.formatOnSave": true
    },
    "favorites.resources": [
        {
            "filePath": "src/navigation2/nav2_bringup/launch/tb3_simulation_launch.py",
            "group": "Default"
        }
    ],
    "cmake.sourceDirectory": "/ros2_ws/src/Mobile-Robot-Collision/src/collision_angle_mitigation"
}

```
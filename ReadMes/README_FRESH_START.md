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
            "name": "ROS 2",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/*/include/**",
                "/usr/include/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}

```
# First Run Configuration

Inside the container, follow these steps to prepare the workspace for the first time:


## 1. Build the Workspace
Use symlink installs so that C++ changes update more efficiently. 
Limits the build to 2 parallel workers to prevent CPU/RAM exhaustion on your host.

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --parallel-workers 2
```
- **Why**: Limits the build to 2 CPU cores to prevent your NYU workstation from freezing during the heavy C++ compilation of Nav2.

### In Case of Build Error:
If `colcon build` fails with errors like `CMake Error: By not providing "FindXXXX.cmake"...` or `package XXXX not found`, it usually means your source code requires libraries that aren't installed in your Docker image yet.

#### When to run this:
* **After cloning a new repository** into your `src` folder (like the MuJoCo bridge).
* **After creating a new ROS 2 package** that uses a new dependency.
* **If you see "package not found" errors** during the build process.

#### How to run it:
Run this **inside the container** from the root of your workspace:

```bash
cd /ros2_ws
sudo apt update
rosdep install --from-paths src --ignore-src -y
```

## 2. Source the New Workspace
```bash
source install/setup.bash
```
(Note: The entrypoint script will handle this for every terminal opened after this step).

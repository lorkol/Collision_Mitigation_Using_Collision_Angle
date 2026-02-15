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

## 2. Source the New Workspace
```bash
source install/setup.bash
```
(Note: The entrypoint script will handle this for every terminal opened after this step).

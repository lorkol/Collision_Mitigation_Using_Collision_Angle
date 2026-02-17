import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('Collision_Angle_Mitigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    
    model_path = os.path.join(pkg_share, 'models', 'turtlebot3_waffle.xml')
    params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')

    return LaunchDescription([
        # 1. Start MuJoCo with our robot model
        Node(
            package='mujoco_ros2_control',
            executable='mujoco_server',
            parameters=[{'modelfile': model_path, 'use_sim_time': True}],
            output='screen'
        ),

        # 2. Tell ROS what the robot looks like (for RViz)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(model_path).read(), 'use_sim_time': True}]
        ),

        # 3. Start Nav2 (but using our custom params)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'use_sim_time': 'True',
                'params_file': params_path,
                'autostart': 'True'
            }.items()
        )
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    edt_publisher_node = Node(
        package='collision_angle_mitigation',
        executable='edt_publisher_node',
        name='edt_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    edt_monitor_node = Node(
        package='collision_angle_mitigation',
        executable='edt_monitor_node',
        name='edt_monitor_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_radius': 0.17,
            'base_frame': 'base_footprint'
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(edt_publisher_node)
    ld.add_action(edt_monitor_node)

    return ld
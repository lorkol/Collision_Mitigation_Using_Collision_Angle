from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='collision_angle_mitigation',
            executable='edt_publisher_node',
            name='edt_publisher_node',
            output='screen',
            parameters=[{
                'costmap_topic': '/global_costmap/costmap_raw',
                'edt_map_topic': '/edt_map',
                'obstacle_threshold': 254  # nav2_costmap_2d::LETHAL_OBSTACLE
            }]
        )
    ])

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#!/usr/bin/env python3
"""
ROS 2 launch file for chassis_keyboard_teleop.
"""

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chassis_keyboard_teleop',
            executable='teleop_node',
            name='keyboard_teleop',
            output='screen',
            parameters=[{'max_linear': 1.0,
                        'max_angular': 9.0,
                        'linear_accel': 1.0,
                        'linear_decel': 2.0,
                        'angular_accel': 9.0,
                        'angular_decel': 18.0,
                        'event_path': '/dev/input/event4'
            }]
        ),
    ])

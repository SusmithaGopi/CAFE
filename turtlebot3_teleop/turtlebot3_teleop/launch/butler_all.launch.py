from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen'
        ),
        Node(
            package='turtlebot3_teleop',
            executable='confirmation_tester',
            name='confirmation_tester',
            output='screen'
        )
    ])


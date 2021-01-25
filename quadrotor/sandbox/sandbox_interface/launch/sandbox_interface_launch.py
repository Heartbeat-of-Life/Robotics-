from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sandbox_interface',
            namespace='',
            executable='sandbox_interface',
#            name='sandbox_io',
            output='screen'
        ),
    ])

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
def generate_launch_description():
        ld = LaunchDescription()
        # Sandbox launch 
        package_prefix = get_package_share_directory('sandbox_interface')
        launch_file=PythonLaunchDescriptionSource([package_prefix,'/launch/sandbox_interface_launch.py'])
        sandbox_interface_launch = IncludeLaunchDescription(launch_file)
        #Gazebo Launch
        package_prefix = get_package_share_directory('gazebo_ros')
        launch_file = PythonLaunchDescriptionSource([package_prefix,'/launch/gazebo.launch.py'])
        gazebo_launch = IncludeLaunchDescription(launch_file)

#        declare_use_world_file_arg=DeclareLaunchArgument(
#                'world',default_value='empty.world',description='World to be launched within gazebo')

        ld.add_action(sandbox_interface_launch)
        ld.add_action(gazebo_launch)
        #ld.add_action(declare_use_world_file_arg)
        n=Node(
                package='test_pg',
                namespace='',
                executable='test_pg',
                name='test_node',
                output='screen')
        ld.add_action(n)

        return ld

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Get the package directories
    cpp_python_package_share = get_package_share_directory('cpp_python_package')

    # Parameters
    parameter_file = os.path.join(cpp_python_package_share, 'config', 'cpp_publisher.yaml')

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='/my_namespace',
        description='Namespace'
    )


    obstacle_tracker = Node(
            package = 'cpp_python_package',
            executable = 'cpp_publisher',
            name='cpp_publisher',
            namespace = LaunchConfiguration('namespace'),
            parameters = [parameter_file],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
    )


    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(obstacle_tracker)
    return ld

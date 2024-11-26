import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Get the package directories
    filter_signal_package_share = get_package_share_directory('filter_signal')

    # Parameters
    parameter_file = os.path.join(filter_signal_package_share, 'config', 'analyse.yaml')

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Namespace'
    )


    analyse_topic_cmd = Node(
            package = 'filter_signal',
            executable = 'analyse.py',
            namespace = LaunchConfiguration('namespace'),
            parameters = [parameter_file],
    )


    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(analyse_topic_cmd)
    return ld

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
        
    node1=Node(
        package = 'sample_signal',
        name = 'sample_signal',
        executable = 'sample_signal',
        parameters = [os.path.join(get_package_share_directory('sample_signal'), 'config', 'sample_signal.yaml')]
    )
    ld.add_action(node1)
    return ld
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    # Get the package directories
    sample_signal_package_share = get_package_share_directory('sample_signal')

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Namespace'
    )
        
    sample_signal_cmd=Node(
        package = 'sample_signal',
        name = 'imu_sample',
        executable = 'imu.py',
        namespace=LaunchConfiguration('namespace'),
        parameters = [os.path.join(sample_signal_package_share, 'config', 'imu.yaml')]
    )
    ld.add_action(declare_namespace_cmd)
    ld.add_action(sample_signal_cmd)
    return ld
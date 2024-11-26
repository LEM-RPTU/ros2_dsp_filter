import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():    
    sample_cmd=Node(
        package = 'fft',
        name = 'fft_sample_signal',
        executable = 'plotfourier.py',
        parameters = [os.path.join(get_package_share_directory('fft'), 'config', 'config.yaml')]
    )

    filtered_sample_cmd=Node(
        package = 'fft',
        name = 'fft_filtered_signal',
        executable = 'plotfourier.py',
        parameters = [os.path.join(get_package_share_directory('fft'), 'config', 'config.yaml')]
    )
    ld = LaunchDescription()
    ld.add_action(sample_cmd)
    #ld.add_action(filtered_sample_cmd)
    return ld
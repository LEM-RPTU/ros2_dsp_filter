from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    generic_filter_signal_cmd = Node(
            package = 'filter_signal',
            executable = 'generic_filter.py',
            name='filter_signal'
    )


    ld = LaunchDescription()
    ld.add_action(generic_filter_signal_cmd)
    return ld

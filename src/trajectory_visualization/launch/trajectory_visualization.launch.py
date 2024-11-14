from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create launch description
    ld = LaunchDescription()

    # RSP -Robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("abb_model"), 'launch', 'rsp_irb4600.launch.py'
        )])
    )

    ld.add_action(rsp)


    # JSP - trajectory visualization
    jsp = Node(
        package='trajectory_visualization',
        executable='trajectory_visualization',
        name='trajectory_visualization',
        output='screen'
    )
    ld.add_action(jsp)

    # Add RViz with the specified config file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    ld.add_action(rviz_node)

    return ld
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    # Include the Robot State Publisher (RSP) launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("abb_model"), 'launch', 'rsp_irb4600.launch.py'
        )])
    )
    ld.add_action(rsp)

    # Add RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    ld.add_action(rviz_node)

    # Add Trajectory Computation node
    trajectory_computation_node = Node(
        package='block2_izmailov',
        executable='trajectory_computation',
        name='trajectory_computation',
        output='screen'
    )
    ld.add_action(trajectory_computation_node)

    # Add Trajectory Visualization node with delay
    trajectory_visualization_node = TimerAction(
        period=3.0,  # Delay in seconds
        actions=[
            Node(
                package='block2_izmailov',
                executable='ik_trajectory_visualization',
                name='ik_trajectory_visualization',
                output='screen'
            )
        ]
    )
    ld.add_action(trajectory_visualization_node)

    graphs_node = Node(
        package='block2_izmailov',
        executable='draw_tool_graphs.py',
        name='draw_tool_graphs',
        output='screen'
    )
    ld.add_action(graphs_node)

    return ld


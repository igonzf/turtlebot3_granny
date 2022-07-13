
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory

    bringup_dir = get_package_share_directory('turtlebot3_granny')
    launch_dir = os.path.join(bringup_dir, 'launch/turtlebot3')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    map_yaml_file = os.path.join(
        bringup_dir,
        'maps/granny',
        'GrannyAnnie.yaml')

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'turtlebot3_nav2_bringup_launch.py')),
        launch_arguments={'map': map_yaml_file,
                          'initial_pose_x': "0.0",
                          'initial_pose_y': "0.0",
                          'initial_pose_z': "0.0",
                          'initial_pose_yaw': "3.13"}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(nav2_bringup_cmd)

    return ld

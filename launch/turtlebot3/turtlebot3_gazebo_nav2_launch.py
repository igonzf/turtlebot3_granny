import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_granny'), 'launch/turtlebot3')

    #
    # LAUNCHES
    #

    turtlebot3_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "turtlebot3_nav2_bringup_launch.py"))
    )

    turtlebot3_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "turtlebot3_gazebo_launch.py")),
        launch_arguments={'launch_gzclient': 'True',
                          'world': ''
                          }.items()
    )

    ld = LaunchDescription()

    #
    # ADD
    #

    ld.add_action(turtlebot3_nav_cmd)
    ld.add_action(turtlebot3_gazebo_cmd)

    return ld

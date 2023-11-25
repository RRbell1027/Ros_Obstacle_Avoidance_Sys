import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Open controller, detector and teleop from my_avoid_sys package once.

    detector = Node(package='my_avoid_sys', executable='obstacle_detect')

    controller = Node(package='my_avoid_sys', executable='robot_controller')

    teleop = Node(package='my_avoid_sys', executable='teleop_twist')

    server = Node(package='my_avoid_sys', executable='data_server')

    launch_sim = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('my_bot'),'launch','launch_sim.launch.py')]),
                )


    return LaunchDescription([
        launch_sim,
        controller,
        detector,
        server,
        # teleop
    ])
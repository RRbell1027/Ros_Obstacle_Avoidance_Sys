from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Open controller, detector and teleop from my_avoid_sys package once.

    detector = Node(package='my_avoid_sys', executable='obstacle_detect')

    controller = Node(package='my_avoid_sys', executable='robot_controller')

    teleop = Node(package='my_avoid_sys', executable='teleop_twist')

    return LaunchDescription([
        controller,
        detector,
        teleop
    ])
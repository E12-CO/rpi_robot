import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    motor_node = Node(package='rpi_robot', executable='rpi_motor')

    cam_node = Node(package='rpi_robot', executable='rpi_cam')
    

    # Launch them all!
    return LaunchDescription([
       motor_node,
       cam_node
    ])

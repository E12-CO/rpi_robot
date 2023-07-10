import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    joy_node = Node(package='joy', executable='joy_node')

    joy_converter_node = Node(package='rpi_robot', executable='rpi_joy')
    
    #cam_node = Node(package='rpi_robot', executable='controller_cam')
    cam_node = Node(package='rqt_image_view', executable='rqt_image_view')

    # Launch them all!
    return LaunchDescription([
       joy_node,
       joy_converter_node,
       cam_node
    ])

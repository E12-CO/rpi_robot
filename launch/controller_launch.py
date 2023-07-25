import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


import xacro
import yaml


def generate_launch_description():

   #Joy_connector node
   joy_node = Node(package='joy', executable='joy_node')

   #Joy_pub
   joy_converter_node = Node(package='rpi_robot', executable='rpi_joy')
   
   #Camera_sub node
   #cam_node = Node(package='rpi_robot', executable='controller_cam')
   cam_node = Node(package='rqt_image_view', executable='rqt_image_view')

   #Getting robot config
   rviz_config_dir = os.path.join(
        get_package_share_directory('rpi_robot_description'),
        'config',
        'view_bot.rviz'
    )

   #Rviz node
   display_rviz = Node(package='rviz2', executable='rviz2',
                        name='rviz2',
                        arguments=['-d', rviz_config_dir],
                        output='screen')
   
   #SLAM
   slam_node = IncludeLaunchDescription(
               PythonLaunchDescriptionSource([
                  FindPackageShare("slam_toolbox"), '/launch', '/online_async_launch.py']),
                  launch_arguments={'slam_params_file':'~/ros2_ws/src/rpi_robot_description/config/mapper_params_online_async.yaml','use_sim_time':'false'}.items()
         )

   # Launch them all!
   return LaunchDescription([
      joy_node,
      joy_converter_node,
      cam_node,
      #display_rviz,
      #slam_node
   ])

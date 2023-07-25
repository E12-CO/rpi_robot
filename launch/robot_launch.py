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

   #Joy_sub to motor_control node
   motor_node = Node(package='rpi_robot', executable='rpi_motor')

   #Camera_pub node
   cam_node = Node(package="v4l2_camera",
            executable="v4l2_camera_node",
            name="v4l2_camera_node",
            parameters=[{"image_size": "[640,480]", "camera_frame_id": "camera_link_optical"}],
        )

   #
   rpi_robot_description_path = os.path.join(
      get_package_share_directory('rpi_robot_description'))

   #Getting robot urdf
   xacro_file = os.path.join(rpi_robot_description_path,
                           'urdf',
                           'rpi_robot.urdf.xacro')

   doc = xacro.parse(open(xacro_file))
   xacro.process_doc(doc)
   robot_description_config = doc.toxml()
   robot_description = {'robot_description': robot_description_config}

   #Robot_state_publisher node
   node_robot_state_publisher = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      output='screen',
      parameters=[robot_description]
   )

   #Lidar_pub node
   lidar_driver_node = IncludeLaunchDescription(
               PythonLaunchDescriptionSource([
                  FindPackageShare("ydlidar_ros2_driver"), '/launch', '/ydlidar_launch.py'])
         )
   

   # Launch them all!
   return LaunchDescription([
      motor_node,
      cam_node,
      #node_robot_state_publisher,
      #lidar_driver_node
   ])

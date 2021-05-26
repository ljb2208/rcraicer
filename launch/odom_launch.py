from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   return LaunchDescription([
       Node(
           package="rcraicer",
           executable="wheel_odometry",
           name='wheel_odometry',
           output='screen'
       )
   ])
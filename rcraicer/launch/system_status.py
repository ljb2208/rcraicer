from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([       
       Node(
           package="rcraicer",           
           executable="system_status",
           name="system_status"
       )
   ])
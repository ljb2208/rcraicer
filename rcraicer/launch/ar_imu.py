from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([       
       Node(
           package="rcraicer",           
           executable="imu_mavlink",
           name="imu_mavlink"
       )
   ])
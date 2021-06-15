from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([       
       Node(
           package="rcraicer_gps",           
           executable="gps_node",
           name="gps_rover",
           parameters=[{"base" : False},{"serial_port" : "/dev/rcGPSRover"},{"msg_debug" : True}]
       )

   ])
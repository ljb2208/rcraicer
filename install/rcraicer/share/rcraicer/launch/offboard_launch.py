from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
       Node(
           package="joy",           
           executable="joy_node",
           name="joy_node"
       ),
       Node(
           package="rcraicer_gps",           
           executable="gps_node",
           name="gps_base",
           parameters=[{"base" : True},{"serial_port" : "/dev/rcGPSBase"}]
       )
   ])
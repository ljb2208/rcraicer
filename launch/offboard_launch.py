from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
       Node(
           package="joy",           
           node_executable="joy_node",
           node_name="joy_node"
       )
   ])
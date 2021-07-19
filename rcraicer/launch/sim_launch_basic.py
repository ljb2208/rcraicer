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
           package="rcraicer_sim",           
           executable="sim_debug_node",
           name="sim_debug_node"    
       ),
       Node(
           package="rcraicer_sim",           
           executable="airsim_node",
           name="airsim_node"    
       )
   ])   
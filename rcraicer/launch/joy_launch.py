from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([       
       Node(
           package="rcraicer",           
           executable="joystick_control",
           name="joystick_control",
           parameters=[{"throttle_enable_button" : 3}, {"steering_enable_button" : 2}, {"throttle_damping" :1.0}, {"steering_damping" : 1.0}, {"runstop_toggle_buttons" : [0]}]
       ),
       Node(
           package="joy",           
           executable="joy_node",
           name="joy_node"
       )

   ])
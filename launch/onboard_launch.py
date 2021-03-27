from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
       Node(
           package="rcraicer",           
           node_executable="arduino_controller",
           node_name="arduino_controller"
       ),
       Node(
           package="rcraicer",           
           node_executable="imu_mavlink",
           node_name="imu_mavlink"
       )
   ])
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
       Node(
           package="rcraicer",           
           executable="arduino_controller",
           name="arduino_controller",
           parameters=[{"steering_input_factor" : 1.0},{"steering_degrees_per_tick" : -0.10834}]
       ),
       Node(
           package="rcraicer",           
           executable="imu_mavlink",
           name="imu_mavlink"
       )
   ])
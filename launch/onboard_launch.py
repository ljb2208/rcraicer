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
       ),
       Node(
           package="rcraicer",           
           executable="wheel_odometry",
           name="wheel_odometry"
       ),
       Node(
           package="rcraicer_gps",           
           executable="gps_node",
           name="gps_rover",
           parameters=[{"base" : False},{"serial_port" : "/dev/rcGPSRover"}]
       )

   ])
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
           package="rcraicer",           
           executable="wheel_odometry",
           name="wheel_odometry",
           parameters=[{"vehicle_wheelbase" : 1.578},{"vehicle_width" : 1.397}]
       ),
       Node(
           package="rcraicer_sim",           
           executable="sim_node",
           name="sim_node"           
       )
   ])   
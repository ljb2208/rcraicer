from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([       
       Node(
           package="rcraicer_gps",           
           executable="gps_node",
           name="gps_base",
           parameters=[{"base" : True},{"serial_port" : "/dev/ttyACM0"},{"msg_debug" : True},
                {"base_svin_acc_limit" : 40000}]
       )

   ])
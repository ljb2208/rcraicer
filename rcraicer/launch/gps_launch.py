from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([       
       Node(
           package="rcraicer_gps",           
           executable="gps_node",
           name="gps_rover",
           parameters=[{"base" : True},{"serial_port" : "/dev/rcGPS"},{"msg_debug" : True}, {"base_svin_acc_limit" : 10000}]
       )

   ])
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
   parameters_file = os.path.join("/home/lbarnett/ros2_ws/src/rcraicer/rcraicer", 'config', 'dual_ekf.yaml')

   return LaunchDescription([
       Node(
           package="robot_localization",           
           executable="ekf_node",
           name="ekf_filter_node_odom",
           parameters=[parameters_file],
           remappings=[('odometry/filtered', 'odometry/local')]                      
       ),
       Node(
           package="robot_localization",           
           executable="ekf_node",
           name="ekf_filter_node_map",
           parameters=[parameters_file],
           remappings=[('odometry/filtered', 'odometry/global')]
       ),
       Node(
           package="robot_localization",           
           executable="navsat_transform_node",
           name="navsat_transform",
           parameters=[parameters_file],
            remappings=[('imu/data', 'imu'),
            ('gps/fix', 'gps/fix'), 
            ('gps/filtered', 'gps/filtered'),
            ('odometry/gps', 'odometry/gps'),
            ('odometry/filtered', 'odometry/global')]  
       )

   ])
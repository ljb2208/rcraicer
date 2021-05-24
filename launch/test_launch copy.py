from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   return LaunchDescription([
       Node(
           package="tf2_ros",           
           executable="static_transform_publisher",
           arguments=["0","0", "0", "0",  "0",  "0",  "1", "base_link", "imu_link"]
        #    parameters=[
        #     {"frame_id": "base_link"},
        #     {"child_frame_id": "imu_link"},
        #     {"translation.x": 0},
        #     {"translation.y": 0},
        #     {"translation.z": 0},
        #     {"rotation.x": 0},
        #     {"rotation.y": 0},
        #     {"rotation.z": 0},
        #     {"rotation.w": 1}
        # ]
       ),
       Node(
           package="robot_localization",
           executable="ekf_node",
           name='ekf_filter_node',
           output='screen',
        #    parameters=[os.path.join(get_package_share_directory("rcraicer"), 'params', 'ekf.yaml')],           
           parameters=["/home/lbarnett/ros2_ws/src/rcraicer/config/ekf.yaml"]
       )
   ])
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
       Node(
           package="mapviz",           
           executable="mapviz",
           name="mapviz"
       ),
       Node(
           package="swri_transform_util",           
           executable="initialize_origin.py",
           name="initialize_origin",
           parameters=[{"local_xy_frame" : "map"},
                        {"local_xy_origin" : "auto"},
                        {"local_xy_origins" :  "[{ name: swri, latitude: 41.00469, longitude: -74.08575, altitude: 20, heading: 0.0}, { name: back_40, latitude: 29.447507, longitude: -98.629367, altitude: 200.0, heading: 0.0}]"}]
       ),
       Node(
           package="tf2_ros",           
           executable="static_transform_publisher",
           name="swri_transform",
           arguments=["0","0", "0", "0",  "0",  "0",  "1", "/map", "/origin"]
       )
   ])   
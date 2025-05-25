from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
       Node(
           package="rcraicer",           
           executable="ar_arduino_controller",
           name="ar_arduino_controller",
           parameters=[{"wheel_diameter" : 0.19}, {"steering_servo_points" : [987, 1503, 2010]}, {"throttle_servo_points" : [987, 1540, 2010]}, {"brake_servo_points": [1500, 1500, 2000]}]           
           #parameters=[{"steering_input_factor" : 1.0},{"steering_degrees_per_tick" : -0.10834}]
       ),
       Node(
           package="rcraicer",           
           executable="imu_mavlink",
           name="imu_mavlink"
       ),
       Node(
           package="rcraicer",           
           executable="system_status.py",
           name="system_status"
       ),
       Node(
           package="rcraicer",           
           executable="wheel_odometry",
           name="wheel_odometry",
           parameters=[{"vehicle_wheelbase" : 0.57785}, {"vehicle_width" : 0.3175}]
       ),
       Node(
           package="rcraicer_gps",           
           executable="gps_node",
           name="gps_rover",
           parameters=[{"base" : False},{"serial_port" : "/dev/rcGPS"},{"output_rf" :  True}]
       ),
        Node(
           package="tf2_ros",           
           executable="static_transform_publisher",
           name="imu_transform_publisher",
           arguments=["0","0", "0", "0",  "0",  "0",  "1", "imu_link", "base_link"]
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
           package="tf2_ros",           
           executable="static_transform_publisher",
           name="gps_transform_publisher",
           arguments=["0","0", "0", "0",  "0",  "0",  "0", "gps_link", "base_link"]
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
       )

   ])
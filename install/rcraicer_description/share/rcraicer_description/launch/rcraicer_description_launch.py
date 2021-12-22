import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_filename = 'rcraicer.urdf.xacro'

    print("xacro_file_name : {}".format(xacro_filename))

    xacro_filename = os.path.join(
          get_package_share_directory('rcraicer_description'),"urdf",
        xacro_filename)  

    urdf_filename = os.path.join(
          get_package_share_directory('rcraicer_description'),"urdf",
        "rcraicer.urdf")  


    print("urdf_filename : {}".format(urdf_filename))
    urdf_doc = xacro.process_file(xacro_filename)
    
    urdf_file = open(urdf_filename , "w")
    urdf_doc.writexml(urdf_file);
    urdf_file.close()

    print("processed\n")

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf_filename]),      
        Node(
            package='rcraicer_description',
            executable='joint_states',
            name='joint_states',
            output='screen',
        ),
    ])
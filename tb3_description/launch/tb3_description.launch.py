#! /usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    xacro_file_name = "turtlebot3_waffle_base.urdf.xacro"
    urdf_file_name = "turtlebot3_waffle_base.urdf"
    urdf_file = os.path.join(
        get_package_share_directory("tb3_description"), 
        'urdf', 
        urdf_file_name
    )
    xacro_file = os.path.join(
        get_package_share_directory("tb3_description"), 
        'urdf',
        xacro_file_name
    )
    doc = xacro.process_file(xacro_file)
    robot_description = doc.toprettyxml(indent='    ')
    f = open(urdf_file, "w+")
    f.write(robot_description)
    f.close()
    return LaunchDescription([
        Node( # fixed joint 
            package='robot_state_publisher',
            executable='robot_state_publisher', 
            name="robot_state_publisher", 
            output="screen", 
            arguments=[urdf_file],
        ),
        Node( # non fixed joint
            package="joint_state_publisher", 
            executable="joint_state_publisher", 
            name="joint_state_executable", 
            output='screen',
        ),
    ])

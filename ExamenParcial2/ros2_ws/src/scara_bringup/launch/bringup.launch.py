#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    desc_share = get_package_share_directory('scara_description')
    urdf_file  = os.path.join(desc_share, 'urdf', 'scara.urdf')
    rviz_cfg   = os.path.join(desc_share, 'rviz', 'scara_rviz.rviz')

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ', urdf_file
        ])
    }

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'scara_bringup', 'initial_joint_state'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen'
        )
    ])

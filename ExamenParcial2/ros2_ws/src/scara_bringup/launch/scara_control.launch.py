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

    # xacro → robot_description
    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ', urdf_file
        ])
    }

    return LaunchDescription([
        # 1) robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),
        # 2) joint_state_publisher_gui
      #  Node(
      #      package='joint_state_publisher_gui',
      #      executable='joint_state_publisher_gui',
      #      output='screen'
      #  ),
        # 3) rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen'
        ),
        # 4) tu nodo de trayectoria (console script instalado en install/.../bin)
        ExecuteProcess(
            cmd=['scara_tray_line'],
            output='screen'
        ),
    ])

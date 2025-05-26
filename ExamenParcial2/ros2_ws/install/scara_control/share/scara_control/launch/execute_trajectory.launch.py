#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Spawnea el joint_state_broadcaster en el controller_manager de Gazebo
    js_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    # 2) Spawnea tu scara_trajectory_controller
    traj_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'scara_trajectory_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    # 3) Arranca tu nodo de trayectoria
    traj_node = Node(
        package='scara_control',
        executable='scara_tray_line',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        js_spawner,
        traj_spawner,
        traj_node,
    ])

if __name__ == '__main__':
    generate_launch_description()

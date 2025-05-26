#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Paquetes ---
    pkg_bring = get_package_share_directory('scara_bringup')
    pkg_desc  = get_package_share_directory('scara_description')
    pkg_gz    = get_package_share_directory('gazebo_ros')
    pkg_control = get_package_share_directory('scara_control')

    # --- Rutas a ficheros ---
    urdf_file  = os.path.join(pkg_desc,   'urdf',   'scara.urdf')
    rviz_cfg   = os.path.join(pkg_desc,   'rviz',   'scara_rviz.rviz')
    world_file = os.path.join(pkg_bring,  'worlds', 'scara_world.sdf')
    controllers_file = os.path.join(pkg_control, 'config', 'scara_controllers.yaml')

    # --- robot_description via xacro ---
    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ', urdf_file
        ])
    }

    # 1) Gazebo con tu world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':   world_file,
            'verbose': 'true'
        }.items()
    )

    # 2) Publica estado inicial y TF
    initial_state = Node(
        package='scara_bringup',
        executable='initial_joint_state',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    # 3) Spawn del robot en Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'scara'],
        output='screen'
    )

    # 4) Lanza los spawners para los controladores
    js_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[controllers_file],
        output='screen'
    )
    traj_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_trajectory_controller'],
        parameters=[controllers_file],
        output='screen'
    )

    # 5) RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        initial_state,
        rsp,
        spawn_entity,
        js_spawner,
        traj_spawner,
        rviz,
    ])

if __name__ == '__main__':
    generate_launch_description()

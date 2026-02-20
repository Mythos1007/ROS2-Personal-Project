#!/usr/bin/env python3
"""
Gazebo 미로 환경에서 터틀봇3 실행 Launch 파일
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 기존 Gazebo 프로세스 종료
    kill_gazebo = ExecuteProcess(
        cmd=['killall', '-9', 'gzserver', 'gzclient'],
        output='screen',
        on_exit=None,  # 실패해도 계속 진행
    )

    # 패키지 경로
    pkg_turtlebot_gui = get_package_share_directory('turtlebot_gui_control')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch 인자
    world_file = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')

    # 미로 월드 파일 경로 (기본값: maze.sdf)
    
    default_world_path = os.path.join(pkg_turtlebot_gui, 'maze.sdf')
    #default_world_path = os.path.join(pkg_turtlebot_gui, 'maze_7x7.sdf')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Full path to world model file to load'
    )

    declare_x_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position of the robot'
    )

    declare_y_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position of the robot'
    )

    # Gazebo 실행
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # TurtleBot3 모델 설정 (burger, waffle, waffle_pi)
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    # TurtleBot3 spawn
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-database', 'turtlebot3_' + turtlebot3_model,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen',
    )

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='turtlebot3_gazebo',
        executable='turtlebot3_drive',
        output='screen',
    )

    # TurtleBot GUI Control
    gui_control_cmd = Node(
        package='turtlebot_gui_control',
        executable='turtlebot_gui_qt',
        output='screen',
    )

    ld = LaunchDescription()

    # 기존 Gazebo 종료
    ld.add_action(kill_gazebo)

    # Launch arguments
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)

    # Gazebo (기존 프로세스 종료 후 1초 뒤 시작)
    ld.add_action(
        TimerAction(
            period=1.0,
            actions=[gzserver_cmd, gzclient_cmd]
        )
    )

    # TurtleBot3 (Gazebo 시작 후 4초 뒤 spawn)
    ld.add_action(
        TimerAction(
            period=5.0,
            actions=[spawn_turtlebot_cmd]
        )
    )

    # GUI Control (로봇 spawn 후 1초 뒤 실행)
    ld.add_action(
        TimerAction(
            period=6.0,
            actions=[gui_control_cmd]
        )
    )

    return ld

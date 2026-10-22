#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_name = 'odom_pkg'
    pkg_share = get_package_share_directory(pkg_name)

    urdf_file = os.path.join(pkg_share, 'urdf', 'diff_bot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # world_file_name = 'pista_robocar.sdf'

    # world_path = os.path.join(pkg_share, 'world', world_file_name)

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}]
    )

    hw_interface_node = Node(
        package=pkg_name,
        executable='hw_interface.py',
        name='hw_interface',
        output='screen'
    )

    movement_node = Node(
        package=pkg_name,
        executable='just_move.py',
        name='just_move',
        output='screen'
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_desc,
                   '-name', 'diff_bot',
                   '-allow_renaming', 'true',
                   '-z', '0.2'],
        output='screen'
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    teleop_node = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2 run teleop_twist_keyboard teleop_twist_keyboard'],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        gazebo,
        spawn_entity,
        bridge_node,
        rviz_node,
        hw_interface_node,
        movement_node
    ])
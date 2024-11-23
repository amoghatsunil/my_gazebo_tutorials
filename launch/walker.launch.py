"""
@file bag.launch.py
@brief Launch file to record talker node.

This py launch file launches the talker/publisher node to publish to "topic" which can optionally be recorded.

@details
- The 'enable_recording' argument is used to control whether the bag recording is enabled.
- The talker node from the 'beginner_tutorials' package is launched.
- The recording process and publisher node are automatically terminated after 15 seconds using a TimerAction.

@copyright
Copyright (c) 2023 Amogha Sunil. All rights reserved.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os


def generate_launch_description():
    bag_dir = os.path.join(os.getcwd(), "results")
    os.makedirs(bag_dir, exist_ok=True)

    # Declare enable_recording argument
    enable_recording_arg = DeclareLaunchArgument(
        'enable_recording',
        default_value='True',
        description='Enable or disable ROS 2 bag recording'
    )

    walker_node = Node(
        package='walker',
        executable='walker_node',
        name='walker',
        output='screen'
    )

    # Conditionally start ros2 bag record for all topics
    bag_record_process = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            [LaunchConfiguration('enable_recording')])),
        cmd=['ros2', 'bag', 'record', '-a', '-o','/camera/.*', '--output',
             os.path.join(bag_dir, 'rosbag')],
        name='rosbag_record',
        shell=False
    )

    # Timer to stop the recording and publisher after 15 seconds
    timer_action = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=['pkill', '-9', '-f', 'ros2.bag.record'],
                shell=False
            ),
            Shutdown(reason='30 seconds elapsed, shutting down nodes.')
        ]
    )

    return LaunchDescription([
        enable_recording_arg,
        walker_node,
        bag_record_process,
        timer_action
    ])

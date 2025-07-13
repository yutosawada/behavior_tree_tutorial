#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Starting BehaviorTree Tutorial 04: Reactive and Asynchronous behaviors"),
        
        Node(
            package='behavior_tree_tutorial',
            executable='tutorial_04_sequence',
            name='behavior_tree_tutorial_04',
            output='screen',
            parameters=[],
            remappings=[],
        ),
        
        LogInfo(msg="BehaviorTree Tutorial 04 launch file loaded successfully"),
    ])

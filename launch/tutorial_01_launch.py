#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Starting BehaviorTree Tutorial 01: First Tree"),
        
        Node(
            package='behavior_tree_tutorial',
            executable='tutorial_01_first_tree',
            name='behavior_tree_tutorial_01',
            output='screen',
            parameters=[],
            remappings=[],
        ),
        
        LogInfo(msg="BehaviorTree Tutorial 01 launch file loaded successfully"),
    ])

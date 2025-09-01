#  노드들을 실행시키는 런치 파일
# joy node: 조이스틱의 전기 신호를 joy 메시지로 변환시켜주는 노드
# kaqu_gamepad node: joy 메시지를 소프 4팀 메시지로 변환시켜주는 노드

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    # Joy 노드 실행
    node_joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    node_kaqu_gamepad = ExecuteProcess(
        cmd=['ros2', 'run', 'kaqu_input_manager', 'kaqu_gamepad_node'],
        output='screen'
    )

    # Launch Description 반환
    return LaunchDescription([
        node_joy,
        node_kaqu_gamepad,
    ])
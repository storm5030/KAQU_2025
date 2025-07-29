from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('kaqu_gazebo_sim')
    xacro_file = PathJoinSubstitution([pkg_share, 'description', 'kaqu.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'kaqu_model.rviz'])

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([
        # Joint State Publisher GUI 추가
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # rviz2 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])

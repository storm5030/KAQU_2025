from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('kaqu_gazebo_sim')
    xacro_file = os.path.join(pkg_path, 'description', 'kaqu.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'kaqu_ground.world')

    # xacro → URDF
    robot_description = xacro.process_file(xacro_file).toxml()

    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Gazebo 실행
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-r'],
        output='screen'
    )

    # ros_gz_bridge 실행
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen'
    )

    # 로봇 스폰 (딜레이: 1초)
    spawn_entity = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', 'robot_description', '-name', 'kaqu', '-x', '0', '-y', '0', '-z', '0.3'],
                output='screen'
            )
        ]
    )

    # 컨트롤러 스폰 (딜레이: 2초)
    spawn_controllers = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_group_position_controller'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gz_sim,
        spawn_entity,
        spawn_controllers,
        imu_bridge
    ])

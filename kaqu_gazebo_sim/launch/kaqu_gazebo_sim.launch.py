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

    # (1) xacro → urdf
    robot_description = xacro.process_file(xacro_file).toxml()

    # (2) 경사로/계단 SDF 경로
    ramp_sdf   = os.path.join(pkg_path, 'models', 'ramp', 'model.sdf')
    stairs_sdf = os.path.join(pkg_path, 'models', 'stairs', 'model.sdf')

    # (3) robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # (4) Gazebo 실행 (Fortress: ign gazebo)
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-r'],
        output='screen'
    )

    # (5) 로봇 스폰 (딜레이: 1.0s)
    spawn_entity = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'kaqu',
                    '-x', '0', '-y', '0', '-z', '0.3'
                ],
                output='screen'
            )
        ]
    )

    # (6) 환경 스폰: 경사로 (딜레이: 1.2s)
    spawn_ramp = TimerAction(
        period=1.2,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', ramp_sdf,
                    '-name', 'ramp15',
                    '-x', '0.0', '-y', '1.5', '-z', '0.0'
                    # 필요시 Yaw 회전: '-Y', '1.5708'  # (라디안)
                ],
                output='screen'
            )
        ]
    )

    # (7) 환경 스폰: 3단 계단 (딜레이: 1.4s)
    spawn_stairs = TimerAction(
        period=1.4,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', stairs_sdf,
                    '-name', 'stairs3',
                    '-x', '1.0', '-y', '0.0', '-z', '0.0', '-Y', '-1.5708'
                ],
                output='screen'
            )
        ]
    )

    # (8) 컨트롤러 스폰 (딜레이: 2.0s)
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

    # (9) 브리지
    imu_and_contacts_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            # IMU 브리지
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',

            # Contact 센서 브리지 (GZ → ROS)
            '/world/default/model/kaqu/link/fl_leg4_1/sensor/fl_foot_contact/contact@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts',
            '/world/default/model/kaqu/link/fr_leg4_1/sensor/fr_foot_contact/contact@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts',
            '/world/default/model/kaqu/link/rl_leg4_1/sensor/rl_foot_contact/contact@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts',
            '/world/default/model/kaqu/link/rr_leg4_1/sensor/rr_foot_contact/contact@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts',

            # ROS 리맵 규칙 추가
            '--ros-args',
            '-r', '/world/default/model/kaqu/link/fl_leg4_1/sensor/fl_foot_contact/contact:=/contact/fl',
            '-r', '/world/default/model/kaqu/link/fr_leg4_1/sensor/fr_foot_contact/contact:=/contact/fr',
            '-r', '/world/default/model/kaqu/link/rl_leg4_1/sensor/rl_foot_contact/contact:=/contact/rl',
            '-r', '/world/default/model/kaqu/link/rr_leg4_1/sensor/rr_foot_contact/contact:=/contact/rr',
        ],
    )


    return LaunchDescription([
        robot_state_publisher_node,
        gz_sim,
        spawn_entity,
        # 새로 추가된 환경 스포너들
        spawn_ramp,
        spawn_stairs,
        spawn_controllers,
        imu_and_contacts_bridge
    ])

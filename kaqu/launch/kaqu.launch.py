from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 각 패키지 경로 지정
    # gazebo_pkg = FindPackageShare('kaqu_gazebo_sim')
    controller_pkg = FindPackageShare('kaqu_controller')
    input_pkg = FindPackageShare('kaqu_input_manager')
    hardware_pkg = FindPackageShare('kaqu_hardware_interfacing')

    # 1. Gazebo 시뮬레이션
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([gazebo_pkg, 'launch', 'kaqu_gazebo_sim.launch.py'])
    #     )
    # )

    # 2. Controller 노드 1 (보행 제어)
    controller_node1 = Node(
        package='kaqu_controller',
        executable='RobotManagerNode',
        name='RobotManagerNode',
        output='screen'
    )

    # 3. Controller 노드 2 (각도)
    controller_node2 = Node(
        package='kaqu_controller',
        executable='AnglePulbisherNode',
        name='AnglePulbisherNode',
        output='screen'
    )

    # 4. Input Manager (조이스틱 GUI)
    input_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([input_pkg, 'launch', 'kaqu_teleop.launch.py'])
        )
    )

    hardware_node = Node(
        package='kaqu_hardware_interfacing',
        executable='bulk_read_write',
        name='HardWareNode',
    output='screen'
    )

    return LaunchDescription([
        # gazebo_launch,
        controller_node1,
        controller_node2,
        input_launch,
        hardware_node
    ])

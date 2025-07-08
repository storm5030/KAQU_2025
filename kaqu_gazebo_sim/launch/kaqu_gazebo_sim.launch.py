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
    ])




# import os
# from launch import LaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PythonExpression
# from launch_ros.actions import Node
# from launch.event_handlers import OnProcessExit
# import xacro

# # robot description 생성
# pkg_kaqu_gazebo = 'kaqu_gazebo_sim'
# robot_description_subpath = 'description/kaqu.urdf.xacro'
# xacro_file = os.path.join(get_package_share_directory(pkg_kaqu_gazebo), robot_description_subpath)
# robot_description_raw = xacro.process_file(xacro_file).toxml()

# # gazebo 관련 경로 설정
# pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
# pkg_kaqu_gazebo = get_package_share_directory('kaqu_gazebo_sim')
# world_file_name = 'kaqu_ground.world'
# world_path = os.path.join(pkg_kaqu_gazebo, 'worlds', world_file_name)

# #world_path = os.path.join(pkg_kaqu_gazebo, 'worlds', 'contact.world')

# def generate_launch_description():

#     headless = LaunchConfiguration('headless')
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     use_simulator = LaunchConfiguration('use_simulator')
#     world = LaunchConfiguration('world')

#     declare_headless_cmd = DeclareLaunchArgument(
#         name='headless',
#         default_value='False',
#         description='gzclient 실행 여부')

#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         name='use_sim_time',
#         default_value='True',
#         description='sim time 사용 여부')

#     declare_use_simulator_cmd = DeclareLaunchArgument(
#         name='use_simulator',
#         default_value='True',
#         description='Gazebo 시뮬레이터 실행 여부')

#     declare_world_cmd = DeclareLaunchArgument(
#         name='world',
#         default_value=world_path,
#         description='로드할 월드 모델 파일의 전체 경로'
#     )
#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{'robot_description': robot_description_raw,
#                      'use_sim_time': True}])

#     robot_spawn_node = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=[
#             '-topic', 'robot_description',
#             '-entity', 'kaqu',
#             '-x', '0', '-y', '0', '-z', '0.2'
#         ],
#         output='screen')

#     controller_manager = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         parameters=[{'robot_description': robot_description_raw,
#                      'use_sim_time': True}],
#         output='screen')

#     load_joint_state_broadcaster = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
#         output='screen')

#     load_joint_trajectory_position_controller = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['joint_group_position_controller', '--controller-manager', '/controller_manager'],
#         output='screen')

#     start_gazebo_server_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
#         condition=IfCondition(use_simulator),
#         launch_arguments={'world': world, 'pause': 'true'}.items())


#     start_gazebo_client_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
#         condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

#     return LaunchDescription([
#         declare_headless_cmd,
#         declare_use_sim_time_cmd,
#         declare_use_simulator_cmd,
#         declare_world_cmd,

#         start_gazebo_server_cmd,
#         start_gazebo_client_cmd,

#         robot_state_publisher_node,
#         robot_spawn_node,

#         RegisterEventHandler(
#             OnProcessExit(
#                 target_action=robot_spawn_node,
#                 on_exit=[controller_manager]
#             )
#         ),
#         RegisterEventHandler(
#             OnProcessExit(
#                 target_action=controller_manager,
#                 on_exit=[
#                     load_joint_state_broadcaster,
#                     load_joint_trajectory_position_controller
#                 ]
#             )
#         )
#     ])

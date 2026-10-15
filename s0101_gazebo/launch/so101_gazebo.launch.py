import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _spawn_robot(package_name, entity_name, x, y, z):
    package_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(package_share, 'urdf', 'so101.urdf')

    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', entity_name,
            '-file', urdf_path,
            # Mounting offset lives in the world->base_link fixed joint in the
            # URDF, so spawn at the origin (do NOT re-apply -0.55/0/0.7774).
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
            '-timeout', '30',
        ],
    )


def _robot_state_publisher(package_name):
    package_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(package_share, 'urdf', 'so101.urdf')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        urdf_content = f.read()

    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': True,
        }],
        output='screen',
    )


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gazebo_share = get_package_share_directory('so101_gazebo')
    world_path = os.path.join(
        gazebo_share,
        'worlds',
        'empty_world.world'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_share,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'false',
        }.items(),
    )

    rsp = _robot_state_publisher('so101_gazebo')

    spawn_follower = _spawn_robot(
        'so101_gazebo',
        'so101_follower',
        0.0,
        0.0,
        0.0
    )

    # Controller spawners. These talk to the controller_manager that the
    # gazebo_ros2_control plugin starts INSIDE Gazebo, so they must run only
    # after the robot is spawned. Sequence them with event handlers to avoid
    # "controller_manager not available" races.

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen',
    )

    # spawn entity -> joint_state_broadcaster -> arm_controller -> gripper_controller
    load_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_follower,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    load_gripper_after_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn_follower,
        load_jsb_after_spawn,
        load_arm_after_jsb,
        load_gripper_after_arm,
    ])

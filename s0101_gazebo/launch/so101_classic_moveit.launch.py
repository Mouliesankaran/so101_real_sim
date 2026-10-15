from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('so101_gazebo'), 'launch', 'so101_gazebo.launch.py'])))

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('so101_moveit_config'), 'launch', 'move_group.launch.py'])),
        launch_arguments={
            'use_gazebo': 'false',   # plain URDF for MoveIt; Gazebo has its own
            'use_camera': 'false',
            'use_sim_time': 'true',
            'use_rviz': 'true',
        }.items())

    # start MoveIt after Gazebo + controllers are up
    return LaunchDescription([gazebo, TimerAction(period=15.0, actions=[moveit])])

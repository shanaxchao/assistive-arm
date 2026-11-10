"""Start the UR5e pickup demo with Gazebo and MoveIt."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    demo_dir = Path(__file__).resolve().parent
    gui = LaunchConfiguration('gui')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ur_simulation_gz'), 'launch', 'ur_sim_control.launch.py',
        ])),
        launch_arguments={
            'ur_type': 'ur5e',
            'description_file': str(demo_dir / 'pickup_gripper.urdf.xacro'),
            'launch_rviz': 'false',
            'gazebo_gui': gui,
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py',
        ])),
        launch_arguments={
            'ur_type': 'ur5e',
            'use_sim_time': 'true',
            'launch_rviz': gui,
        }.items(),
    )
    gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'pickup_gripper', '--param-file', str(demo_dir / 'pickup_gripper.yaml'),
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        simulation, moveit, gripper,
    ])

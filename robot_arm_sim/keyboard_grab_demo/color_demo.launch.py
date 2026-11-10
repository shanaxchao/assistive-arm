"""Start the existing pickup robot and automatically spawn three colored cubes."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    directory = Path(__file__).resolve().parent
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(directory / 'pickup.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items(),
        ),
        ExecuteProcess(
            cmd=['python3', str(directory / 'color_demo.py'), '--spawn-only'],
            output='screen',
        ),
    ])

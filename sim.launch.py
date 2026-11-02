from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---------------------------------------------------------
    # Files
    # ---------------------------------------------------------

    xacro_file = PathJoinSubstitution([
        FindPackageShare('robot_arm_sim'),
        'urdf',
        'arm.urdf.xacro'
    ])

    controller_yaml = PathJoinSubstitution([
        FindPackageShare('robot_arm_sim'),
        'config',
        'controllers.yaml'
    ])

    # ---------------------------------------------------------
    # Convert Xacro -> URDF
    # ---------------------------------------------------------

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file,
        ' ',
        'controller_yaml:=',
        controller_yaml
    ])

    robot_description = {
        'robot_description': robot_description_content
    }

    # ---------------------------------------------------------
    # Robot State Publisher
    # ---------------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ]
    )

    # ---------------------------------------------------------
    # Gazebo
    # ---------------------------------------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r -v 2 empty.sdf'
        }.items()
    )

    # ---------------------------------------------------------
    # Bridge Gazebo simulation clock -> ROS
    # ---------------------------------------------------------

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Spawn robot into Gazebo
    # ---------------------------------------------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            'six_dof_arm',
            '-allow_renaming',
            'false'
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Controllers
    # ---------------------------------------------------------

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # Start joint state broadcaster after robot exists
    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner
            ]
        )
    )

    # Start trajectory controller after joint state broadcaster
    start_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                arm_controller_spawner
            ]
        )
    )

    return LaunchDescription([
        gazebo,

        clock_bridge,

        robot_state_publisher,

        start_joint_state_broadcaster,

        start_arm_controller,

        spawn_robot,
    ])

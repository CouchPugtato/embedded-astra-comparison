from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    seed = LaunchConfiguration('seed')
    bringup_share = FindPackageShare('tabletop_sim')
    world = PathJoinSubstitution([bringup_share, 'worlds', 'tabletop.sdf'])
    controllers = PathJoinSubstitution([bringup_share, 'config', 'controllers.yaml'])

    moveit_config = (
        MoveItConfigsBuilder('panda_sim', package_name='tabletop_sim')
        .robot_description(
            file_path='config/panda_sim.urdf.xacro',
            mappings={'controllers_file': controllers},
        )
        .robot_description_semantic(file_path='config/panda_sim.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r -v 3 --seed ', seed, ' ', world]}.items(),
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, {'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'panda', '-allow_renaming', 'false'],
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['panda_arm_controller', '--controller-manager', '/controller_manager'],
    )
    hand_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['panda_hand_controller', '--controller-manager', '/controller_manager'],
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_spawner])
    )
    activate_controllers = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_spawner, on_exit=[arm_spawner, hand_spawner])
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
        # MoveIt 2.12 always tries to initialize its optional OctoMap monitor.
        # This RGB-only simulation deliberately has no depth/point-cloud plugin.
        arguments=['--ros-args', '--log-level', 'moveit.ros.occupancy_map_monitor:=fatal'],
    )

    scene_setup = Node(
        package='tabletop_sim',
        executable='scene_setup',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='1'),
        # Render through WSLg's D3D12 backend on the requested GPU.
        SetEnvironmentVariable('MESA_D3D12_DEFAULT_ADAPTER_NAME', 'NVIDIA GeForce RTX 5080'),
        gazebo,
        bridge,
        state_publisher,
        spawn_robot,
        spawn_controllers,
        activate_controllers,
        move_group,
        scene_setup,
    ])

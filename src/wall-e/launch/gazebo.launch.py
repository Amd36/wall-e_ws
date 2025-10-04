from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_path = PathJoinSubstitution([
        FindPackageShare('wall-e'),
        'worlds',
        'empty.sdf'
    ])

    urdf_path = PathJoinSubstitution([
        FindPackageShare('wall-e'),
        'urdf',
        'robot.urdf'
    ])

    bridge_params = os.path.join(
        get_package_share_directory('wall-e'),
        'config',
        'ros_gz_bridge_config.yaml'
    )

    # Include Gazebo simulation launch
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': world_path
        }.items()
    )

    # Spawn robot in simulation
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-file', urdf_path,
            '-name', 'wall-e',
            '-x', '0', '-y', '0', '-z', '0.02'
        ]
    )

    # Include ROS 2 to Gazebo bridge
    ros_gz_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='robot_ros_gz_bridge',
        output='screen',
        arguments=[
        '--ros-args',
        '-p',
        f'config_file:={bridge_params}'
        ],
    )

    ld = LaunchDescription()
    
    ld.add_action(gz_sim_launch)
    ld.add_action(spawn_robot)
    ld.add_action(ros_gz_bridge_cmd)

    return ld

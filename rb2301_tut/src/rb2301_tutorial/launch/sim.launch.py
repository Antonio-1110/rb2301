from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # initialize launch description
    ld = LaunchDescription()

    # 1. PACKAGE INSTALLATION DIRECTORIES
    pkg_rb2301_bringup = FindPackageShare('rb2301_bringup')

    # 2. LAUNCH ARGUMENTS
    world = DeclareLaunchArgument(
    'world', 
    default_value='test.sdf',
    description='Name of the Gazebo world file to load. Must be in rb2301_bringup/worlds/'
    )
    ld.add_action(world)
    
    # 3. LAUNCH OTHER LAUNCH FILES
    launch_sim_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            pkg_rb2301_bringup, 'launch', 'sim.launch.py'
        ])
    ]),
    launch_arguments={
        'world': LaunchConfiguration('world'),
    }.items()
    )
    ld.add_action(launch_sim_launch)
    
    # 4. RUN NODES
    sim = Node(
    package='rb2301_tutorial',
    executable='sim',
    output='screen',
    emulate_tty=True,
    )
    ld.add_action(sim)

    return ld

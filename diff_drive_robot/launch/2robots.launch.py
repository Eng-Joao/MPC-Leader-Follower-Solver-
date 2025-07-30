import os
import numpy as np
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, LogInfo, ExecuteProcess

def generate_launch_description():
    # Package name
    package_name = 'diff_drive_robot'
    
    # Launch configurations
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    
    # Path to default world
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world')
    
    # Launch Arguments
    declare_world = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')
    
    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Opens rviz is set to True')
    
    # URDF path
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'differential_robot.xacro')
    
    # Verify that the XACRO file can be processed with namespaces
    verify_xacro = ExecuteProcess(
        cmd=['xacro', urdf_path, 'namespace:=robot1'],
        output='screen',
        shell=False,
        # Don't actually run this - just check if it compiles
        emulate_tty=True
    )
    
    # Launch the gazebo server to initialize the simulation
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    
    # Always launch the gazebo client to visualize the simulation
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), launch_arguments={'gz_args': '-g '}.items()
    )
    
    # Log info about namespaces
    log_info = LogInfo(
        msg=["Launching multi-robot system with namespaced robots:", 
             "- robot1: Will use /robot1/cmd_vel for control", 
             "- robot2: Will use /robot2/cmd_vel for control"]
    )
    
    # ===== Robot 1 configuration =====
    # Robot State Publisher for robot1
    robot1_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), 
        launch_arguments={
            'use_sim_time': 'true', 
            'urdf': urdf_path,
            'namespace': 'robot1'
        }.items()
    )
    
    # Spawn robot1
    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot1/robot_description',
            '-name', 'robot1',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
        ],
        output='screen'
    )
    
    # ===== Robot 2 configuration =====
    # Robot State Publisher for robot2
    robot2_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), 
        launch_arguments={
            'use_sim_time': 'true', 
            'urdf': urdf_path,
            'namespace': 'robot2'
        }.items()
    )
    
    # Spawn robot2
    spawn_robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot2/robot_description',
            '-name', 'robot2',
            '-x', '4.0',
            '-y', '-4.0',
            '-z', '0.2',
            '-Y', str(-np.pi / 2)
        ],
        output='screen'
    )
    
    # Launch the Gazebo-ROS bridge
    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge2.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    
    # Launch Rviz with multi-robot config file
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'multi_robot.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config_file],
                output='screen',
            )
        ]
    )
    
    # Launch them all!
    return LaunchDescription([
        # Declare launch arguments
        declare_rviz,
        declare_world,
        log_info,
        
        # Launch the nodes
        gazebo_server,
        gazebo_client,
        
        # Robot 1
        robot1_rsp,
        spawn_robot1,
        
        # Robot 2
        robot2_rsp,
        spawn_robot2,
        
        # Common components
        ros_gz_bridge,
        #rviz2,
    ])
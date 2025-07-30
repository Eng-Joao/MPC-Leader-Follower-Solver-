from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

def generate_launch_description():
    # Package name
    package_name = FindPackageShare("diff_drive_robot")
    
    # Default robot description if none is specified
    urdf_path = PathJoinSubstitution([package_name, "urdf", "differential_robot.xacro"])
    
    # Launch configurations
    urdf = LaunchConfiguration('urdf')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use sim time if true')
    
    declare_urdf = DeclareLaunchArgument(
        name='urdf', default_value=urdf_path,
        description='Path to the robot description file')
    
    declare_namespace = DeclareLaunchArgument(
        name='namespace', default_value='',
        description='Namespace for the robot')
    
    # Create a robot state publisher with namespace support
    robot_state_publisher = GroupAction([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                # Pass the namespace to the xacro file
                'robot_description': Command(['xacro ', urdf, ' namespace:=', namespace]),
                # Add this line to fix TF namespacing
                'frame_prefix': [namespace, '/']  # This will result in "robot1/" or "robot2/"
            }],
        )
    ])
    
    # Launch!
    return LaunchDescription([
        declare_urdf,
        declare_use_sim_time,
        declare_namespace,
        robot_state_publisher
    ])
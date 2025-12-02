"""Main launch file for ROS 2 CAN Bridge with native SocketCAN support."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('mes1_system')
    config_dir = os.path.join(pkg_dir, 'config')
    
    # Launch arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='SocketCAN interface name'
    )
    
    node_id_arg = DeclareLaunchArgument(
        'node_id',
        default_value='21',
        description='CANopen node ID'
    )
    
    bitrate_arg = DeclareLaunchArgument(
        'bitrate',
        default_value='1000000',
        description='CAN bus bitrate'
    )
    
    setup_can_arg = DeclareLaunchArgument(
        'setup_can',
        default_value='true',
        description='Automatically setup CAN interface'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(config_dir, 'can_bridge.yaml'),
        description='Path to configuration file'
    )
    
    # Setup CAN interface
    setup_can = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('setup_can')),
        cmd=[
            'bash', '-c', [
                'sudo ip link set ', LaunchConfiguration('can_interface'), 
                ' type can bitrate ', LaunchConfiguration('bitrate'), 
                ' && sudo ip link set ', LaunchConfiguration('can_interface'), 
                ' txqueuelen 1000 && sudo ip link set ', LaunchConfiguration('can_interface'), ' up'
            ]
        ],
        shell=True,
        output='screen'
    )
    
    # Main bridge node
    bridge_node = Node(
        package='ros_can_bridge_native',  # Executables are under this name
        executable='socketcan_bridge_node',
        name='can_bridge_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'can_interface': LaunchConfiguration('can_interface'),
                'node_id': LaunchConfiguration('node_id'),
                'bitrate': LaunchConfiguration('bitrate'),
                'log_level': PythonExpression([
                    '"DEBUG" if "', LaunchConfiguration('debug'), '" == "true" else "INFO"'
                ])
            }
        ],
        respawn=True,
        respawn_delay=2.0
    )
    
    # Diagnostics node
    diagnostics_node = Node(
        package='ros_can_bridge_native',  # Executables are under this name
        executable='diagnostics_publisher',
        name='can_diagnostics',
        output='screen',
        parameters=[{
            'update_rate': 1.0
        }],
        respawn=True
    )
    
    return LaunchDescription([
        can_interface_arg,
        node_id_arg,
        bitrate_arg,
        setup_can_arg,
        debug_arg,
        config_file_arg,
        setup_can,
        bridge_node,
        diagnostics_node
    ])

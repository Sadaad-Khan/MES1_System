"""
Launch file for multi-node CANopen relay network.

This launch file starts the multi-node CAN bridge that automatically discovers
and manages multiple CANopen relay nodes on a single CAN bus.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for multi-node CAN bridge."""
    
    # Package share directory
    pkg_share = FindPackageShare('mes1_system')
    
    # Default config file path
    default_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'multi_node_map.yaml'
    ])
    
    # Launch arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name (e.g., can0, vcan0)'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to multi-node configuration YAML file'
    )
    
    bitrate_arg = DeclareLaunchArgument(
        'bitrate',
        default_value='1000000',
        description='CAN bus bitrate (default: 1 Mbps)'
    )
    
    heartbeat_timeout_arg = DeclareLaunchArgument(
        'heartbeat_timeout',
        default_value='10.0',
        description='Heartbeat timeout in seconds'
    )
    
    auto_discover_arg = DeclareLaunchArgument(
        'auto_discover',
        default_value='true',
        description='Enable automatic node discovery'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # Multi-node bridge node
    multi_node_bridge = Node(
        package='ros_can_bridge_native',
        executable='multi_node_bridge',
        name='multi_node_can_bridge',
        output='screen',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'config_file': LaunchConfiguration('config_file'),
            'bitrate': LaunchConfiguration('bitrate'),
            'heartbeat_timeout': LaunchConfiguration('heartbeat_timeout'),
            'auto_discover': LaunchConfiguration('auto_discover'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        respawn=True,
        respawn_delay=2.0
    )
    
    # Command client node (for interactive control)
    command_client = Node(
        package='ros_can_bridge_native',
        executable='multi_node_command_client',
        name='multi_node_command_client',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
        }],
        respawn=False  # Don't auto-restart interactive client
    )
    
    return LaunchDescription([
        # Launch arguments
        can_interface_arg,
        config_file_arg,
        bitrate_arg,
        heartbeat_timeout_arg,
        auto_discover_arg,
        log_level_arg,
        
        # Nodes
        multi_node_bridge,
        # command_client,  # Commented out - start manually for interactive use
    ])

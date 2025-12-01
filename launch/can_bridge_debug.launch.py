"""Debug launch file with verbose logging and monitoring tools."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ros_can_bridge_native')
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
    
    # Main bridge node with debug logging
    bridge_node = Node(
        package='ros_can_bridge_native',
        executable='socketcan_bridge_node',
        name='can_bridge_node',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'can_bridge.yaml'),
            {
                'can_interface': LaunchConfiguration('can_interface'),
                'node_id': LaunchConfiguration('node_id'),
                'log_level': 'DEBUG',
                'log_can_frames': True,
                'publish_all_frames': True
            }
        ],
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )
    
    # Diagnostics node
    diagnostics_node = Node(
        package='ros_can_bridge_native',
        executable='diagnostics_publisher',
        name='can_diagnostics',
        output='screen',
        parameters=[{
            'update_rate': 5.0  # Faster updates for debugging
        }],
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )
    
    return LaunchDescription([
        can_interface_arg,
        node_id_arg,
        bridge_node,
        diagnostics_node
    ])

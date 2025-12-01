#!/usr/bin/env python3
"""
Multi-Node SocketCAN Bridge for ROS 2.

Production-grade CAN bridge with multi-node discovery, tracking, and management.
Supports distributed CANopen relay networks with automatic node registration.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import can
import threading
import queue
import time
import yaml
from pathlib import Path
from collections import deque
from typing import Optional, Dict, Any, List
from dataclasses import dataclass, field
from datetime import datetime
import struct

# ROS 2 message imports
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# ============================================================================
# Node Registry and Tracking
# ============================================================================

@dataclass
class NodeInfo:
    """Information about a discovered CANopen node."""
    node_id: int
    name: str
    role: str
    description: str
    relays: List[Dict[str, Any]] = field(default_factory=list)
    supported_commands: List[int] = field(default_factory=list)
    
    # Runtime state
    last_heartbeat: Optional[datetime] = None
    heartbeat_count: int = 0
    is_online: bool = False
    state_bitmap: int = 0
    
    # Diagnostics
    tx_success: int = 0
    tx_fail: int = 0
    rx_count: int = 0
    recovery_count: int = 0
    
    # Topics and services
    state_topic: str = ""
    diagnostics_topic: str = ""
    control_service: str = ""
    
    def __post_init__(self):
        """Initialize computed fields."""
        if not self.state_topic:
            self.state_topic = f"/relay_{self.name}/state"
        if not self.diagnostics_topic:
            self.diagnostics_topic = f"/relay_{self.name}/diagnostics"
        if not self.control_service:
            self.control_service = f"/relay_{self.name}/control"
    
    def update_heartbeat(self):
        """Update heartbeat timestamp."""
        self.last_heartbeat = datetime.now()
        self.heartbeat_count += 1
        self.is_online = True
    
    def check_timeout(self, timeout_seconds: float) -> bool:
        """
        Check if node has timed out.
        
        Args:
            timeout_seconds: Timeout threshold in seconds
        
        Returns:
            True if node has timed out
        """
        if self.last_heartbeat is None:
            return False
        
        elapsed = (datetime.now() - self.last_heartbeat).total_seconds()
        if elapsed > timeout_seconds:
            self.is_online = False
            return True
        return False
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for publishing."""
        return {
            'node_id': self.node_id,
            'name': self.name,
            'role': self.role,
            'is_online': self.is_online,
            'heartbeat_count': self.heartbeat_count,
            'last_heartbeat': self.last_heartbeat.isoformat() if self.last_heartbeat else None,
            'state_bitmap': self.state_bitmap,
            'tx_success': self.tx_success,
            'tx_fail': self.tx_fail,
            'rx_count': self.rx_count,
            'recovery_count': self.recovery_count
        }


class NodeRegistry:
    """Registry for managing multiple CANopen nodes."""
    
    def __init__(self):
        """Initialize node registry."""
        self.nodes: Dict[int, NodeInfo] = {}  # node_id -> NodeInfo
        self.nodes_by_name: Dict[str, NodeInfo] = {}  # name -> NodeInfo
        self.lock = threading.Lock()
    
    def register_node(self, node_info: NodeInfo):
        """
        Register a new node.
        
        Args:
            node_info: Node information
        """
        with self.lock:
            self.nodes[node_info.node_id] = node_info
            self.nodes_by_name[node_info.name] = node_info
    
    def get_node(self, node_id: int) -> Optional[NodeInfo]:
        """
        Get node by ID.
        
        Args:
            node_id: CANopen node ID
        
        Returns:
            NodeInfo or None if not found
        """
        with self.lock:
            return self.nodes.get(node_id)
    
    def get_node_by_name(self, name: str) -> Optional[NodeInfo]:
        """
        Get node by name.
        
        Args:
            name: Node name
        
        Returns:
            NodeInfo or None if not found
        """
        with self.lock:
            return self.nodes_by_name.get(name)
    
    def get_all_nodes(self) -> List[NodeInfo]:
        """
        Get all registered nodes.
        
        Returns:
            List of NodeInfo objects
        """
        with self.lock:
            return list(self.nodes.values())
    
    def update_heartbeat(self, node_id: int):
        """
        Update heartbeat for node.
        
        Args:
            node_id: Node ID
        """
        with self.lock:
            if node_id in self.nodes:
                self.nodes[node_id].update_heartbeat()
    
    def check_timeouts(self, timeout_seconds: float) -> List[NodeInfo]:
        """
        Check for timed out nodes.
        
        Args:
            timeout_seconds: Timeout threshold
        
        Returns:
            List of timed out nodes
        """
        timed_out = []
        with self.lock:
            for node in self.nodes.values():
                if node.check_timeout(timeout_seconds):
                    timed_out.append(node)
        return timed_out
    
    def get_online_nodes(self) -> List[NodeInfo]:
        """
        Get all online nodes.
        
        Returns:
            List of online nodes
        """
        with self.lock:
            return [n for n in self.nodes.values() if n.is_online]
    
    def get_offline_nodes(self) -> List[NodeInfo]:
        """
        Get all offline nodes.
        
        Returns:
            List of offline nodes
        """
        with self.lock:
            return [n for n in self.nodes.values() if not n.is_online]


# ============================================================================
# CANopen Constants
# ============================================================================

class CANopenConstants:
    """CANopen protocol constants."""
    
    # COB-ID functions
    RSDO_BASE = 0x600  # Receive SDO base
    TSDO_BASE = 0x580  # Transmit SDO base
    HEARTBEAT_BASE = 0x700  # Heartbeat base
    
    # SDO command specifiers
    SDO_WRITE_1BYTE = 0x2F
    SDO_WRITE_2BYTE = 0x2B
    SDO_WRITE_4BYTE = 0x23
    SDO_READ = 0x40
    SDO_RESPONSE = 0x60
    SDO_ABORT = 0x80
    
    # Object dictionary addresses
    OD_CONTROL_COMMAND = 0x2000
    OD_RELAY_FEEDBACK = 0x2001
    OD_COMMAND_RESULT = 0x2002
    OD_DIAGNOSTICS = 0x2003
    OD_NODE_ROLE = 0x2004
    
    # Role identifiers
    ROLE_DRL = 1
    ROLE_SAFETY = 2
    ROLE_BATTERY = 3
    ROLE_FUTURE = 4
    
    ROLE_NAMES = {
        1: "DRL",
        2: "SAFETY",
        3: "BATTERY",
        4: "FUTURE"
    }


# ============================================================================
# Multi-Node SocketCAN Bridge
# ============================================================================

class MultiNodeSocketCANBridge(Node):
    """Multi-node SocketCAN bridge with automatic node discovery."""
    
    def __init__(self):
        super().__init__('multi_node_socketcan_bridge')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('can_interface', 'can0'),
                ('bitrate', 1000000),
                ('config_file', ''),
                ('heartbeat_timeout', 10.0),
                ('heartbeat_check_rate', 2.0),
                ('diagnostics_rate', 1.0),
                ('auto_discover', True),
            ]
        )
        
        # Get parameters
        self.can_interface = self.get_parameter('can_interface').value
        self.bitrate = self.get_parameter('bitrate').value
        self.config_file = self.get_parameter('config_file').value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self.heartbeat_check_rate = self.get_parameter('heartbeat_check_rate').value
        self.diagnostics_rate = self.get_parameter('diagnostics_rate').value
        self.auto_discover = self.get_parameter('auto_discover').value
        
        # Node registry
        self.registry = NodeRegistry()
        
        # Load configuration
        self._load_config()
        
        # CAN bus setup
        self.can_bus: Optional[can.Bus] = None
        self.can_lock = threading.Lock()
        self._setup_can_bus()
        
        # Publishers (per-node topics created dynamically)
        self.state_publishers = {}  # node_name -> publisher
        self.diagnostics_publishers = {}  # node_name -> publisher
        
        # Global network status publisher
        self.network_status_pub = self.create_publisher(
            String,
            '/can_network/status',
            10
        )
        
        # Create publishers for each registered node
        self._create_node_publishers()
        
        # Start CAN receiver thread
        self.can_rx_queue = queue.Queue(maxsize=1000)
        self.running = True
        self.rx_thread = threading.Thread(target=self._can_rx_worker, daemon=True)
        self.rx_thread.start()
        
        # Start heartbeat monitoring timer
        self.heartbeat_timer = self.create_timer(
            1.0 / self.heartbeat_check_rate,
            self._check_heartbeats
        )
        
        # Start diagnostics publishing timer
        self.diagnostics_timer = self.create_timer(
            1.0 / self.diagnostics_rate,
            self._publish_diagnostics
        )
        
        # Message processing timer
        self.msg_timer = self.create_timer(
            0.01,  # 10ms
            self._process_can_messages
        )
        
        self.get_logger().info(f'Multi-node CAN bridge started on {self.can_interface}')
        self.get_logger().info(f'Registered {len(self.registry.get_all_nodes())} nodes')
    
    def _load_config(self):
        """Load multi-node configuration from YAML."""
        if not self.config_file:
            # Try default location
            pkg_share = Path(__file__).parent.parent
            default_config = pkg_share / 'config' / 'multi_node_map.yaml'
            if default_config.exists():
                self.config_file = str(default_config)
            else:
                self.get_logger().warn('No config file specified and default not found')
                return
        
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            # Register nodes from config
            for node_cfg in config.get('nodes', []):
                # Skip disabled nodes
                if not node_cfg.get('enabled', True):
                    continue
                
                node_info = NodeInfo(
                    node_id=node_cfg['node_id'],
                    name=node_cfg['name'],
                    role=node_cfg['role'],
                    description=node_cfg.get('description', ''),
                    relays=node_cfg.get('relays', []),
                    supported_commands=node_cfg.get('supported_commands', []),
                    state_topic=node_cfg.get('topics', {}).get('state', ''),
                    diagnostics_topic=node_cfg.get('topics', {}).get('diagnostics', ''),
                    control_service=node_cfg.get('services', {}).get('control', '')
                )
                
                self.registry.register_node(node_info)
                self.get_logger().info(
                    f"Registered node: {node_info.name} "
                    f"(ID: 0x{node_info.node_id:02X}, Role: {node_info.role})"
                )
            
            # Store broadcast commands
            self.broadcast_commands = config.get('broadcast_commands', {})
            self.command_aliases = config.get('command_aliases', {})
            
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
    
    def _setup_can_bus(self):
        """Initialize CAN bus connection."""
        try:
            self.can_bus = can.Bus(
                interface='socketcan',
                channel=self.can_interface,
                bitrate=self.bitrate
            )
            self.get_logger().info(f'CAN bus initialized: {self.can_interface}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus: {e}')
            self.can_bus = None
    
    def _create_node_publishers(self):
        """Create ROS 2 publishers for each node."""
        for node in self.registry.get_all_nodes():
            # State publisher
            self.state_publishers[node.name] = self.create_publisher(
                String,  # TODO: Replace with custom RelayState message
                node.state_topic,
                10
            )
            
            # Diagnostics publisher
            self.diagnostics_publishers[node.name] = self.create_publisher(
                DiagnosticStatus,
                node.diagnostics_topic,
                10
            )
            
            self.get_logger().debug(f'Created publishers for {node.name}')
    
    def _can_rx_worker(self):
        """CAN receive worker thread."""
        while self.running and self.can_bus:
            try:
                msg = self.can_bus.recv(timeout=0.1)
                if msg:
                    self.can_rx_queue.put(msg)
            except Exception as e:
                self.get_logger().error(f'CAN RX error: {e}')
                time.sleep(0.1)
    
    def _process_can_messages(self):
        """Process incoming CAN messages."""
        processed = 0
        max_per_cycle = 50
        
        while processed < max_per_cycle:
            try:
                msg = self.can_rx_queue.get_nowait()
                self._handle_can_message(msg)
                processed += 1
            except queue.Empty:
                break
    
    def _handle_can_message(self, msg: can.Message):
        """
        Handle incoming CAN message.
        
        Args:
            msg: CAN message
        """
        can_id = msg.arbitration_id
        data = msg.data
        
        # Check if this is a heartbeat message
        if (can_id >= CANopenConstants.HEARTBEAT_BASE and 
            can_id < CANopenConstants.HEARTBEAT_BASE + 0x7F):
            
            node_id = can_id - CANopenConstants.HEARTBEAT_BASE
            self._handle_heartbeat(node_id, data)
        
        # Check if this is an SDO response (TSDO)
        elif (can_id >= CANopenConstants.TSDO_BASE and
              can_id < CANopenConstants.TSDO_BASE + 0x7F):
            
            node_id = can_id - CANopenConstants.TSDO_BASE
            self._handle_sdo_response(node_id, data)
    
    def _handle_heartbeat(self, node_id: int, data: bytes):
        """
        Handle heartbeat message.
        
        Args:
            node_id: Node ID
            data: Heartbeat data
        """
        node = self.registry.get_node(node_id)
        
        if node:
            # Known node - update heartbeat
            was_offline = not node.is_online
            self.registry.update_heartbeat(node_id)
            
            if was_offline:
                self.get_logger().info(f'Node {node.name} (0x{node_id:02X}) came online')
        
        elif self.auto_discover:
            # Unknown node - auto-discover
            self.get_logger().info(f'Discovered new node: 0x{node_id:02X}')
            
            # Query node role
            self._query_node_role(node_id)
    
    def _handle_sdo_response(self, node_id: int, data: bytes):
        """
        Handle SDO response message.
        
        Args:
            node_id: Node ID
            data: SDO data
        """
        if len(data) < 4:
            return
        
        cmd = data[0]
        index = data[1] | (data[2] << 8)
        subindex = data[3]
        
        node = self.registry.get_node(node_id)
        if not node:
            return
        
        # Relay feedback (0x2001)
        if index == CANopenConstants.OD_RELAY_FEEDBACK and len(data) >= 5:
            bitmap = data[4]
            node.state_bitmap = bitmap
            self._publish_node_state(node)
        
        # Diagnostics (0x2003)
        elif index == CANopenConstants.OD_DIAGNOSTICS and len(data) >= 8:
            node.tx_success = data[4] | (data[5] << 8)
            node.tx_fail = data[6] | (data[7] << 8)
        
        # Node role response (0x2004)
        elif index == CANopenConstants.OD_NODE_ROLE and len(data) >= 5:
            role_id = data[4]
            role_name = CANopenConstants.ROLE_NAMES.get(role_id, "UNKNOWN")
            self.get_logger().info(
                f'Node 0x{node_id:02X} reported role: {role_name}'
            )
    
    def _publish_node_state(self, node: NodeInfo):
        """
        Publish node state to ROS topic.
        
        Args:
            node: Node information
        """
        if node.name in self.state_publishers:
            # TODO: Use custom RelayState message
            msg = String()
            msg.data = f'Node {node.name}: bitmap=0x{node.state_bitmap:02X}'
            self.state_publishers[node.name].publish(msg)
    
    def _check_heartbeats(self):
        """Check for heartbeat timeouts."""
        timed_out = self.registry.check_timeouts(self.heartbeat_timeout)
        
        for node in timed_out:
            self.get_logger().warn(
                f'Node {node.name} (0x{node.node_id:02X}) heartbeat timeout'
            )
    
    def _publish_diagnostics(self):
        """Publish diagnostics for all nodes."""
        for node in self.registry.get_all_nodes():
            if node.name in self.diagnostics_publishers:
                status = DiagnosticStatus()
                status.name = f'CANopen Node: {node.name}'
                status.hardware_id = f'0x{node.node_id:02X}'
                
                if node.is_online:
                    status.level = DiagnosticStatus.OK
                    status.message = 'Node online'
                else:
                    status.level = DiagnosticStatus.ERROR
                    status.message = 'Node offline (heartbeat timeout)'
                
                # Add diagnostic values
                status.values = [
                    KeyValue(key='node_id', value=f'0x{node.node_id:02X}'),
                    KeyValue(key='role', value=node.role),
                    KeyValue(key='online', value=str(node.is_online)),
                    KeyValue(key='heartbeat_count', value=str(node.heartbeat_count)),
                    KeyValue(key='tx_success', value=str(node.tx_success)),
                    KeyValue(key='tx_fail', value=str(node.tx_fail)),
                    KeyValue(key='rx_count', value=str(node.rx_count)),
                    KeyValue(key='state_bitmap', value=f'0x{node.state_bitmap:02X}'),
                ]
                
                self.diagnostics_publishers[node.name].publish(status)
    
    def _query_node_role(self, node_id: int):
        """
        Query node role via SDO read.
        
        Args:
            node_id: Node ID
        """
        # Construct SDO read request for OD_NODE_ROLE
        sdo_data = bytes([
            CANopenConstants.SDO_READ,
            CANopenConstants.OD_NODE_ROLE & 0xFF,
            (CANopenConstants.OD_NODE_ROLE >> 8) & 0xFF,
            0x00,  # subindex
            0, 0, 0, 0
        ])
        
        can_id = CANopenConstants.RSDO_BASE + node_id
        msg = can.Message(
            arbitration_id=can_id,
            data=sdo_data,
            is_extended_id=False
        )
        
        try:
            with self.can_lock:
                if self.can_bus:
                    self.can_bus.send(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to query node role: {e}')
    
    def send_command(self, node_id: int, command: int) -> bool:
        """
        Send command to specific node.
        
        Args:
            node_id: Target node ID
            command: Command code
        
        Returns:
            True if sent successfully
        """
        # Construct SDO write request for OD_CONTROL_COMMAND
        sdo_data = bytes([
            CANopenConstants.SDO_WRITE_1BYTE,
            CANopenConstants.OD_CONTROL_COMMAND & 0xFF,
            (CANopenConstants.OD_CONTROL_COMMAND >> 8) & 0xFF,
            0x00,  # subindex
            command,
            0, 0, 0
        ])
        
        can_id = CANopenConstants.RSDO_BASE + node_id
        msg = can.Message(
            arbitration_id=can_id,
            data=sdo_data,
            is_extended_id=False
        )
        
        try:
            with self.can_lock:
                if self.can_bus:
                    self.can_bus.send(msg)
                    self.get_logger().debug(
                        f'Sent command {command} to node 0x{node_id:02X}'
                    )
                    return True
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
        
        return False
    
    def broadcast_command(self, command: int, target_nodes: Optional[List[str]] = None):
        """
        Send command to multiple nodes.
        
        Args:
            command: Command code
            target_nodes: List of node names (None = all nodes)
        """
        nodes_to_target = []
        
        if target_nodes:
            for name in target_nodes:
                node = self.registry.get_node_by_name(name)
                if node:
                    nodes_to_target.append(node)
        else:
            nodes_to_target = self.registry.get_online_nodes()
        
        self.get_logger().info(
            f'Broadcasting command {command} to {len(nodes_to_target)} nodes'
        )
        
        for node in nodes_to_target:
            self.send_command(node.node_id, command)
            time.sleep(0.01)  # Small delay between broadcasts
    
    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('Shutting down multi-node bridge...')
        self.running = False
        
        if self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
        
        if self.can_bus:
            self.can_bus.shutdown()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = MultiNodeSocketCANBridge()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

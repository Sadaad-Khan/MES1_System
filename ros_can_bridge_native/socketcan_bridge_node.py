#!/usr/bin/env python3
"""
SocketCAN Bridge Node for ROS 2.

Production-grade CAN bridge with native SocketCAN support for CANopen communication.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import can
import threading
import queue
import time
from collections import deque
from typing import Optional, Dict, Any
import struct

# ROS 2 message imports (these will be generated)
# from ros_can_bridge_native_interfaces.srv import RelayControl, CANTransmit, CANDiagnostics
# from ros_can_bridge_native_interfaces.msg import CANFrame, RelayState, CANBusStatus
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# CANopen constants
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
    
    # Relay commands
    COMMANDS = {
        0: "ALL_OFF",
        1: "K1_ON",
        2: "K1_OFF",
        3: "K2_ON",
        4: "K2_OFF",
        5: "K3_ON",
        6: "K3_OFF",
        7: "K4_ON",
        8: "K4_OFF",
        99: "ALL_ON",
        254: "SOFTWARE_RESET"
    }


class SocketCANBridgeNode(Node):
    """Main bridge node for SocketCAN <-> ROS 2 communication."""
    
    def __init__(self):
        super().__init__('socketcan_bridge_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('can_interface', 'can0'),
                ('node_id', 21),
                ('bitrate', 1000000),
                ('sdo_timeout', 3.0),
                ('heartbeat_timeout', 5.0),
                ('reconnect_delay', 2.0),
                ('enable_diagnostics', True),
                ('diagnostic_rate', 1.0),
                ('log_level', 'INFO'),
                ('enable_loopback', False),
                ('max_retry_attempts', 3),
                ('publish_all_frames', True),
                ('rx_buffer_size', 1000),
                ('tx_buffer_size', 100),
            ]
        )
        
        # Get parameters
        self.can_interface = self.get_parameter('can_interface').value
        self.node_id = self.get_parameter('node_id').value
        self.bitrate = self.get_parameter('bitrate').value
        self.sdo_timeout = self.get_parameter('sdo_timeout').value
        self.max_retry_attempts = self.get_parameter('max_retry_attempts').value
        
        # Calculate CANopen IDs
        self.rsdo_id = CANopenConstants.RSDO_BASE + self.node_id
        self.tsdo_id = CANopenConstants.TSDO_BASE + self.node_id
        self.heartbeat_id = CANopenConstants.HEARTBEAT_BASE + self.node_id
        
        # Thread-safe queues
        self.tx_queue = queue.Queue(maxsize=self.get_parameter('tx_buffer_size').value)
        self.rx_queue = queue.Queue(maxsize=self.get_parameter('rx_buffer_size').value)
        
        # SDO response tracking
        self.sdo_responses = {}
        self.sdo_lock = threading.Lock()
        
        # Statistics
        self.stats = {
            'frames_tx': 0,
            'frames_rx': 0,
            'errors_tx': 0,
            'errors_rx': 0,
            'sdo_success': 0,
            'sdo_failures': 0,
            'latency_samples': deque(maxlen=1000)
        }
        self.stats_lock = threading.Lock()
        
        # CAN bus interface
        self.can_bus: Optional[can.Bus] = None
        self.bus_connected = False
        
        # Threading
        self.running = threading.Event()
        self.running.set()
        self.rx_thread: Optional[threading.Thread] = None
        self.tx_thread: Optional[threading.Thread] = None
        
        # Relay state
        self.current_relay_state = 0
        self.relay_state_lock = threading.Lock()
        
        # Callback group for services
        self.callback_group = ReentrantCallbackGroup()
        
        # Create publishers (using std_msgs until custom messages are built)
        self.relay_state_pub = self.create_publisher(
            String, '~/relay_state', 10)
        self.bus_status_pub = self.create_publisher(
            String, '~/bus_status', 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        
        # Create service (simplified version using String until custom services are built)
        # In production, use custom RelayControl service
        # self.relay_service = self.create_service(
        #     RelayControl, '~/relay_control', 
        #     self.handle_relay_control, callback_group=self.callback_group)
        
        # Initialize CAN bus
        self.initialize_can_bus()
        
        # Start threads
        self.start_threads()
        
        # Create timer for diagnostics
        if self.get_parameter('enable_diagnostics').value:
            diag_rate = self.get_parameter('diagnostic_rate').value
            self.diagnostics_timer = self.create_timer(
                1.0 / diag_rate, self.publish_diagnostics)
        
        self.get_logger().info(
            f'SocketCAN Bridge initialized: interface={self.can_interface}, '
            f'node_id=0x{self.node_id:02X}, RSDO=0x{self.rsdo_id:03X}, '
            f'TSDO=0x{self.tsdo_id:03X}')
    
    def initialize_can_bus(self):
        """Initialize SocketCAN interface."""
        try:
            self.can_bus = can.interface.Bus(
                channel=self.can_interface,
                bustype='socketcan',
                bitrate=self.bitrate
            )
            self.bus_connected = True
            self.get_logger().info(
                f'Connected to CAN interface: {self.can_interface}')
        except Exception as e:
            self.get_logger().error(
                f'Failed to initialize CAN interface {self.can_interface}: {e}')
            self.bus_connected = False
    
    def start_threads(self):
        """Start worker threads for CAN I/O."""
        self.rx_thread = threading.Thread(
            target=self._rx_worker, daemon=True, name='CAN_RX')
        self.tx_thread = threading.Thread(
            target=self._tx_worker, daemon=True, name='CAN_TX')
        
        self.rx_thread.start()
        self.tx_thread.start()
        
        self.get_logger().info('CAN I/O threads started')
    
    def _rx_worker(self):
        """Worker thread for receiving CAN frames."""
        self.get_logger().info('RX worker thread started')
        
        while self.running.is_set():
            if not self.bus_connected or self.can_bus is None:
                time.sleep(self.get_parameter('reconnect_delay').value)
                self.initialize_can_bus()
                continue
            
            try:
                # Blocking read with timeout
                message = self.can_bus.recv(timeout=1.0)
                
                if message is None:
                    continue
                
                # Update statistics
                with self.stats_lock:
                    self.stats['frames_rx'] += 1
                
                # Process received frame
                self._process_rx_frame(message)
                
            except can.CanError as e:
                with self.stats_lock:
                    self.stats['errors_rx'] += 1
                self.get_logger().error(
                    f'CAN receive error: {e}', throttle_duration_sec=1.0)
            except Exception as e:
                self.get_logger().error(
                    f'Unexpected error in RX worker: {e}', 
                    throttle_duration_sec=1.0)
        
        self.get_logger().info('RX worker thread stopped')
    
    def _tx_worker(self):
        """Worker thread for transmitting CAN frames."""
        self.get_logger().info('TX worker thread started')
        
        while self.running.is_set():
            if not self.bus_connected or self.can_bus is None:
                time.sleep(0.1)
                continue
            
            try:
                # Get frame from queue with timeout
                message = self.tx_queue.get(timeout=0.1)
                
                # Send frame
                self.can_bus.send(message)
                
                # Update statistics
                with self.stats_lock:
                    self.stats['frames_tx'] += 1
                
            except queue.Empty:
                continue
            except can.CanError as e:
                with self.stats_lock:
                    self.stats['errors_tx'] += 1
                self.get_logger().error(
                    f'CAN transmit error: {e}', throttle_duration_sec=1.0)
            except Exception as e:
                self.get_logger().error(
                    f'Unexpected error in TX worker: {e}',
                    throttle_duration_sec=1.0)
        
        self.get_logger().info('TX worker thread stopped')
    
    def _process_rx_frame(self, message: can.Message):
        """Process received CAN frame."""
        # Check if this is an SDO response
        if message.arbitration_id == self.tsdo_id:
            self._handle_sdo_response(message)
        
        # Check if this is a heartbeat
        elif message.arbitration_id == self.heartbeat_id:
            self._handle_heartbeat(message)
        
        # Publish all frames if enabled
        if self.get_parameter('publish_all_frames').value:
            self._publish_can_frame(message)
    
    def _handle_sdo_response(self, message: can.Message):
        """Handle SDO response from device."""
        if len(message.data) < 4:
            return
        
        cmd = message.data[0]
        index = struct.unpack('<H', message.data[1:3])[0]
        subindex = message.data[3]
        
        # Create response key
        response_key = (index, subindex)
        
        # Store response
        with self.sdo_lock:
            if cmd == CANopenConstants.SDO_ABORT:
                abort_code = struct.unpack('<I', message.data[4:8])[0]
                self.sdo_responses[response_key] = {
                    'success': False,
                    'error_code': abort_code,
                    'timestamp': time.time()
                }
                self.get_logger().warning(
                    f'SDO abort: index=0x{index:04X}, '
                    f'subindex=0x{subindex:02X}, code=0x{abort_code:08X}')
            else:
                # Extract data if present
                data = message.data[4:8] if len(message.data) >= 8 else []
                self.sdo_responses[response_key] = {
                    'success': True,
                    'data': data,
                    'timestamp': time.time()
                }
    
    def _handle_heartbeat(self, message: can.Message):
        """Handle heartbeat from device."""
        if len(message.data) >= 1:
            state = message.data[0]
            self.get_logger().debug(
                f'Heartbeat received: state=0x{state:02X}',
                throttle_duration_sec=5.0)
    
    def _publish_can_frame(self, message: can.Message):
        """Publish CAN frame to ROS topic."""
        # This would publish to custom CANFrame message in production
        frame_str = (f'ID: 0x{message.arbitration_id:03X}, '
                    f'Data: {message.data.hex()}')
        msg = String()
        msg.data = frame_str
        # Publishing commented out to avoid spam during development
        # self.can_rx_pub.publish(msg)
    
    def send_sdo_write(self, index: int, subindex: int, data: bytes, 
                       timeout: Optional[float] = None) -> Dict[str, Any]:
        """
        Send SDO write request and wait for response.
        
        Args:
            index: Object dictionary index
            subindex: Object dictionary subindex
            data: Data to write (1, 2, or 4 bytes)
            timeout: Response timeout (uses default if None)
        
        Returns:
            Dictionary with 'success', 'data', and 'error_code' keys
        """
        if timeout is None:
            timeout = self.sdo_timeout
        
        # Determine command specifier based on data length
        if len(data) == 1:
            cmd = CANopenConstants.SDO_WRITE_1BYTE
        elif len(data) == 2:
            cmd = CANopenConstants.SDO_WRITE_2BYTE
        elif len(data) == 4:
            cmd = CANopenConstants.SDO_WRITE_4BYTE
        else:
            return {'success': False, 'error_code': 0, 
                   'message': 'Invalid data length'}
        
        # Build SDO message
        sdo_data = bytearray(8)
        sdo_data[0] = cmd
        sdo_data[1:3] = struct.pack('<H', index)
        sdo_data[3] = subindex
        sdo_data[4:4+len(data)] = data
        
        # Clear any old response
        response_key = (index, subindex)
        with self.sdo_lock:
            self.sdo_responses.pop(response_key, None)
        
        # Send SDO request
        message = can.Message(
            arbitration_id=self.rsdo_id,
            data=sdo_data,
            is_extended_id=False
        )
        
        start_time = time.time()
        
        try:
            self.tx_queue.put(message, timeout=1.0)
        except queue.Full:
            return {'success': False, 'error_code': 0, 
                   'message': 'TX queue full'}
        
        # Wait for response
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.sdo_lock:
                if response_key in self.sdo_responses:
                    response = self.sdo_responses.pop(response_key)
                    
                    # Record latency
                    latency = time.time() - start_time
                    with self.stats_lock:
                        self.stats['latency_samples'].append(latency)
                        if response['success']:
                            self.stats['sdo_success'] += 1
                        else:
                            self.stats['sdo_failures'] += 1
                    
                    return response
            
            time.sleep(0.001)  # 1ms polling interval
        
        # Timeout
        with self.stats_lock:
            self.stats['sdo_failures'] += 1
        
        return {'success': False, 'error_code': 0, 'message': 'Timeout'}
    
    def send_relay_command(self, cmd: int) -> Dict[str, Any]:
        """
        Send relay control command.
        
        Args:
            cmd: Command code (0-8, 99, 254)
        
        Returns:
            Dictionary with operation result
        """
        if cmd not in CANopenConstants.COMMANDS:
            return {
                'success': False,
                'message': f'Invalid command: {cmd}',
                'relay_state': self.current_relay_state
            }
        
        cmd_name = CANopenConstants.COMMANDS[cmd]
        self.get_logger().info(f'Sending relay command: {cmd_name} ({cmd})')
        
        # Send command
        start_time = time.time()
        result = self.send_sdo_write(
            CANopenConstants.OD_CONTROL_COMMAND, 0, bytes([cmd]))
        
        if not result['success']:
            return {
                'success': False,
                'message': f'Failed to send command: {result.get("message", "Unknown error")}',
                'relay_state': self.current_relay_state,
                'error_code': result.get('error_code', 0)
            }
        
        # Read back relay state
        time.sleep(0.05)  # Small delay for relay to actuate
        feedback_result = self.send_sdo_write(
            CANopenConstants.OD_RELAY_FEEDBACK, 0, bytes([0]))
        
        if feedback_result['success'] and 'data' in feedback_result:
            relay_state = feedback_result['data'][0] if feedback_result['data'] else 0
            with self.relay_state_lock:
                self.current_relay_state = relay_state
            
            # Publish relay state
            self._publish_relay_state(relay_state)
        
        execution_time = time.time() - start_time
        
        return {
            'success': True,
            'message': f'Command {cmd_name} executed successfully',
            'relay_state': self.current_relay_state,
            'error_code': 0,
            'execution_time': execution_time
        }
    
    def _publish_relay_state(self, relay_state: int):
        """Publish current relay state."""
        # Decode bitmap
        relays = [
            bool(relay_state & (1 << i)) for i in range(4)
        ]
        
        state_str = f'Relays: K1={relays[0]}, K2={relays[1]}, K3={relays[2]}, K4={relays[3]}'
        msg = String()
        msg.data = state_str
        self.relay_state_pub.publish(msg)
    
    def publish_diagnostics(self):
        """Publish diagnostic information."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # CAN bus status
        bus_status = DiagnosticStatus()
        bus_status.name = f'{self.get_name()}: CAN Bus'
        bus_status.hardware_id = self.can_interface
        
        if self.bus_connected:
            bus_status.level = DiagnosticStatus.OK
            bus_status.message = 'Connected'
        else:
            bus_status.level = DiagnosticStatus.ERROR
            bus_status.message = 'Disconnected'
        
        with self.stats_lock:
            bus_status.values = [
                KeyValue(key='frames_tx', value=str(self.stats['frames_tx'])),
                KeyValue(key='frames_rx', value=str(self.stats['frames_rx'])),
                KeyValue(key='errors_tx', value=str(self.stats['errors_tx'])),
                KeyValue(key='errors_rx', value=str(self.stats['errors_rx'])),
                KeyValue(key='sdo_success', value=str(self.stats['sdo_success'])),
                KeyValue(key='sdo_failures', value=str(self.stats['sdo_failures'])),
            ]
            
            # Calculate average latency
            if self.stats['latency_samples']:
                avg_latency = sum(self.stats['latency_samples']) / len(self.stats['latency_samples'])
                bus_status.values.append(
                    KeyValue(key='avg_latency_ms', value=f'{avg_latency*1000:.2f}'))
        
        diag_array.status.append(bus_status)
        self.diagnostics_pub.publish(diag_array)
    
    def shutdown(self):
        """Clean shutdown of node."""
        self.get_logger().info('Shutting down SocketCAN bridge...')
        
        # Stop threads
        self.running.clear()
        
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=2.0)
        
        if self.tx_thread and self.tx_thread.is_alive():
            self.tx_thread.join(timeout=2.0)
        
        # Close CAN bus
        if self.can_bus:
            try:
                self.can_bus.shutdown()
            except Exception as e:
                self.get_logger().error(f'Error closing CAN bus: {e}')
        
        self.get_logger().info('Shutdown complete')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = SocketCANBridgeNode()
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

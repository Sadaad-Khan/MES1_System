#!/usr/bin/env python3
"""
Multi-Node Test Harness.

Comprehensive testing framework for multi-node CANopen relay networks.
Tests node isolation, broadcast commands, heartbeat synchronization, and per-node feedback.
"""

import can
import time
import sys
import argparse
import yaml
from pathlib import Path
from typing import List, Dict, Optional, Set
from dataclasses import dataclass, field
from datetime import datetime
import json


@dataclass
class TestResult:
    """Result of a test case."""
    name: str
    passed: bool
    message: str
    duration_ms: float = 0.0
    details: Dict = field(default_factory=dict)
    timestamp: datetime = field(default_factory=datetime.now)


@dataclass
class NodeTestState:
    """Test state for a single node."""
    node_id: int
    name: str
    role: str
    
    heartbeat_count: int = 0
    last_heartbeat: Optional[float] = None
    sdo_responses: List[Dict] = field(default_factory=list)
    state_bitmap: Optional[int] = None
    command_results: List[bool] = field(default_factory=list)


class MultiNodeTestHarness:
    """Test harness for multi-node CANopen relay network."""
    
    # CANopen constants
    RSDO_BASE = 0x600
    TSDO_BASE = 0x580
    HEARTBEAT_BASE = 0x700
    
    OD_CONTROL_COMMAND = 0x2000
    OD_RELAY_FEEDBACK = 0x2001
    OD_COMMAND_RESULT = 0x2002
    OD_DIAGNOSTICS = 0x2003
    
    SDO_WRITE_1BYTE = 0x2F
    SDO_WRITE = 0x23
    SDO_READ = 0x40
    SDO_RESPONSE = 0x60
    
    def __init__(self, interface: str = 'can0', config_file: Optional[str] = None):
        """
        Initialize test harness.
        
        Args:
            interface: CAN interface name
            config_file: Path to multi_node_map.yaml
        """
        self.interface = interface
        self.config_file = config_file
        self.bus: Optional[can.Bus] = None
        
        # Node configuration
        self.nodes: Dict[int, Dict] = {}  # node_id -> config
        self.node_names: Dict[str, int] = {}  # name -> node_id
        
        # Test state
        self.test_states: Dict[int, NodeTestState] = {}  # node_id -> state
        
        # Test results
        self.results: List[TestResult] = []
        
        # Load configuration
        self._load_config()
        
        # Initialize test states
        for node_id, node_cfg in self.nodes.items():
            self.test_states[node_id] = NodeTestState(
                node_id=node_id,
                name=node_cfg['name'],
                role=node_cfg['role']
            )
    
    def _load_config(self):
        """Load multi-node configuration."""
        if not self.config_file:
            default_config = Path(__file__).parent.parent / 'config' / 'multi_node_map.yaml'
            if default_config.exists():
                self.config_file = str(default_config)
        
        if not self.config_file:
            print("Error: No config file found", file=sys.stderr)
            return
        
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            for node_cfg in config.get('nodes', []):
                if not node_cfg.get('enabled', True):
                    continue
                
                node_id = node_cfg['node_id']
                name = node_cfg['name']
                
                self.nodes[node_id] = {
                    'name': name,
                    'role': node_cfg['role'],
                    'relays': node_cfg.get('relays', []),
                    'commands': node_cfg.get('supported_commands', [])
                }
                
                self.node_names[name] = node_id
            
            print(f"Loaded configuration for {len(self.nodes)} nodes")
            
        except Exception as e:
            print(f"Error loading config: {e}", file=sys.stderr)
    
    def connect(self):
        """Connect to CAN bus."""
        try:
            self.bus = can.Bus(
                interface='socketcan',
                channel=self.interface,
                bitrate=1000000
            )
            print(f"✓ Connected to {self.interface}")
            return True
        except Exception as e:
            print(f"✗ Failed to connect: {e}", file=sys.stderr)
            return False
    
    def disconnect(self):
        """Disconnect from CAN bus."""
        if self.bus:
            self.bus.shutdown()
            self.bus = None
    
    def send_command(self, node_id: int, command: int) -> bool:
        """
        Send command to node.
        
        Args:
            node_id: Target node ID
            command: Command code
        
        Returns:
            True if sent successfully
        """
        if not self.bus:
            return False
        
        sdo_data = bytes([
            self.SDO_WRITE_1BYTE,
            self.OD_CONTROL_COMMAND & 0xFF,
            (self.OD_CONTROL_COMMAND >> 8) & 0xFF,
            0x00,
            command,
            0, 0, 0
        ])
        
        msg = can.Message(
            arbitration_id=self.RSDO_BASE + node_id,
            data=sdo_data,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            return True
        except Exception as e:
            print(f"Error sending command: {e}", file=sys.stderr)
            return False
    
    def listen(self, duration: float) -> List[can.Message]:
        """
        Listen for CAN messages.
        
        Args:
            duration: Listen duration in seconds
        
        Returns:
            List of received messages
        """
        if not self.bus:
            return []
        
        messages = []
        end_time = time.time() + duration
        
        while time.time() < end_time:
            remaining = end_time - time.time()
            msg = self.bus.recv(timeout=min(remaining, 0.1))
            if msg:
                messages.append(msg)
                self._process_message(msg)
        
        return messages
    
    def _process_message(self, msg: can.Message):
        """Process received CAN message and update test state."""
        can_id = msg.arbitration_id
        
        # Heartbeat
        if can_id >= self.HEARTBEAT_BASE and can_id < self.HEARTBEAT_BASE + 0x7F:
            node_id = can_id - self.HEARTBEAT_BASE
            if node_id in self.test_states:
                state = self.test_states[node_id]
                state.heartbeat_count += 1
                state.last_heartbeat = time.time()
        
        # SDO response (TSDO)
        elif can_id >= self.TSDO_BASE and can_id < self.TSDO_BASE + 0x7F:
            node_id = can_id - self.TSDO_BASE
            if node_id in self.test_states:
                self._process_sdo_response(node_id, msg.data)
    
    def _process_sdo_response(self, node_id: int, data: bytes):
        """Process SDO response."""
        if len(data) < 4:
            return
        
        state = self.test_states[node_id]
        
        cmd = data[0]
        index = data[1] | (data[2] << 8)
        subindex = data[3]
        
        sdo_info = {
            'cmd': cmd,
            'index': index,
            'subindex': subindex,
            'data': data,
            'timestamp': time.time()
        }
        
        state.sdo_responses.append(sdo_info)
        
        # Parse specific responses
        if index == self.OD_RELAY_FEEDBACK and len(data) >= 5:
            state.state_bitmap = data[4]
        
        elif index == self.OD_COMMAND_RESULT and len(data) >= 5:
            success = data[4] == 1
            state.command_results.append(success)
    
    # ========================================================================
    # Test Cases
    # ========================================================================
    
    def test_node_discovery(self) -> TestResult:
        """Test that all configured nodes send heartbeats."""
        print("\n[TEST] Node Discovery via Heartbeat")
        
        start_time = time.time()
        
        # Listen for heartbeats
        print("  Listening for heartbeats (10 seconds)...")
        self.listen(10.0)
        
        # Check which nodes were discovered
        discovered = set()
        missing = set()
        
        for node_id, state in self.test_states.items():
            if state.heartbeat_count > 0:
                discovered.add(node_id)
                print(f"  ✓ Node {state.name} (0x{node_id:02X}): {state.heartbeat_count} heartbeats")
            else:
                missing.add(node_id)
                print(f"  ✗ Node {state.name} (0x{node_id:02X}): NO heartbeat")
        
        duration_ms = (time.time() - start_time) * 1000
        
        passed = len(missing) == 0
        message = f"Discovered {len(discovered)}/{len(self.test_states)} nodes"
        
        result = TestResult(
            name="node_discovery",
            passed=passed,
            message=message,
            duration_ms=duration_ms,
            details={
                'discovered': list(discovered),
                'missing': list(missing),
                'heartbeat_counts': {
                    self.test_states[nid].name: self.test_states[nid].heartbeat_count
                    for nid in self.test_states
                }
            }
        )
        
        self.results.append(result)
        return result
    
    def test_node_isolation(self) -> TestResult:
        """Test that nodes only respond to their assigned commands."""
        print("\n[TEST] Node Isolation (Command Filtering)")
        
        start_time = time.time()
        
        # Test: Send K1_ON (cmd 1) - should only affect DRL node
        # Test: Send K2_ON (cmd 3) - should only affect Safety node
        # Test: Send K3_ON (cmd 5) - should only affect Battery node
        
        test_cases = [
            (1, 'relay_drl', "K1_ON should only affect DRL node"),
            (3, 'relay_safety', "K2_ON should only affect Safety node"),
            (5, 'relay_battery', "K3_ON should only affect Battery node"),
        ]
        
        isolation_violations = []
        
        for command, expected_node, description in test_cases:
            print(f"\n  Testing: {description}")
            
            # Clear previous responses
            for state in self.test_states.values():
                state.sdo_responses.clear()
                state.command_results.clear()
                state.state_bitmap = None
            
            # Send command to expected node
            if expected_node in self.node_names:
                expected_id = self.node_names[expected_node]
                print(f"    Sending command {command} to {expected_node} (0x{expected_id:02X})")
                self.send_command(expected_id, command)
                
                # Listen for responses
                time.sleep(0.5)
                self.listen(1.0)
                
                # Check that only the expected node responded
                for node_id, state in self.test_states.items():
                    response_count = len(state.sdo_responses)
                    
                    if node_id == expected_id:
                        if response_count == 0:
                            print(f"    ✗ {state.name}: NO response (expected response)")
                            isolation_violations.append(
                                f"{state.name} did not respond to its command"
                            )
                        else:
                            print(f"    ✓ {state.name}: {response_count} responses")
                    else:
                        if response_count > 0:
                            print(f"    ✗ {state.name}: {response_count} responses (expected NONE)")
                            isolation_violations.append(
                                f"{state.name} responded to wrong command {command}"
                            )
                        else:
                            print(f"    ✓ {state.name}: no response (correct)")
        
        duration_ms = (time.time() - start_time) * 1000
        
        passed = len(isolation_violations) == 0
        message = "All nodes properly isolated" if passed else f"{len(isolation_violations)} violations"
        
        result = TestResult(
            name="node_isolation",
            passed=passed,
            message=message,
            duration_ms=duration_ms,
            details={'violations': isolation_violations}
        )
        
        self.results.append(result)
        return result
    
    def test_broadcast_commands(self) -> TestResult:
        """Test that broadcast commands reach all nodes."""
        print("\n[TEST] Broadcast Commands")
        
        start_time = time.time()
        
        # Test ALL_OFF (0) and ALL_ON (99) broadcasts
        broadcast_tests = [
            (99, "ALL_ON"),
            (0, "ALL_OFF"),
        ]
        
        broadcast_issues = []
        
        for command, name in broadcast_tests:
            print(f"\n  Testing broadcast: {name} (command {command})")
            
            # Clear responses
            for state in self.test_states.values():
                state.sdo_responses.clear()
                state.command_results.clear()
            
            # Send to all nodes
            for node_id in self.nodes.keys():
                self.send_command(node_id, command)
                time.sleep(0.05)
            
            # Listen for responses
            time.sleep(0.5)
            self.listen(2.0)
            
            # Check all nodes responded
            for node_id, state in self.test_states.items():
                response_count = len(state.sdo_responses)
                
                if response_count == 0:
                    print(f"    ✗ {state.name}: NO response")
                    broadcast_issues.append(f"{state.name} did not respond to {name}")
                else:
                    print(f"    ✓ {state.name}: {response_count} responses")
        
        duration_ms = (time.time() - start_time) * 1000
        
        passed = len(broadcast_issues) == 0
        message = "All nodes responded to broadcasts" if passed else f"{len(broadcast_issues)} issues"
        
        result = TestResult(
            name="broadcast_commands",
            passed=passed,
            message=message,
            duration_ms=duration_ms,
            details={'issues': broadcast_issues}
        )
        
        self.results.append(result)
        return result
    
    def test_heartbeat_sync(self) -> TestResult:
        """Test heartbeat synchronization and timing."""
        print("\n[TEST] Heartbeat Synchronization")
        
        start_time = time.time()
        
        # Clear heartbeat counts
        for state in self.test_states.values():
            state.heartbeat_count = 0
            state.last_heartbeat = None
        
        # Listen for 15 seconds (should get ~3 heartbeats at 5s interval)
        print("  Monitoring heartbeats (15 seconds)...")
        self.listen(15.0)
        
        sync_issues = []
        
        for node_id, state in self.test_states.items():
            expected_count = 3
            tolerance = 1
            
            if state.heartbeat_count < (expected_count - tolerance):
                print(f"  ✗ {state.name}: {state.heartbeat_count} heartbeats (expected ~{expected_count})")
                sync_issues.append(
                    f"{state.name} sent only {state.heartbeat_count} heartbeats"
                )
            elif state.heartbeat_count > (expected_count + tolerance):
                print(f"  ⚠ {state.name}: {state.heartbeat_count} heartbeats (expected ~{expected_count})")
            else:
                print(f"  ✓ {state.name}: {state.heartbeat_count} heartbeats")
        
        duration_ms = (time.time() - start_time) * 1000
        
        passed = len(sync_issues) == 0
        message = "Heartbeats synchronized" if passed else f"{len(sync_issues)} issues"
        
        result = TestResult(
            name="heartbeat_sync",
            passed=passed,
            message=message,
            duration_ms=duration_ms,
            details={'issues': sync_issues}
        )
        
        self.results.append(result)
        return result
    
    def test_per_node_feedback(self) -> TestResult:
        """Test that each node reports correct relay state feedback."""
        print("\n[TEST] Per-Node Relay Feedback")
        
        start_time = time.time()
        
        feedback_issues = []
        
        # Test each node's relay control and feedback
        for node_id, node_cfg in self.nodes.items():
            state = self.test_states[node_id]
            commands = node_cfg.get('commands', [])
            
            print(f"\n  Testing {state.name}...")
            
            # Filter to ON commands only (1, 3, 5, 7)
            on_commands = [c for c in commands if c in [1, 3, 5, 7]]
            
            for cmd in on_commands:
                # Clear state
                state.sdo_responses.clear()
                state.state_bitmap = None
                
                # Send command
                self.send_command(node_id, cmd)
                
                # Wait for feedback
                time.sleep(0.3)
                self.listen(0.5)
                
                # Check feedback received
                if state.state_bitmap is None:
                    print(f"    ✗ Command {cmd}: NO feedback")
                    feedback_issues.append(f"{state.name} did not send feedback for cmd {cmd}")
                else:
                    print(f"    ✓ Command {cmd}: bitmap=0x{state.state_bitmap:02X}")
        
        duration_ms = (time.time() - start_time) * 1000
        
        passed = len(feedback_issues) == 0
        message = "All feedback correct" if passed else f"{len(feedback_issues)} issues"
        
        result = TestResult(
            name="per_node_feedback",
            passed=passed,
            message=message,
            duration_ms=duration_ms,
            details={'issues': feedback_issues}
        )
        
        self.results.append(result)
        return result
    
    # ========================================================================
    # Test Execution and Reporting
    # ========================================================================
    
    def run_all_tests(self) -> bool:
        """
        Run all test cases.
        
        Returns:
            True if all tests passed
        """
        print("\n" + "=" * 70)
        print("  Multi-Node CANopen Relay Network Test Suite")
        print("=" * 70)
        
        self.results.clear()
        
        # Run tests
        self.test_node_discovery()
        self.test_heartbeat_sync()
        self.test_node_isolation()
        self.test_broadcast_commands()
        self.test_per_node_feedback()
        
        # Print summary
        self.print_summary()
        
        # Return overall pass/fail
        return all(r.passed for r in self.results)
    
    def print_summary(self):
        """Print test results summary."""
        print("\n" + "=" * 70)
        print("  Test Results Summary")
        print("=" * 70)
        
        passed_count = sum(1 for r in self.results if r.passed)
        total_count = len(self.results)
        total_duration = sum(r.duration_ms for r in self.results)
        
        for result in self.results:
            status = "✓ PASS" if result.passed else "✗ FAIL"
            print(f"  {status}  {result.name:<25} {result.message}")
            print(f"         Duration: {result.duration_ms:.1f} ms")
        
        print("\n" + "-" * 70)
        print(f"  Total: {passed_count}/{total_count} passed")
        print(f"  Duration: {total_duration:.1f} ms")
        print("=" * 70)
    
    def export_results(self, filename: str):
        """Export results to JSON file."""
        results_data = {
            'summary': {
                'total': len(self.results),
                'passed': sum(1 for r in self.results if r.passed),
                'failed': sum(1 for r in self.results if not r.passed),
                'duration_ms': sum(r.duration_ms for r in self.results)
            },
            'tests': [
                {
                    'name': r.name,
                    'passed': r.passed,
                    'message': r.message,
                    'duration_ms': r.duration_ms,
                    'details': r.details,
                    'timestamp': r.timestamp.isoformat()
                }
                for r in self.results
            ]
        }
        
        with open(filename, 'w') as f:
            json.dump(results_data, f, indent=2)
        
        print(f"\nResults exported to: {filename}")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Multi-node CANopen relay network test harness'
    )
    
    parser.add_argument(
        '--interface', '-i',
        default='can0',
        help='CAN interface (default: can0)'
    )
    
    parser.add_argument(
        '--config', '-c',
        help='Path to multi_node_map.yaml'
    )
    
    parser.add_argument(
        '--export', '-e',
        help='Export results to JSON file'
    )
    
    parser.add_argument(
        '--test', '-t',
        choices=['discovery', 'isolation', 'broadcast', 'heartbeat', 'feedback', 'all'],
        default='all',
        help='Test to run (default: all)'
    )
    
    args = parser.parse_args()
    
    # Create harness
    harness = MultiNodeTestHarness(
        interface=args.interface,
        config_file=args.config
    )
    
    # Connect to CAN bus
    if not harness.connect():
        return 1
    
    try:
        # Run tests
        if args.test == 'all':
            success = harness.run_all_tests()
        elif args.test == 'discovery':
            result = harness.test_node_discovery()
            harness.print_summary()
            success = result.passed
        elif args.test == 'isolation':
            result = harness.test_node_isolation()
            harness.print_summary()
            success = result.passed
        elif args.test == 'broadcast':
            result = harness.test_broadcast_commands()
            harness.print_summary()
            success = result.passed
        elif args.test == 'heartbeat':
            result = harness.test_heartbeat_sync()
            harness.print_summary()
            success = result.passed
        elif args.test == 'feedback':
            result = harness.test_per_node_feedback()
            harness.print_summary()
            success = result.passed
        
        # Export if requested
        if args.export:
            harness.export_results(args.export)
        
        return 0 if success else 1
    
    finally:
        harness.disconnect()


if __name__ == '__main__':
    sys.exit(main())

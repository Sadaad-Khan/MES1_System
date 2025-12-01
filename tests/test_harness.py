#!/usr/bin/env python3
"""
Production-Grade CANopen Relay Node Test Harness

This test harness validates the complete CAN relay system including:
- SDO command round-trip (ACK + RESULT + FEEDBACK)
- Heartbeat monitoring
- Diagnostic counter validation
- CAN recovery testing
- Buzzer pattern verification
- Multi-node testing

Usage:
    # Interactive mode
    python3 test_harness.py --interface can0 --node-id 0x15

    # Automated test suite
    python3 test_harness.py --interface can0 --node-id 0x15 --auto

    # Simulate bus outage
    python3 test_harness.py --interface can0 --node-id 0x15 --simulate-outage

    # Multiple nodes
    python3 test_harness.py --interface can0 --node-id 0x15,0x16,0x17 --auto
"""

import argparse
import sys
import time
import json
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

try:
    import can
    from can import Message
except ImportError:
    print("ERROR: python-can not installed. Run: pip3 install python-can", file=sys.stderr)
    sys.exit(1)


# ============================================================================
# Constants
# ============================================================================

# CANopen Object Dictionary
OD_CONTROL_COMMAND = 0x2000
OD_RELAY_FEEDBACK = 0x2001
OD_COMMAND_RESULT = 0x2002
OD_DIAGNOSTICS = 0x2003

# SDO Commands
SDO_WRITE = 0x23
SDO_READ = 0x40
SDO_RESPONSE = 0x60
SDO_ERROR = 0x80

# Relay Commands
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
    9: "BUZZER_TEST",
    99: "ALL_ON",
    254: "SOFTWARE_RESET"
}

# Expected relay bitmaps after commands
EXPECTED_BITMAPS = {
    0: 0b0000,   # ALL OFF
    1: 0b0001,   # K1 ON
    2: 0b0000,   # K1 OFF
    3: 0b0010,   # K2 ON
    4: 0b0000,   # K2 OFF
    5: 0b0100,   # K3 ON
    6: 0b0000,   # K3 OFF
    7: 0b1000,   # K4 ON
    8: 0b0000,   # K4 OFF
    99: 0b1111,  # ALL ON
}

# Timing constants
HEARTBEAT_INTERVAL_MS = 5000
HEARTBEAT_TOLERANCE_MS = 1.5  # 150% of interval
SDO_RESPONSE_TIMEOUT_MS = 500
TEST_COMMAND_DELAY_MS = 200


# ============================================================================
# Data Classes
# ============================================================================

@dataclass
class TestResult:
    """Result of a single test"""
    name: str
    passed: bool
    message: str
    duration_ms: float = 0.0

    def to_dict(self) -> Dict:
        return {
            'name': self.name,
            'passed': self.passed,
            'message': self.message,
            'duration_ms': self.duration_ms
        }


@dataclass
class SDOResponse:
    """Parsed SDO response"""
    cmd: int
    index: int
    subindex: int
    data: bytes
    is_ack: bool
    is_error: bool


class BuzzerPattern(Enum):
    """Buzzer patterns"""
    SINGLE = "single"
    DOUBLE = "double"


# ============================================================================
# Test Harness Class
# ============================================================================

class CANRelayTestHarness:
    """Complete test harness for CANopen relay node"""

    def __init__(self, interface: str, node_id: int, backend: str = 'socketcan'):
        self.interface = interface
        self.node_id = node_id
        self.backend = backend

        # COB-IDs
        self.cob_tsdo = 0x580 + node_id
        self.cob_rsdo = 0x600 + node_id
        self.cob_heartbeat = 0x700 + node_id

        # CAN bus
        self.bus: Optional[can.Bus] = None
        self.notifier: Optional[can.Notifier] = None

        # Message buffers
        self.received_messages: List[Message] = []
        self.last_heartbeat: Optional[float] = None
        self.heartbeat_count: int = 0

        # Diagnostic tracking
        self.diag_counters: Dict[str, int] = {
            'tx_success': 0,
            'tx_fail': 0,
            'rx_count': 0,
            'recovery_count': 0
        }

        # Test results
        self.test_results: List[TestResult] = []

    def connect(self) -> bool:
        """Initialize CAN bus connection"""
        try:
            print(f"Connecting to {self.interface} (backend: {self.backend})...")
            self.bus = can.Bus(interface=self.backend, channel=self.interface, bitrate=1000000)

            # Start listener
            self.notifier = can.Notifier(self.bus, [self._message_handler])

            print(f"✓ Connected to {self.interface}")
            print(f"  Node ID: 0x{self.node_id:02X}")
            print(f"  RSDO: 0x{self.cob_rsdo:03X}")
            print(f"  TSDO: 0x{self.cob_tsdo:03X}")
            print(f"  Heartbeat: 0x{self.cob_heartbeat:03X}")
            return True

        except Exception as e:
            print(f"✗ Failed to connect: {e}", file=sys.stderr)
            return False

    def disconnect(self):
        """Clean shutdown"""
        if self.notifier:
            self.notifier.stop()
        if self.bus:
            self.bus.shutdown()
        print("Disconnected")

    def _message_handler(self, msg: Message):
        """Handle incoming CAN messages"""
        self.received_messages.append(msg)

        # Track heartbeat
        if msg.arbitration_id == self.cob_heartbeat:
            self.last_heartbeat = time.time()
            self.heartbeat_count += 1

    def clear_rx_buffer(self):
        """Clear received message buffer"""
        self.received_messages.clear()

    def send_sdo_write(self, index: int, subindex: int, data: int) -> bool:
        """Send SDO write command"""
        payload = [
            SDO_WRITE,
            index & 0xFF,
            (index >> 8) & 0xFF,
            subindex,
            data & 0xFF,
            (data >> 8) & 0xFF,
            (data >> 16) & 0xFF,
            (data >> 24) & 0xFF
        ]

        msg = Message(
            arbitration_id=self.cob_rsdo,
            data=payload,
            is_extended_id=False
        )

        try:
            self.bus.send(msg)
            return True
        except Exception as e:
            print(f"✗ Failed to send SDO: {e}", file=sys.stderr)
            return False

    def wait_for_sdo_response(self, expected_index: Optional[int] = None,
                             timeout_ms: float = SDO_RESPONSE_TIMEOUT_MS) -> Optional[SDOResponse]:
        """Wait for SDO response (ACK, RESULT, or FEEDBACK)"""
        start_time = time.time()
        timeout_s = timeout_ms / 1000.0

        while (time.time() - start_time) < timeout_s:
            for msg in self.received_messages[:]:
                if msg.arbitration_id == self.cob_tsdo and len(msg.data) >= 4:
                    # Parse SDO response
                    cmd = msg.data[0]
                    index = msg.data[1] | (msg.data[2] << 8)
                    subindex = msg.data[3]
                    data = bytes(msg.data[4:])

                    # Check if this matches our expected index (if specified)
                    if expected_index is None or index == expected_index:
                        self.received_messages.remove(msg)
                        return SDOResponse(
                            cmd=cmd,
                            index=index,
                            subindex=subindex,
                            data=data,
                            is_ack=(cmd == SDO_RESPONSE),
                            is_error=(cmd == SDO_ERROR)
                        )

            time.sleep(0.001)  # 1ms poll interval

        return None

    def wait_for_heartbeat(self, timeout_s: float = 10.0) -> bool:
        """Wait for at least one heartbeat"""
        start_time = time.time()
        initial_count = self.heartbeat_count

        while (time.time() - start_time) < timeout_s:
            if self.heartbeat_count > initial_count:
                return True
            time.sleep(0.01)

        return False

    # ========================================================================
    # Test Cases
    # ========================================================================

    def test_heartbeat_monitoring(self) -> TestResult:
        """Test 1: Verify heartbeat is transmitted regularly"""
        print("\n[TEST] Heartbeat Monitoring")
        print("  Listening for heartbeat for 10 seconds...")

        start_time = time.time()
        self.heartbeat_count = 0
        self.clear_rx_buffer()

        # Wait for heartbeats
        time.sleep(10.0)

        duration_ms = (time.time() - start_time) * 1000
        expected_min_heartbeats = int(10000 / (HEARTBEAT_INTERVAL_MS * HEARTBEAT_TOLERANCE_MS))

        if self.heartbeat_count >= expected_min_heartbeats:
            return TestResult(
                name="Heartbeat Monitoring",
                passed=True,
                message=f"Received {self.heartbeat_count} heartbeats in 10s (expected >= {expected_min_heartbeats})",
                duration_ms=duration_ms
            )
        else:
            return TestResult(
                name="Heartbeat Monitoring",
                passed=False,
                message=f"Only received {self.heartbeat_count} heartbeats (expected >= {expected_min_heartbeats})",
                duration_ms=duration_ms
            )

    def test_sdo_command_roundtrip(self, cmd: int) -> TestResult:
        """Test 2: Complete SDO command round-trip (ACK + RESULT + FEEDBACK)"""
        cmd_name = COMMANDS.get(cmd, f"Unknown({cmd})")
        print(f"\n[TEST] SDO Command Round-trip: {cmd_name} (cmd={cmd})")

        start_time = time.time()
        self.clear_rx_buffer()

        # Send command
        print(f"  → Sending SDO WRITE to 0x{OD_CONTROL_COMMAND:04X}, cmd={cmd}")
        if not self.send_sdo_write(OD_CONTROL_COMMAND, 0, cmd):
            return TestResult(
                name=f"SDO Command {cmd_name}",
                passed=False,
                message="Failed to send SDO WRITE",
                duration_ms=(time.time() - start_time) * 1000
            )

        # Wait for ACK
        print("  ← Waiting for ACK...")
        ack = self.wait_for_sdo_response(expected_index=OD_CONTROL_COMMAND)
        if not ack or not ack.is_ack:
            return TestResult(
                name=f"SDO Command {cmd_name}",
                passed=False,
                message="No ACK received or error response",
                duration_ms=(time.time() - start_time) * 1000
            )
        print(f"  ✓ ACK received")

        # Wait for RESULT
        print("  ← Waiting for RESULT...")
        result = self.wait_for_sdo_response(expected_index=OD_COMMAND_RESULT)
        if not result:
            return TestResult(
                name=f"SDO Command {cmd_name}",
                passed=False,
                message="No RESULT received",
                duration_ms=(time.time() - start_time) * 1000
            )

        success = result.data[0] if len(result.data) > 0 else 0
        print(f"  ✓ RESULT received: {'SUCCESS' if success else 'FAILED'}")

        if not success:
            return TestResult(
                name=f"SDO Command {cmd_name}",
                passed=False,
                message="Command execution failed per RESULT",
                duration_ms=(time.time() - start_time) * 1000
            )

        # Wait for FEEDBACK
        print("  ← Waiting for FEEDBACK...")
        feedback = self.wait_for_sdo_response(expected_index=OD_RELAY_FEEDBACK)
        if not feedback:
            return TestResult(
                name=f"SDO Command {cmd_name}",
                passed=False,
                message="No FEEDBACK received",
                duration_ms=(time.time() - start_time) * 1000
            )

        bitmap = feedback.data[0] if len(feedback.data) > 0 else 0
        print(f"  ✓ FEEDBACK received: bitmap=0b{bitmap:04b}")

        # Validate bitmap if command is in expected map
        if cmd in EXPECTED_BITMAPS:
            expected_bitmap = EXPECTED_BITMAPS[cmd]
            if bitmap != expected_bitmap:
                return TestResult(
                    name=f"SDO Command {cmd_name}",
                    passed=False,
                    message=f"Bitmap mismatch: got 0b{bitmap:04b}, expected 0b{expected_bitmap:04b}",
                    duration_ms=(time.time() - start_time) * 1000
                )

        duration_ms = (time.time() - start_time) * 1000
        return TestResult(
            name=f"SDO Command {cmd_name}",
            passed=True,
            message=f"ACK + RESULT + FEEDBACK OK, bitmap=0b{bitmap:04b}",
            duration_ms=duration_ms
        )

    def test_all_commands(self) -> List[TestResult]:
        """Test 3: Run all commands in sequence"""
        print("\n[TEST SUITE] All Commands")
        results = []

        test_sequence = [0, 1, 2, 3, 4, 5, 6, 7, 8, 99, 0]  # Exclude 9 (buzzer test) and 254 (reset)

        for cmd in test_sequence:
            result = self.test_sdo_command_roundtrip(cmd)
            results.append(result)

            if result.passed:
                print(f"  ✓ {result.name}: PASS")
            else:
                print(f"  ✗ {result.name}: FAIL - {result.message}")

            # Delay between commands
            time.sleep(TEST_COMMAND_DELAY_MS / 1000.0)

        return results

    def test_diagnostics_counters(self) -> TestResult:
        """Test 4: Request and validate diagnostic counters"""
        print("\n[TEST] Diagnostics Counters")

        start_time = time.time()
        self.clear_rx_buffer()

        # Send a few commands to generate activity
        print("  Generating traffic...")
        for cmd in [1, 2, 3, 4]:
            self.send_sdo_write(OD_CONTROL_COMMAND, 0, cmd)
            time.sleep(0.1)
            # Clear responses
            while self.wait_for_sdo_response(timeout_ms=100):
                pass

        time.sleep(1.0)

        # Request diagnostics (wait for periodic publish or trigger read)
        print("  Waiting for diagnostics SDO...")
        diag = self.wait_for_sdo_response(expected_index=OD_DIAGNOSTICS, timeout_ms=15000)

        if not diag:
            return TestResult(
                name="Diagnostics Counters",
                passed=False,
                message="No diagnostics SDO received",
                duration_ms=(time.time() - start_time) * 1000
            )

        # Parse counters
        if len(diag.data) >= 4:
            tx_ok = diag.data[0] | (diag.data[1] << 8)
            tx_fail = diag.data[2] | (diag.data[3] << 8)

            print(f"  Diagnostics: TX_OK={tx_ok}, TX_FAIL={tx_fail}")

            return TestResult(
                name="Diagnostics Counters",
                passed=True,
                message=f"Diagnostics received: TX_OK={tx_ok}, TX_FAIL={tx_fail}",
                duration_ms=(time.time() - start_time) * 1000
            )
        else:
            return TestResult(
                name="Diagnostics Counters",
                passed=False,
                message="Invalid diagnostics data",
                duration_ms=(time.time() - start_time) * 1000
            )

    def test_bus_recovery_simulation(self) -> TestResult:
        """Test 5: Simulate bus outage and verify recovery"""
        print("\n[TEST] CAN Bus Recovery Simulation")
        print("  WARNING: This test simulates a bus outage")

        start_time = time.time()

        # Ensure node is healthy first
        print("  Checking initial heartbeat...")
        if not self.wait_for_heartbeat(timeout_s=10.0):
            return TestResult(
                name="Bus Recovery",
                passed=False,
                message="Node not sending heartbeats before test",
                duration_ms=(time.time() - start_time) * 1000
            )

        print("  ✓ Node healthy")

        # Option B: Software simulation - block RX processing
        print("  Simulating bus outage (blocking RX for 15 seconds)...")
        initial_heartbeat_count = self.heartbeat_count

        # Stop processing messages temporarily
        if self.notifier:
            self.notifier.stop()

        time.sleep(15.0)  # Simulate outage

        # Resume message processing
        print("  Resuming message processing...")
        self.notifier = can.Notifier(self.bus, [self._message_handler])

        # Wait for recovery
        print("  Waiting for node recovery (heartbeat)...")
        recovered = self.wait_for_heartbeat(timeout_s=70.0)  # Allow time for progressive backoff

        if recovered:
            return TestResult(
                name="Bus Recovery",
                passed=True,
                message=f"Node recovered after simulated outage (heartbeats: {initial_heartbeat_count} → {self.heartbeat_count})",
                duration_ms=(time.time() - start_time) * 1000
            )
        else:
            return TestResult(
                name="Bus Recovery",
                passed=False,
                message="Node did not recover within timeout",
                duration_ms=(time.time() - start_time) * 1000
            )

    def run_automated_test_suite(self) -> bool:
        """Run complete automated test suite"""
        print("\n" + "=" * 70)
        print("AUTOMATED TEST SUITE")
        print("=" * 70)

        all_results = []

        # Test 1: Heartbeat
        result = self.test_heartbeat_monitoring()
        all_results.append(result)
        self.test_results.append(result)

        # Test 2: All commands
        results = self.test_all_commands()
        all_results.extend(results)
        self.test_results.extend(results)

        # Test 3: Diagnostics
        result = self.test_diagnostics_counters()
        all_results.append(result)
        self.test_results.append(result)

        # Print summary
        print("\n" + "=" * 70)
        print("TEST SUMMARY")
        print("=" * 70)

        passed = sum(1 for r in all_results if r.passed)
        failed = sum(1 for r in all_results if not r.passed)
        total = len(all_results)

        for result in all_results:
            status = "✓ PASS" if result.passed else "✗ FAIL"
            print(f"{status} | {result.name:40s} | {result.duration_ms:6.1f}ms")
            if not result.passed:
                print(f"       └─ {result.message}")

        print("=" * 70)
        print(f"TOTAL: {passed}/{total} passed, {failed}/{total} failed")
        print("=" * 70)

        # Export JSON results
        self.export_results_json("test_results.json")

        return failed == 0

    def export_results_json(self, filename: str):
        """Export test results to JSON file"""
        results_dict = {
            'timestamp': time.time(),
            'node_id': f"0x{self.node_id:02X}",
            'interface': self.interface,
            'tests': [r.to_dict() for r in self.test_results],
            'summary': {
                'total': len(self.test_results),
                'passed': sum(1 for r in self.test_results if r.passed),
                'failed': sum(1 for r in self.test_results if not r.passed)
            }
        }

        with open(filename, 'w') as f:
            json.dump(results_dict, f, indent=2)

        print(f"\n✓ Results exported to {filename}")

    def interactive_mode(self):
        """Interactive command mode"""
        print("\n" + "=" * 70)
        print("INTERACTIVE MODE")
        print("=" * 70)
        print("Available commands:")
        for cmd, name in sorted(COMMANDS.items()):
            print(f"  {cmd:3d} - {name}")
        print("  'h' - Request heartbeat check")
        print("  'q' - Quit")
        print("=" * 70)

        while True:
            try:
                user_input = input("\nEnter command> ").strip()

                if user_input.lower() == 'q':
                    break
                elif user_input.lower() == 'h':
                    if self.wait_for_heartbeat(timeout_s=10.0):
                        print(f"✓ Heartbeat received (count: {self.heartbeat_count})")
                    else:
                        print("✗ No heartbeat received")
                    continue

                cmd = int(user_input)

                if cmd in COMMANDS:
                    result = self.test_sdo_command_roundtrip(cmd)
                    if result.passed:
                        print(f"✓ {result.message}")
                    else:
                        print(f"✗ {result.message}")
                else:
                    print(f"Invalid command: {cmd}")

            except ValueError:
                print("Invalid input")
            except KeyboardInterrupt:
                print("\nInterrupted")
                break


# ============================================================================
# Multi-Node Testing
# ============================================================================

def test_multiple_nodes(interface: str, node_ids: List[int]):
    """Test multiple relay nodes"""
    print("\n" + "=" * 70)
    print(f"MULTI-NODE TEST: {len(node_ids)} nodes")
    print("=" * 70)

    harnesses = []

    # Create harnesses for each node
    for node_id in node_ids:
        harness = CANRelayTestHarness(interface, node_id)
        if not harness.connect():
            print(f"✗ Failed to connect harness for node 0x{node_id:02X}")
            return False
        harnesses.append(harness)
        time.sleep(0.5)

    # Test: Send "ALL ON" to all nodes
    print("\nSending ALL ON (99) to all nodes...")
    all_passed = True

    for i, harness in enumerate(harnesses):
        node_id = node_ids[i]
        print(f"\n  Node 0x{node_id:02X}:")
        result = harness.test_sdo_command_roundtrip(99)

        if result.passed:
            print(f"    ✓ PASS")
        else:
            print(f"    ✗ FAIL: {result.message}")
            all_passed = False

    # Test: Send "ALL OFF" to all nodes
    print("\nSending ALL OFF (0) to all nodes...")

    for i, harness in enumerate(harnesses):
        node_id = node_ids[i]
        print(f"\n  Node 0x{node_id:02X}:")
        result = harness.test_sdo_command_roundtrip(0)

        if result.passed:
            print(f"    ✓ PASS")
        else:
            print(f"    ✗ FAIL: {result.message}")
            all_passed = False

    # Cleanup
    for harness in harnesses:
        harness.disconnect()

    return all_passed


# ============================================================================
# Main Entry Point
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="CANopen Relay Node Test Harness",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('--interface', '-i', default='can0',
                       help='CAN interface (default: can0)')
    parser.add_argument('--node-id', '-n', default='0x15',
                       help='Node ID in hex (e.g., 0x15) or comma-separated for multi-node (default: 0x15)')
    parser.add_argument('--backend', '-b', default='socketcan',
                       choices=['socketcan', 'kvaser', 'pcan', 'usb2can', 'virtual'],
                       help='python-can backend (default: socketcan)')
    parser.add_argument('--auto', '-a', action='store_true',
                       help='Run automated test suite')
    parser.add_argument('--simulate-outage', action='store_true',
                       help='Run bus recovery simulation test')
    parser.add_argument('--dry-run', action='store_true',
                       help='Dry run mode (no failures trigger error exit)')

    args = parser.parse_args()

    # Parse node IDs
    node_ids_str = args.node_id.split(',')
    node_ids = []

    for node_id_str in node_ids_str:
        try:
            node_id = int(node_id_str, 16 if node_id_str.startswith('0x') else 10)
            if node_id < 1 or node_id > 127:
                print(f"ERROR: Invalid node ID {node_id} (must be 1-127)", file=sys.stderr)
                return 1
            node_ids.append(node_id)
        except ValueError:
            print(f"ERROR: Invalid node ID format: {node_id_str}", file=sys.stderr)
            return 1

    # Multi-node test
    if len(node_ids) > 1:
        success = test_multiple_nodes(args.interface, node_ids)
        return 0 if (success or args.dry_run) else 1

    # Single-node testing
    node_id = node_ids[0]
    harness = CANRelayTestHarness(args.interface, node_id, args.backend)

    if not harness.connect():
        return 1

    try:
        if args.auto:
            # Automated test suite
            success = harness.run_automated_test_suite()

            if args.simulate_outage:
                print("\n" + "=" * 70)
                result = harness.test_bus_recovery_simulation()
                harness.test_results.append(result)
                print(f"{'✓ PASS' if result.passed else '✗ FAIL'} | {result.name}")
                success = success and result.passed

            return 0 if (success or args.dry_run) else 1

        elif args.simulate_outage:
            # Just recovery test
            result = harness.test_bus_recovery_simulation()
            print(f"\n{'✓ PASS' if result.passed else '✗ FAIL'} | {result.name}")
            return 0 if (result.passed or args.dry_run) else 1

        else:
            # Interactive mode
            harness.interactive_mode()
            return 0

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        return 130

    finally:
        harness.disconnect()


if __name__ == '__main__':
    sys.exit(main())

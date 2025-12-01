#!/usr/bin/env python3
"""
Batch Control Script for Multiple CANopen Relay Nodes

Send the same command to multiple relay nodes sequentially.

Usage:
    python3 batch_control.py --node-ids 21,22,23 --command 99  # ALL ON
    python3 batch_control.py --node-ids 0x15,0x16,0x17 --command 0  # ALL OFF
"""

import argparse
import sys
import time

try:
    import can
    from can import Message
except ImportError:
    print("ERROR: python-can not installed. Run: pip3 install python-can", file=sys.stderr)
    sys.exit(1)


# CANopen constants
OD_CONTROL_COMMAND = 0x2000
SDO_WRITE = 0x23


def send_command_to_node(bus: can.Bus, node_id: int, command: int) -> bool:
    """Send SDO command to a single node"""
    cob_rsdo = 0x600 + node_id

    payload = [
        SDO_WRITE,
        OD_CONTROL_COMMAND & 0xFF,
        (OD_CONTROL_COMMAND >> 8) & 0xFF,
        0x00,  # subindex
        command & 0xFF,
        0, 0, 0
    ]

    msg = Message(
        arbitration_id=cob_rsdo,
        data=payload,
        is_extended_id=False
    )

    try:
        bus.send(msg)
        print(f"  ✓ Node 0x{node_id:02X}: Command {command} sent")
        return True
    except Exception as e:
        print(f"  ✗ Node 0x{node_id:02X}: Failed - {e}", file=sys.stderr)
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Send command to multiple CANopen relay nodes",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('--node-ids', '-n', required=True,
                       help='Comma-separated node IDs (decimal or hex with 0x prefix)')
    parser.add_argument('--command', '-c', type=int, required=True,
                       help='Command to send (0-8, 99, 254)')
    parser.add_argument('--interface', '-i', default='can0',
                       help='CAN interface (default: can0)')
    parser.add_argument('--delay', '-d', type=float, default=0.2,
                       help='Delay between commands in seconds (default: 0.2)')

    args = parser.parse_args()

    # Parse node IDs
    node_ids = []
    for node_id_str in args.node_ids.split(','):
        try:
            node_id = int(node_id_str, 16 if node_id_str.startswith('0x') else 10)
            if node_id < 1 or node_id > 127:
                print(f"ERROR: Invalid node ID {node_id} (must be 1-127)", file=sys.stderr)
                return 1
            node_ids.append(node_id)
        except ValueError:
            print(f"ERROR: Invalid node ID format: {node_id_str}", file=sys.stderr)
            return 1

    # Validate command
    if args.command not in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 99, 254]:
        print(f"WARNING: Command {args.command} may be invalid", file=sys.stderr)

    # Connect to CAN bus
    try:
        print(f"Connecting to {args.interface}...")
        bus = can.Bus(interface='socketcan', channel=args.interface, bitrate=1000000)
    except Exception as e:
        print(f"ERROR: Failed to connect to CAN bus: {e}", file=sys.stderr)
        return 1

    # Send commands
    print(f"\nSending command {args.command} to {len(node_ids)} nodes:")
    success_count = 0

    for node_id in node_ids:
        if send_command_to_node(bus, node_id, args.command):
            success_count += 1

        if args.delay > 0:
            time.sleep(args.delay)

    bus.shutdown()

    # Summary
    print(f"\nComplete: {success_count}/{len(node_ids)} successful")

    return 0 if success_count == len(node_ids) else 1


if __name__ == '__main__':
    sys.exit(main())

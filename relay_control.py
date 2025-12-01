#!/usr/bin/env python3
"""
Simple relay control script for CANopen nodes.
Usage: python3 relay_control.py k1_on
       python3 relay_control.py k1_off
       python3 relay_control.py all_off
       
Note: Use NODE_ALL_ID (0x10) for all-in-one firmware, or NODE_DRL_ID (0x11) for role-specific.
"""

import can
import sys
import time

# CANopen configuration
NODE_ALL_ID = 0x10  # All-in-one node (ROLE_ALL)
NODE_DRL_ID = 0x11  # DRL-only node (ROLE_DRL)
RSDO_BASE = 0x600
TSDO_BASE = 0x580

# Auto-detect which node to use (default to ALL node)
DEFAULT_NODE_ID = NODE_ALL_ID

# Command codes (from firmware)
CMD_ALL_OFF = 0
CMD_K1_ON = 1
CMD_K1_OFF = 2
CMD_K2_ON = 3
CMD_K2_OFF = 4
CMD_K3_ON = 5
CMD_K3_OFF = 6
CMD_K4_ON = 7
CMD_K4_OFF = 8
CMD_ALL_ON = 99

COMMANDS = {
    'all_off': CMD_ALL_OFF,
    'k1_on': CMD_K1_ON,
    'k1_off': CMD_K1_OFF,
    'k2_on': CMD_K2_ON,
    'k2_off': CMD_K2_OFF,
    'k3_on': CMD_K3_ON,
    'k3_off': CMD_K3_OFF,
    'k4_on': CMD_K4_ON,
    'k4_off': CMD_K4_OFF,
    'all_on': CMD_ALL_ON,
}


def send_command(bus, node_id, command_byte):
    """Send SDO write command to node."""
    cob_id = RSDO_BASE + node_id
    
    # SDO write: index 0x2000, subindex 0x00, data = command_byte
    data = [
        0x23,           # SDO write (1 byte)
        0x00, 0x20,     # Index 0x2000 (little-endian)
        0x00,           # Subindex 0x00
        command_byte,   # Command data
        0x00, 0x00, 0x00  # Padding
    ]
    
    msg = can.Message(
        arbitration_id=cob_id,
        data=data,
        is_extended_id=False
    )
    
    bus.send(msg)
    print(f"✓ Sent command {command_byte} to node 0x{node_id:02x}")
    
    # Wait for responses
    time.sleep(0.3)
    
    # Read responses
    response_cob = TSDO_BASE + node_id
    responses = []
    timeout = time.time() + 0.5
    
    while time.time() < timeout:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == response_cob:
            responses.append(msg)
            if msg.data[0] == 0x23 and msg.data[1] == 0x01 and msg.data[2] == 0x20:
                # Relay feedback message (last response)
                bitmap = msg.data[4]
                print(f"  Relay state: 0b{bitmap:08b} (K1={'ON' if bitmap & 0x01 else 'OFF'})")
                break
    
    return len(responses) > 0


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 relay_control.py <command>")
        print("\nAvailable commands:")
        for cmd in sorted(COMMANDS.keys()):
            print(f"  {cmd}")
        sys.exit(1)
    
    command_name = sys.argv[1].lower()
    
    if command_name not in COMMANDS:
        print(f"Error: Unknown command '{command_name}'")
        print(f"Available: {', '.join(sorted(COMMANDS.keys()))}")
        sys.exit(1)
    
    command_byte = COMMANDS[command_name]
    
    # Connect to CAN bus
    try:
        bus = can.Bus('can0', interface='socketcan', bitrate=1000000)
        print(f"Connected to can0")
    except Exception as e:
        print(f"Error connecting to can0: {e}")
        sys.exit(1)
    
    # Send command to DRL node
    try:
        send_command(bus, DEFAULT_NODE_ID, command_byte)
    except Exception as e:
        print(f"Error sending command: {e}")
    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()

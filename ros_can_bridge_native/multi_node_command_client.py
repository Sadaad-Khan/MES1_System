#!/usr/bin/env python3
"""
Multi-Node Command Client.

Interactive and programmatic command interface for multi-node CANopen relay network.
Supports broadcast commands, per-node targeting, and batch operations.
"""

import argparse
import sys
import yaml
from pathlib import Path
from typing import List, Dict, Optional
import can
import time


class MultiNodeCommandClient:
    """Command client for multi-node CANopen relay network."""
    
    # CANopen constants
    RSDO_BASE = 0x600
    OD_CONTROL_COMMAND = 0x2000
    SDO_WRITE_1BYTE = 0x2F
    
    # Command codes
    COMMANDS = {
        'all_off': 0,
        'k1_on': 1,
        'k1_off': 2,
        'k2_on': 3,
        'k2_off': 4,
        'k3_on': 5,
        'k3_off': 6,
        'k4_on': 7,
        'k4_off': 8,
        'all_on': 99,
        'reset': 254,
    }
    
    # Convenience aliases
    ALIASES = {
        'drl_on': ('relay_drl', 1),
        'drl_off': ('relay_drl', 2),
        'safety_on': ('relay_safety', 3),
        'safety_off': ('relay_safety', 4),
        'battery_right_on': ('relay_battery', 5),
        'battery_right_off': ('relay_battery', 6),
        'battery_left_on': ('relay_battery', 7),
        'battery_left_off': ('relay_battery', 8),
        'battery_both_on': [('relay_battery', 5), ('relay_battery', 7)],
        'battery_both_off': [('relay_battery', 6), ('relay_battery', 8)],
    }
    
    def __init__(self, can_interface: str = 'can0', config_file: Optional[str] = None):
        """
        Initialize command client.
        
        Args:
            can_interface: CAN interface name
            config_file: Path to multi_node_map.yaml
        """
        self.can_interface = can_interface
        self.config_file = config_file
        self.nodes: Dict[str, Dict] = {}
        self.nodes_by_id: Dict[int, Dict] = {}
        self.can_bus: Optional[can.Bus] = None
        
        # Load configuration
        self._load_config()
        
        # Initialize CAN bus
        self._init_can_bus()
    
    def _load_config(self):
        """Load multi-node configuration."""
        if not self.config_file:
            # Try default location
            default_config = Path(__file__).parent.parent / 'config' / 'multi_node_map.yaml'
            if default_config.exists():
                self.config_file = str(default_config)
            else:
                print("Warning: No config file found", file=sys.stderr)
                return
        
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            for node_cfg in config.get('nodes', []):
                if not node_cfg.get('enabled', True):
                    continue
                
                name = node_cfg['name']
                node_id = node_cfg['node_id']
                
                self.nodes[name] = {
                    'node_id': node_id,
                    'role': node_cfg['role'],
                    'description': node_cfg.get('description', ''),
                    'relays': node_cfg.get('relays', []),
                    'commands': node_cfg.get('supported_commands', [])
                }
                
                self.nodes_by_id[node_id] = self.nodes[name]
            
            print(f"Loaded configuration for {len(self.nodes)} nodes")
            
        except Exception as e:
            print(f"Error loading config: {e}", file=sys.stderr)
    
    def _init_can_bus(self):
        """Initialize CAN bus connection."""
        try:
            self.can_bus = can.Bus(
                interface='socketcan',
                channel=self.can_interface,
                bitrate=1000000
            )
            print(f"Connected to {self.can_interface}")
        except Exception as e:
            print(f"Failed to connect to CAN bus: {e}", file=sys.stderr)
            self.can_bus = None
    
    def send_command(self, node_id: int, command: int) -> bool:
        """
        Send command to specific node.
        
        Args:
            node_id: Target node ID
            command: Command code
        
        Returns:
            True if sent successfully
        """
        if not self.can_bus:
            print("CAN bus not connected", file=sys.stderr)
            return False
        
        # Construct SDO write
        sdo_data = bytes([
            self.SDO_WRITE_1BYTE,
            self.OD_CONTROL_COMMAND & 0xFF,
            (self.OD_CONTROL_COMMAND >> 8) & 0xFF,
            0x00,  # subindex
            command,
            0, 0, 0
        ])
        
        can_id = self.RSDO_BASE + node_id
        msg = can.Message(
            arbitration_id=can_id,
            data=sdo_data,
            is_extended_id=False
        )
        
        try:
            self.can_bus.send(msg)
            print(f"✓ Sent command {command} to node 0x{node_id:02X}")
            return True
        except Exception as e:
            print(f"✗ Failed to send command: {e}", file=sys.stderr)
            return False
    
    def broadcast_command(self, command: int, target_names: Optional[List[str]] = None):
        """
        Broadcast command to multiple nodes.
        
        Args:
            command: Command code
            target_names: List of node names (None = all nodes)
        """
        targets = []
        
        if target_names:
            for name in target_names:
                if name in self.nodes:
                    targets.append(self.nodes[name])
                else:
                    print(f"Warning: Unknown node '{name}'", file=sys.stderr)
        else:
            targets = list(self.nodes.values())
        
        print(f"Broadcasting command {command} to {len(targets)} nodes...")
        
        success_count = 0
        for node in targets:
            if self.send_command(node['node_id'], command):
                success_count += 1
            time.sleep(0.05)  # 50ms delay between broadcasts
        
        print(f"Broadcast complete: {success_count}/{len(targets)} successful")
    
    def execute_alias(self, alias: str) -> bool:
        """
        Execute command alias.
        
        Args:
            alias: Alias name (e.g., 'drl_on', 'battery_both_on')
        
        Returns:
            True if executed successfully
        """
        if alias not in self.ALIASES:
            print(f"Unknown alias: {alias}", file=sys.stderr)
            return False
        
        alias_def = self.ALIASES[alias]
        
        # Single command
        if isinstance(alias_def, tuple):
            node_name, cmd = alias_def
            if node_name not in self.nodes:
                print(f"Unknown node: {node_name}", file=sys.stderr)
                return False
            
            node_id = self.nodes[node_name]['node_id']
            return self.send_command(node_id, cmd)
        
        # Multiple commands (batch)
        elif isinstance(alias_def, list):
            success = True
            for node_name, cmd in alias_def:
                if node_name not in self.nodes:
                    print(f"Unknown node: {node_name}", file=sys.stderr)
                    success = False
                    continue
                
                node_id = self.nodes[node_name]['node_id']
                if not self.send_command(node_id, cmd):
                    success = False
                time.sleep(0.05)
            
            return success
        
        return False
    
    def list_nodes(self):
        """List all configured nodes."""
        print("\n=== Configured Nodes ===")
        print(f"{'Name':<20} {'ID':<8} {'Role':<12} {'Description'}")
        print("-" * 70)
        
        for name, node in sorted(self.nodes.items()):
            print(f"{name:<20} 0x{node['node_id']:02X}    {node['role']:<12} {node['description']}")
        
        print()
    
    def list_commands(self):
        """List all available commands and aliases."""
        print("\n=== Basic Commands ===")
        for name, code in sorted(self.COMMANDS.items()):
            print(f"  {name:<20} (code: {code})")
        
        print("\n=== Command Aliases ===")
        for alias in sorted(self.ALIASES.keys()):
            print(f"  {alias}")
        
        print()
    
    def interactive_mode(self):
        """Run interactive command prompt."""
        print("\n" + "=" * 70)
        print("  Multi-Node CANopen Relay Control - Interactive Mode")
        print("=" * 70)
        print()
        print("Commands:")
        print("  list / ls          - List all nodes")
        print("  help / h           - Show available commands")
        print("  <alias>            - Execute command alias (e.g., 'drl_on')")
        print("  <node> <command>   - Send command to specific node")
        print("  broadcast <cmd>    - Broadcast command to all nodes")
        print("  quit / exit / q    - Exit")
        print()
        
        while True:
            try:
                cmd_input = input("relay> ").strip()
                
                if not cmd_input:
                    continue
                
                parts = cmd_input.split()
                cmd = parts[0].lower()
                
                # Special commands
                if cmd in ['quit', 'exit', 'q']:
                    print("Goodbye!")
                    break
                
                elif cmd in ['list', 'ls']:
                    self.list_nodes()
                
                elif cmd in ['help', 'h']:
                    self.list_commands()
                
                elif cmd == 'broadcast':
                    if len(parts) < 2:
                        print("Usage: broadcast <command>")
                        continue
                    
                    cmd_name = parts[1].lower()
                    if cmd_name in self.COMMANDS:
                        self.broadcast_command(self.COMMANDS[cmd_name])
                    else:
                        print(f"Unknown command: {cmd_name}")
                
                # Check if it's an alias
                elif cmd in self.ALIASES:
                    self.execute_alias(cmd)
                
                # Check if it's node + command
                elif len(parts) == 2:
                    node_name = parts[0]
                    cmd_name = parts[1].lower()
                    
                    if node_name not in self.nodes:
                        print(f"Unknown node: {node_name}")
                        continue
                    
                    if cmd_name not in self.COMMANDS:
                        print(f"Unknown command: {cmd_name}")
                        continue
                    
                    node_id = self.nodes[node_name]['node_id']
                    self.send_command(node_id, self.COMMANDS[cmd_name])
                
                else:
                    print(f"Unknown command: {cmd_input}")
                    print("Type 'help' for available commands")
            
            except KeyboardInterrupt:
                print("\nUse 'quit' to exit")
            except EOFError:
                break
            except Exception as e:
                print(f"Error: {e}", file=sys.stderr)
    
    def close(self):
        """Close CAN bus connection."""
        if self.can_bus:
            self.can_bus.shutdown()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Multi-node CANopen relay control client'
    )
    
    parser.add_argument(
        '--interface', '-i',
        default='can0',
        help='CAN interface (default: can0)'
    )
    
    parser.add_argument(
        '--config', '-c',
        help='Path to multi_node_map.yaml config file'
    )
    
    parser.add_argument(
        '--node-id', '-n',
        type=lambda x: int(x, 0),  # Allow hex (0x11) or decimal
        help='Target node ID (hex or decimal)'
    )
    
    parser.add_argument(
        '--node-name',
        help='Target node name (e.g., relay_drl)'
    )
    
    parser.add_argument(
        '--command', '-cmd',
        help='Command to send (name or code)'
    )
    
    parser.add_argument(
        '--broadcast',
        action='store_true',
        help='Broadcast command to all nodes'
    )
    
    parser.add_argument(
        '--list-nodes',
        action='store_true',
        help='List all configured nodes and exit'
    )
    
    parser.add_argument(
        '--list-commands',
        action='store_true',
        help='List all available commands and exit'
    )
    
    parser.add_argument(
        '--alias',
        help='Execute command alias (e.g., drl_on, battery_both_on)'
    )
    
    args = parser.parse_args()
    
    # Create client
    client = MultiNodeCommandClient(
        can_interface=args.interface,
        config_file=args.config
    )
    
    try:
        # Handle list commands
        if args.list_nodes:
            client.list_nodes()
            return 0
        
        if args.list_commands:
            client.list_commands()
            return 0
        
        # Handle alias execution
        if args.alias:
            success = client.execute_alias(args.alias)
            return 0 if success else 1
        
        # Handle command execution
        if args.command:
            # Parse command
            if args.command.isdigit():
                cmd_code = int(args.command)
            elif args.command.lower() in client.COMMANDS:
                cmd_code = client.COMMANDS[args.command.lower()]
            else:
                print(f"Unknown command: {args.command}", file=sys.stderr)
                return 1
            
            # Determine target
            if args.broadcast:
                client.broadcast_command(cmd_code)
            elif args.node_id is not None:
                client.send_command(args.node_id, cmd_code)
            elif args.node_name:
                if args.node_name not in client.nodes:
                    print(f"Unknown node: {args.node_name}", file=sys.stderr)
                    return 1
                node_id = client.nodes[args.node_name]['node_id']
                client.send_command(node_id, cmd_code)
            else:
                print("Error: Must specify --node-id, --node-name, or --broadcast", 
                      file=sys.stderr)
                return 1
            
            return 0
        
        # No command specified - enter interactive mode
        client.interactive_mode()
        return 0
    
    finally:
        client.close()


if __name__ == '__main__':
    sys.exit(main())

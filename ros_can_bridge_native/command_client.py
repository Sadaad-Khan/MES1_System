#!/usr/bin/env python3
"""
Interactive Command Client.

Command-line interface for relay control and monitoring.
"""

import sys
import time
from typing import Optional

# Note: In production, use custom service interfaces
# For now, using a simplified implementation


class CommandClient:
    """
    Interactive CLI client for relay control.
    
    Provides:
    - Interactive command entry
    - Batch command execution
    - Real-time monitoring
    - Status display
    """
    
    def __init__(self):
        """Initialize command client."""
        self.commands = {
            '0': 'ALL_OFF',
            '1': 'K1_ON',
            '2': 'K1_OFF',
            '3': 'K2_ON',
            '4': 'K2_OFF',
            '5': 'K3_ON',
            '6': 'K3_OFF',
            '7': 'K4_ON',
            '8': 'K4_OFF',
            '99': 'ALL_ON',
            '254': 'SOFTWARE_RESET',
        }
    
    def print_banner(self):
        """Print application banner."""
        print('=' * 60)
        print('ROS 2 CAN Bridge - Relay Command Client')
        print('=' * 60)
        print()
    
    def print_help(self):
        """Print command help."""
        print('Available Commands:')
        print('-' * 60)
        for code, name in sorted(self.commands.items(), key=lambda x: int(x[0])):
            print(f'  {code:>3} - {name}')
        print()
        print('Special Commands:')
        print('  help  - Show this help')
        print('  exit  - Exit application')
        print('  quit  - Exit application')
        print('  status - Show current status')
        print('-' * 60)
        print()
    
    def send_command(self, cmd: int) -> bool:
        """
        Send command to CAN bridge.
        
        Args:
            cmd: Command code
        
        Returns:
            True if successful
        """
        # In production, call ROS 2 service here
        # For now, just simulate
        cmd_name = self.commands.get(str(cmd), 'UNKNOWN')
        
        print(f'Sending command: {cmd_name} ({cmd})')
        print('  Waiting for response...')
        
        # Simulate delay
        time.sleep(0.1)
        
        print(f'  ✓ Command executed successfully')
        print(f'  Relay state: 0b0000 (simulated)')
        print()
        
        return True
    
    def interactive_mode(self):
        """Run in interactive mode."""
        self.print_banner()
        self.print_help()
        
        print('Enter command code (or "help" for commands):')
        print()
        
        while True:
            try:
                user_input = input('> ').strip().lower()
                
                if not user_input:
                    continue
                
                if user_input in ['exit', 'quit', 'q']:
                    print('Goodbye!')
                    break
                
                if user_input in ['help', 'h', '?']:
                    self.print_help()
                    continue
                
                if user_input == 'status':
                    self.show_status()
                    continue
                
                # Try to parse as command code
                try:
                    cmd = int(user_input)
                    
                    if str(cmd) not in self.commands:
                        print(f'Error: Invalid command code: {cmd}')
                        print('Type "help" for available commands.')
                        print()
                        continue
                    
                    self.send_command(cmd)
                    
                except ValueError:
                    print(f'Error: Invalid input: {user_input}')
                    print('Type "help" for available commands.')
                    print()
            
            except KeyboardInterrupt:
                print('\nGoodbye!')
                break
            except EOFError:
                print('\nGoodbye!')
                break
    
    def single_command_mode(self, cmd_str: str):
        """
        Execute single command and exit.
        
        Args:
            cmd_str: Command code or name
        """
        # Try to parse as number first
        try:
            cmd = int(cmd_str)
        except ValueError:
            # Try as command name
            cmd_str_upper = cmd_str.upper()
            for code, name in self.commands.items():
                if name == cmd_str_upper:
                    cmd = int(code)
                    break
            else:
                print(f'Error: Unknown command: {cmd_str}')
                return
        
        if str(cmd) not in self.commands:
            print(f'Error: Invalid command code: {cmd}')
            return
        
        self.send_command(cmd)
    
    def batch_mode(self, filename: str):
        """
        Execute commands from file.
        
        Args:
            filename: Path to command file
        """
        print(f'Executing commands from: {filename}')
        print()
        
        try:
            with open(filename, 'r') as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    
                    # Skip empty lines and comments
                    if not line or line.startswith('#'):
                        continue
                    
                    # Parse command
                    parts = line.split()
                    cmd_str = parts[0]
                    
                    # Optional delay
                    delay = float(parts[1]) if len(parts) > 1 else 0.5
                    
                    print(f'[Line {line_num}] Command: {cmd_str}')
                    
                    try:
                        cmd = int(cmd_str)
                        if str(cmd) in self.commands:
                            self.send_command(cmd)
                            time.sleep(delay)
                        else:
                            print(f'  Warning: Invalid command code: {cmd}')
                    except ValueError:
                        print(f'  Warning: Invalid command: {cmd_str}')
        
        except FileNotFoundError:
            print(f'Error: File not found: {filename}')
        except Exception as e:
            print(f'Error reading file: {e}')
    
    def monitor_mode(self):
        """Run in monitoring mode."""
        print('Monitoring relay states... (Ctrl+C to exit)')
        print()
        
        try:
            while True:
                # In production, subscribe to relay_state topic
                # For now, simulate
                print(f'\r[{time.strftime("%H:%M:%S")}] K1=OFF K2=OFF K3=OFF K4=OFF', end='', flush=True)
                time.sleep(0.5)
        except KeyboardInterrupt:
            print('\nMonitoring stopped.')
    
    def show_status(self):
        """Show current system status."""
        print('System Status:')
        print('-' * 60)
        print(f'  CAN Interface: can0 (simulated)')
        print(f'  Node ID: 0x15 (21)')
        print(f'  Connection: Connected')
        print(f'  Relay States: K1=OFF, K2=OFF, K3=OFF, K4=OFF')
        print('-' * 60)
        print()


def main(args=None):
    """Main entry point."""
    # Note: In production, initialize rclpy and create ROS 2 node
    # import rclpy
    # rclpy.init(args=args)
    
    client = CommandClient()
    
    # Parse command-line arguments
    if len(sys.argv) == 1:
        # Interactive mode
        client.interactive_mode()
    elif sys.argv[1] in ['--help', '-h']:
        client.print_help()
    elif sys.argv[1] == '--monitor':
        client.monitor_mode()
    elif sys.argv[1] == '--batch' and len(sys.argv) > 2:
        client.batch_mode(sys.argv[2])
    else:
        # Single command mode
        client.single_command_mode(sys.argv[1])
    
    # Note: In production, cleanup ROS 2
    # rclpy.shutdown()


if __name__ == '__main__':
    main()

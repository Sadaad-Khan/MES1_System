#!/usr/bin/env python3
"""
Relay Controller.

High-level relay control logic with state management and safety features.
"""

import threading
from typing import Optional, Dict, List
from enum import IntEnum
from dataclasses import dataclass
from datetime import datetime


class RelayCommand(IntEnum):
    """Relay command codes."""
    ALL_OFF = 0
    K1_ON = 1
    K1_OFF = 2
    K2_ON = 3
    K2_OFF = 4
    K3_ON = 5
    K3_OFF = 6
    K4_ON = 7
    K4_OFF = 8
    ALL_ON = 99
    SOFTWARE_RESET = 254


@dataclass
class RelayState:
    """Relay state representation."""
    k1: bool = False
    k2: bool = False
    k3: bool = False
    k4: bool = False
    bitmap: int = 0
    timestamp: Optional[datetime] = None
    
    def from_bitmap(self, bitmap: int):
        """Update state from bitmap."""
        self.k1 = bool(bitmap & 0x01)
        self.k2 = bool(bitmap & 0x02)
        self.k3 = bool(bitmap & 0x04)
        self.k4 = bool(bitmap & 0x08)
        self.bitmap = bitmap
        self.timestamp = datetime.now()
    
    def to_bitmap(self) -> int:
        """Convert state to bitmap."""
        bitmap = 0
        if self.k1:
            bitmap |= 0x01
        if self.k2:
            bitmap |= 0x02
        if self.k3:
            bitmap |= 0x04
        if self.k4:
            bitmap |= 0x08
        return bitmap
    
    def __str__(self) -> str:
        """String representation."""
        return f'K1={self.k1}, K2={self.k2}, K3={self.k3}, K4={self.k4}'


class RelayController:
    """
    High-level relay control with state management.
    
    Features:
    - State tracking and validation
    - Command history
    - Safety interlocks
    - Emergency stop
    """
    
    def __init__(self, enable_interlocks: bool = False):
        """
        Initialize relay controller.
        
        Args:
            enable_interlocks: Enable safety interlocks
        """
        self.current_state = RelayState()
        self.enable_interlocks = enable_interlocks
        self.command_history: List[Dict] = []
        self.max_history = 100
        self.state_lock = threading.Lock()
        self.emergency_stop = False
        
        # Interlock configuration (example: K1 and K2 cannot be on simultaneously)
        self.interlocks = [
            # (relay1, relay2) - these relays cannot be on at the same time
            # ('k1', 'k2'),  # Example interlock
        ]
    
    def validate_command(self, command: int) -> tuple[bool, str]:
        """
        Validate command before execution.
        
        Args:
            command: Command code
        
        Returns:
            Tuple of (valid, error_message)
        """
        if self.emergency_stop:
            return False, 'Emergency stop active'
        
        try:
            cmd_enum = RelayCommand(command)
        except ValueError:
            return False, f'Invalid command code: {command}'
        
        # Check interlocks if enabled
        if self.enable_interlocks:
            # Predict new state after command
            new_state = self._predict_state(command)
            
            # Check interlock violations
            for relay1, relay2 in self.interlocks:
                if getattr(new_state, relay1) and getattr(new_state, relay2):
                    return False, f'Interlock violation: {relay1} and {relay2} cannot be on simultaneously'
        
        return True, ''
    
    def _predict_state(self, command: int) -> RelayState:
        """
        Predict state after command execution.
        
        Args:
            command: Command code
        
        Returns:
            Predicted RelayState
        """
        new_state = RelayState()
        new_state.k1 = self.current_state.k1
        new_state.k2 = self.current_state.k2
        new_state.k3 = self.current_state.k3
        new_state.k4 = self.current_state.k4
        
        cmd = RelayCommand(command)
        
        if cmd == RelayCommand.ALL_OFF:
            new_state.k1 = new_state.k2 = new_state.k3 = new_state.k4 = False
        elif cmd == RelayCommand.ALL_ON:
            new_state.k1 = new_state.k2 = new_state.k3 = new_state.k4 = True
        elif cmd == RelayCommand.K1_ON:
            new_state.k1 = True
        elif cmd == RelayCommand.K1_OFF:
            new_state.k1 = False
        elif cmd == RelayCommand.K2_ON:
            new_state.k2 = True
        elif cmd == RelayCommand.K2_OFF:
            new_state.k2 = False
        elif cmd == RelayCommand.K3_ON:
            new_state.k3 = True
        elif cmd == RelayCommand.K3_OFF:
            new_state.k3 = False
        elif cmd == RelayCommand.K4_ON:
            new_state.k4 = True
        elif cmd == RelayCommand.K4_OFF:
            new_state.k4 = False
        
        return new_state
    
    def update_state(self, bitmap: int):
        """
        Update current state from feedback.
        
        Args:
            bitmap: Relay state bitmap
        """
        with self.state_lock:
            self.current_state.from_bitmap(bitmap)
    
    def get_state(self) -> RelayState:
        """
        Get current relay state (thread-safe).
        
        Returns:
            Current RelayState
        """
        with self.state_lock:
            state = RelayState()
            state.k1 = self.current_state.k1
            state.k2 = self.current_state.k2
            state.k3 = self.current_state.k3
            state.k4 = self.current_state.k4
            state.bitmap = self.current_state.bitmap
            state.timestamp = self.current_state.timestamp
            return state
    
    def add_to_history(self, command: int, success: bool, message: str):
        """
        Add command to history.
        
        Args:
            command: Command code
            success: Execution success
            message: Result message
        """
        entry = {
            'command': command,
            'command_name': RelayCommand(command).name if command in RelayCommand._value2member_map_ else 'UNKNOWN',
            'success': success,
            'message': message,
            'timestamp': datetime.now(),
            'state_before': self.current_state.to_bitmap()
        }
        
        self.command_history.append(entry)
        
        # Limit history size
        if len(self.command_history) > self.max_history:
            self.command_history.pop(0)
    
    def get_history(self, count: Optional[int] = None) -> List[Dict]:
        """
        Get command history.
        
        Args:
            count: Number of recent entries (None for all)
        
        Returns:
            List of history entries
        """
        if count is None:
            return self.command_history.copy()
        else:
            return self.command_history[-count:]
    
    def set_emergency_stop(self, enabled: bool):
        """
        Set emergency stop state.
        
        Args:
            enabled: True to enable emergency stop
        """
        self.emergency_stop = enabled
    
    def clear_emergency_stop(self):
        """Clear emergency stop."""
        self.emergency_stop = False
    
    def get_relay_by_name(self, name: str) -> Optional[bool]:
        """
        Get relay state by name.
        
        Args:
            name: Relay name ('k1', 'k2', 'k3', 'k4')
        
        Returns:
            Relay state or None if invalid name
        """
        name = name.lower()
        with self.state_lock:
            if name == 'k1':
                return self.current_state.k1
            elif name == 'k2':
                return self.current_state.k2
            elif name == 'k3':
                return self.current_state.k3
            elif name == 'k4':
                return self.current_state.k4
        return None
    
    def format_state(self) -> str:
        """
        Format current state as string.
        
        Returns:
            Formatted state string
        """
        state = self.get_state()
        return (f'Relay States: K1={"ON" if state.k1 else "OFF"}, '
                f'K2={"ON" if state.k2 else "OFF"}, '
                f'K3={"ON" if state.k3 else "OFF"}, '
                f'K4={"ON" if state.k4 else "OFF"} '
                f'(0b{state.bitmap:04b})')


# Command name to code mapping
COMMAND_NAME_MAP = {
    'ALL_OFF': RelayCommand.ALL_OFF,
    'K1_ON': RelayCommand.K1_ON,
    'K1_OFF': RelayCommand.K1_OFF,
    'K2_ON': RelayCommand.K2_ON,
    'K2_OFF': RelayCommand.K2_OFF,
    'K3_ON': RelayCommand.K3_ON,
    'K3_OFF': RelayCommand.K3_OFF,
    'K4_ON': RelayCommand.K4_ON,
    'K4_OFF': RelayCommand.K4_OFF,
    'ALL_ON': RelayCommand.ALL_ON,
    'SOFTWARE_RESET': RelayCommand.SOFTWARE_RESET,
}


def get_command_from_name(name: str) -> Optional[int]:
    """
    Get command code from name.
    
    Args:
        name: Command name (e.g., 'K1_ON')
    
    Returns:
        Command code or None if invalid
    """
    return COMMAND_NAME_MAP.get(name.upper())


def get_all_command_names() -> List[str]:
    """
    Get list of all command names.
    
    Returns:
        List of command name strings
    """
    return list(COMMAND_NAME_MAP.keys())

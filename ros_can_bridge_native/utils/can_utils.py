"""
CAN utilities.

Helper functions for CAN frame manipulation and analysis.
"""

from typing import List, Optional
import struct


def format_can_id(can_id: int, extended: bool = False) -> str:
    """
    Format CAN ID for display.
    
    Args:
        can_id: CAN arbitration ID
        extended: True if extended ID (29-bit)
    
    Returns:
        Formatted ID string
    """
    if extended:
        return f'0x{can_id:08X}'
    else:
        return f'0x{can_id:03X}'


def format_can_data(data: bytes) -> str:
    """
    Format CAN data bytes for display.
    
    Args:
        data: Data bytes
    
    Returns:
        Formatted data string (space-separated hex)
    """
    return ' '.join(f'{b:02X}' for b in data)


def parse_can_frame(frame_str: str) -> Optional[tuple]:
    """
    Parse CAN frame from candump format.
    
    Args:
        frame_str: Frame string (e.g., "can0  123#0102030405060708")
    
    Returns:
        Tuple of (interface, can_id, data) or None if invalid
    """
    try:
        parts = frame_str.split()
        if len(parts) < 2:
            return None
        
        interface = parts[0]
        frame_parts = parts[1].split('#')
        
        if len(frame_parts) != 2:
            return None
        
        can_id = int(frame_parts[0], 16)
        data = bytes.fromhex(frame_parts[1]) if frame_parts[1] else b''
        
        return interface, can_id, data
    
    except Exception:
        return None


def calculate_dlc(data: bytes) -> int:
    """
    Calculate DLC (Data Length Code) for CAN frame.
    
    Args:
        data: Frame data
    
    Returns:
        DLC value (0-8)
    """
    return min(len(data), 8)


def validate_can_id(can_id: int, extended: bool = False) -> bool:
    """
    Validate CAN ID range.
    
    Args:
        can_id: CAN arbitration ID
        extended: True if extended ID format
    
    Returns:
        True if valid
    """
    if extended:
        return 0 <= can_id <= 0x1FFFFFFF
    else:
        return 0 <= can_id <= 0x7FF


def is_standard_can_id(can_id: int) -> bool:
    """
    Check if CAN ID is in standard (11-bit) range.
    
    Args:
        can_id: CAN arbitration ID
    
    Returns:
        True if standard ID
    """
    return 0 <= can_id <= 0x7FF


def pack_u8(value: int) -> bytes:
    """Pack unsigned 8-bit value."""
    return struct.pack('B', value)


def pack_u16(value: int, little_endian: bool = True) -> bytes:
    """Pack unsigned 16-bit value."""
    fmt = '<H' if little_endian else '>H'
    return struct.pack(fmt, value)


def pack_u32(value: int, little_endian: bool = True) -> bytes:
    """Pack unsigned 32-bit value."""
    fmt = '<I' if little_endian else '>I'
    return struct.pack(fmt, value)


def pack_s8(value: int) -> bytes:
    """Pack signed 8-bit value."""
    return struct.pack('b', value)


def pack_s16(value: int, little_endian: bool = True) -> bytes:
    """Pack signed 16-bit value."""
    fmt = '<h' if little_endian else '>h'
    return struct.pack(fmt, value)


def pack_s32(value: int, little_endian: bool = True) -> bytes:
    """Pack signed 32-bit value."""
    fmt = '<i' if little_endian else '>i'
    return struct.pack(fmt, value)


def unpack_u8(data: bytes, offset: int = 0) -> int:
    """Unpack unsigned 8-bit value."""
    return struct.unpack_from('B', data, offset)[0]


def unpack_u16(data: bytes, offset: int = 0, little_endian: bool = True) -> int:
    """Unpack unsigned 16-bit value."""
    fmt = '<H' if little_endian else '>H'
    return struct.unpack_from(fmt, data, offset)[0]


def unpack_u32(data: bytes, offset: int = 0, little_endian: bool = True) -> int:
    """Unpack unsigned 32-bit value."""
    fmt = '<I' if little_endian else '>I'
    return struct.unpack_from(fmt, data, offset)[0]


def unpack_s8(data: bytes, offset: int = 0) -> int:
    """Unpack signed 8-bit value."""
    return struct.unpack_from('b', data, offset)[0]


def unpack_s16(data: bytes, offset: int = 0, little_endian: bool = True) -> int:
    """Unpack signed 16-bit value."""
    fmt = '<h' if little_endian else '>h'
    return struct.unpack_from(fmt, data, offset)[0]


def unpack_s32(data: bytes, offset: int = 0, little_endian: bool = True) -> int:
    """Unpack signed 32-bit value."""
    fmt = '<i' if little_endian else '>i'
    return struct.unpack_from(fmt, data, offset)[0]


def calculate_bus_load(frames_per_second: int, avg_dlc: int, bitrate: int) -> float:
    """
    Estimate CAN bus load.
    
    Args:
        frames_per_second: Frame rate
        avg_dlc: Average data length
        bitrate: Bus bitrate
    
    Returns:
        Bus load (0.0 to 1.0)
    """
    # CAN frame overhead: ~47 bits minimum + 10 bits per data byte
    bits_per_frame = 47 + (avg_dlc * 10)
    bits_per_second = frames_per_second * bits_per_frame
    
    return min(bits_per_second / bitrate, 1.0)


def get_can_error_state(tx_errors: int, rx_errors: int) -> str:
    """
    Determine CAN error state from error counters.
    
    Args:
        tx_errors: TX error count
        rx_errors: RX error count
    
    Returns:
        Error state string
    """
    max_errors = max(tx_errors, rx_errors)
    
    if max_errors >= 256:
        return 'BUS_OFF'
    elif max_errors >= 128:
        return 'ERROR_PASSIVE'
    else:
        return 'ERROR_ACTIVE'


def format_bitrate(bitrate: int) -> str:
    """
    Format bitrate for display.
    
    Args:
        bitrate: Bitrate in bps
    
    Returns:
        Formatted string (e.g., "1 Mbps", "500 kbps")
    """
    if bitrate >= 1_000_000:
        return f'{bitrate // 1_000_000} Mbps'
    elif bitrate >= 1_000:
        return f'{bitrate // 1_000} kbps'
    else:
        return f'{bitrate} bps'

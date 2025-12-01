"""
CANopen utilities.

Helper functions for CANopen protocol operations.
"""

from typing import Dict, Tuple
import struct


# CANopen function codes
NMT_FUNCTION_CODE = 0x000
SYNC_FUNCTION_CODE = 0x080
EMERGENCY_FUNCTION_CODE = 0x080
TIMESTAMP_FUNCTION_CODE = 0x100
TPDO1_FUNCTION_CODE = 0x180
RPDO1_FUNCTION_CODE = 0x200
TPDO2_FUNCTION_CODE = 0x280
RPDO2_FUNCTION_CODE = 0x300
TPDO3_FUNCTION_CODE = 0x380
RPDO3_FUNCTION_CODE = 0x400
TPDO4_FUNCTION_CODE = 0x480
RPDO4_FUNCTION_CODE = 0x500
TSDO_FUNCTION_CODE = 0x580
RSDO_FUNCTION_CODE = 0x600
HEARTBEAT_FUNCTION_CODE = 0x700


def calculate_cob_id(function_code: int, node_id: int) -> int:
    """
    Calculate CANopen COB-ID.
    
    Args:
        function_code: CANopen function code
        node_id: Node ID (1-127)
    
    Returns:
        COB-ID
    """
    return function_code + node_id


def parse_cob_id(cob_id: int) -> Tuple[int, int]:
    """
    Parse CANopen COB-ID into function code and node ID.
    
    Args:
        cob_id: COB-ID to parse
    
    Returns:
        Tuple of (function_code, node_id)
    """
    # Determine function code by finding the highest matching base
    function_codes = [
        0x700, 0x680, 0x600, 0x580, 0x500, 0x480,
        0x400, 0x380, 0x300, 0x280, 0x200, 0x180,
        0x100, 0x080, 0x000
    ]
    
    for fc in function_codes:
        if cob_id >= fc and cob_id < fc + 128:
            return fc, cob_id - fc
    
    return 0, 0


def get_cob_id_name(cob_id: int) -> str:
    """
    Get human-readable name for COB-ID.
    
    Args:
        cob_id: COB-ID
    
    Returns:
        Name string
    """
    function_code, node_id = parse_cob_id(cob_id)
    
    names = {
        0x000: 'NMT',
        0x080: 'SYNC/EMCY',
        0x100: 'TIMESTAMP',
        0x180: 'TPDO1',
        0x200: 'RPDO1',
        0x280: 'TPDO2',
        0x300: 'RPDO2',
        0x380: 'TPDO3',
        0x400: 'RPDO3',
        0x480: 'TPDO4',
        0x500: 'RPDO4',
        0x580: 'TSDO',
        0x600: 'RSDO',
        0x680: 'HEARTBEAT_TX',
        0x700: 'HEARTBEAT',
    }
    
    base_name = names.get(function_code, 'UNKNOWN')
    
    if node_id > 0:
        return f'{base_name}[{node_id}]'
    else:
        return base_name


def encode_sdo_write(index: int, subindex: int, data: bytes) -> bytes:
    """
    Encode SDO write request (expedited).
    
    Args:
        index: Object dictionary index
        subindex: Subindex
        data: Data to write (1, 2, or 4 bytes)
    
    Returns:
        SDO frame data (8 bytes)
    """
    if len(data) == 1:
        cmd = 0x2F
    elif len(data) == 2:
        cmd = 0x2B
    elif len(data) == 4:
        cmd = 0x23
    else:
        raise ValueError('Data length must be 1, 2, or 4 bytes')
    
    frame = bytearray(8)
    frame[0] = cmd
    frame[1:3] = struct.pack('<H', index)
    frame[3] = subindex
    frame[4:4+len(data)] = data
    
    return bytes(frame)


def encode_sdo_read(index: int, subindex: int) -> bytes:
    """
    Encode SDO read request.
    
    Args:
        index: Object dictionary index
        subindex: Subindex
    
    Returns:
        SDO frame data (8 bytes)
    """
    frame = bytearray(8)
    frame[0] = 0x40  # Read command
    frame[1:3] = struct.pack('<H', index)
    frame[3] = subindex
    
    return bytes(frame)


def decode_sdo_response(data: bytes) -> Dict:
    """
    Decode SDO response.
    
    Args:
        data: SDO frame data (8 bytes)
    
    Returns:
        Dictionary with decoded information
    """
    if len(data) < 4:
        return {'valid': False}
    
    cmd = data[0]
    index = struct.unpack('<H', data[1:3])[0]
    subindex = data[3]
    
    result = {
        'valid': True,
        'command': cmd,
        'index': index,
        'subindex': subindex,
    }
    
    if cmd == 0x80:  # Abort
        if len(data) >= 8:
            abort_code = struct.unpack('<I', data[4:8])[0]
            result['abort'] = True
            result['abort_code'] = abort_code
        else:
            result['abort'] = True
            result['abort_code'] = 0
    else:
        result['abort'] = False
        if len(data) >= 8:
            result['data'] = data[4:8]
    
    return result


def format_object_dict_address(index: int, subindex: int) -> str:
    """
    Format object dictionary address.
    
    Args:
        index: Index
        subindex: Subindex
    
    Returns:
        Formatted string (e.g., "0x2000:0x00")
    """
    return f'0x{index:04X}:0x{subindex:02X}'


def parse_object_dict_address(address: str) -> Tuple[int, int]:
    """
    Parse object dictionary address string.
    
    Args:
        address: Address string (e.g., "0x2000:0x00" or "0x2000")
    
    Returns:
        Tuple of (index, subindex)
    """
    parts = address.split(':')
    
    if len(parts) == 1:
        index = int(parts[0], 16)
        subindex = 0
    else:
        index = int(parts[0], 16)
        subindex = int(parts[1], 16)
    
    return index, subindex


def get_nmt_command_name(command: int) -> str:
    """
    Get NMT command name.
    
    Args:
        command: NMT command code
    
    Returns:
        Command name
    """
    names = {
        0x01: 'START',
        0x02: 'STOP',
        0x80: 'PRE_OPERATIONAL',
        0x81: 'RESET_NODE',
        0x82: 'RESET_COMMUNICATION',
    }
    
    return names.get(command, f'UNKNOWN(0x{command:02X})')


def get_heartbeat_state_name(state: int) -> str:
    """
    Get heartbeat state name.
    
    Args:
        state: Heartbeat state code
    
    Returns:
        State name
    """
    states = {
        0x00: 'BOOTUP',
        0x04: 'STOPPED',
        0x05: 'OPERATIONAL',
        0x7F: 'PRE_OPERATIONAL',
    }
    
    return states.get(state, f'UNKNOWN(0x{state:02X})')


def validate_node_id(node_id: int) -> bool:
    """
    Validate CANopen node ID.
    
    Args:
        node_id: Node ID to validate
    
    Returns:
        True if valid
    """
    return 0x01 <= node_id <= 0x7F


def get_standard_object_dict_name(index: int) -> str:
    """
    Get standard object dictionary entry name.
    
    Args:
        index: Object dictionary index
    
    Returns:
        Entry name or hex string if unknown
    """
    standard_objects = {
        0x1000: 'Device Type',
        0x1001: 'Error Register',
        0x1002: 'Manufacturer Status Register',
        0x1003: 'Pre-defined Error Field',
        0x1005: 'COB-ID SYNC',
        0x1006: 'Communication Cycle Period',
        0x1007: 'Synchronous Window Length',
        0x1008: 'Manufacturer Device Name',
        0x1009: 'Manufacturer Hardware Version',
        0x100A: 'Manufacturer Software Version',
        0x1010: 'Store Parameters',
        0x1011: 'Restore Default Parameters',
        0x1014: 'COB-ID EMCY',
        0x1015: 'Inhibit Time EMCY',
        0x1016: 'Consumer Heartbeat Time',
        0x1017: 'Producer Heartbeat Time',
        0x1018: 'Identity Object',
    }
    
    return standard_objects.get(index, f'0x{index:04X}')

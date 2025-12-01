#!/usr/bin/env python3
"""
CANopen SDO Handler.

Handles CANopen SDO protocol encoding, decoding, and transaction management.
"""

import struct
from typing import Optional, Tuple, List
from dataclasses import dataclass
from enum import IntEnum


class SDOCommand(IntEnum):
    """SDO command specifiers."""
    WRITE_1BYTE = 0x2F
    WRITE_2BYTE = 0x2B
    WRITE_4BYTE = 0x23
    READ = 0x40
    RESPONSE = 0x60
    ABORT = 0x80


class SDOAbortCode(IntEnum):
    """Standard CANopen SDO abort codes."""
    TOGGLE_BIT = 0x05030000
    SDO_TIMEOUT = 0x05040000
    INVALID_CS = 0x05040001
    INVALID_BLOCK_SIZE = 0x05040002
    INVALID_SEQ_NUM = 0x05040003
    CRC_ERROR = 0x05040004
    OUT_OF_MEMORY = 0x05040005
    UNSUPPORTED_ACCESS = 0x06010000
    WRITE_ONLY = 0x06010001
    READ_ONLY = 0x06010002
    OBJECT_NOT_EXISTS = 0x06020000
    PDO_MAPPING = 0x06040041
    PDO_LENGTH = 0x06040042
    PARAMETER_INCOMPATIBILITY = 0x06040043
    INTERNAL_INCOMPATIBILITY = 0x06040047
    HARDWARE_ERROR = 0x06060000
    DATA_TYPE_MISMATCH = 0x06070010
    DATA_TOO_LONG = 0x06070012
    DATA_TOO_SHORT = 0x06070013
    SUBINDEX_NOT_EXISTS = 0x06090011
    INVALID_VALUE = 0x06090030
    VALUE_TOO_HIGH = 0x06090031
    VALUE_TOO_LOW = 0x06090032
    MAX_VALUE_LESS_MIN = 0x06090036
    RESOURCE_NOT_AVAILABLE = 0x060A0023
    GENERAL_ERROR = 0x08000000
    DATA_TRANSFER_ERROR = 0x08000020
    DATA_LOCAL_CONTROL = 0x08000021
    DATA_DEVICE_STATE = 0x08000022
    OBJECT_DICTIONARY_ERROR = 0x08000023
    NO_DATA_AVAILABLE = 0x08000024


@dataclass
class SDORequest:
    """SDO request message."""
    index: int
    subindex: int
    data: bytes
    command: SDOCommand
    timestamp: float


@dataclass
class SDOResponse:
    """SDO response message."""
    index: int
    subindex: int
    data: Optional[bytes]
    success: bool
    error_code: int
    timestamp: float


class CANopenSDOHandler:
    """
    Handles CANopen SDO protocol operations.
    
    Features:
    - Expedited SDO transfers (1-4 bytes)
    - Request/response correlation
    - Error code interpretation
    - Transaction logging
    """
    
    def __init__(self, node_id: int):
        """
        Initialize SDO handler.
        
        Args:
            node_id: CANopen node ID (0x01-0x7F)
        """
        self.node_id = node_id
        self.rsdo_id = 0x600 + node_id  # Receive SDO
        self.tsdo_id = 0x580 + node_id  # Transmit SDO
    
    def encode_write_request(
        self,
        index: int,
        subindex: int,
        data: bytes
    ) -> Tuple[int, bytes]:
        """
        Encode SDO write request (expedited transfer).
        
        Args:
            index: Object dictionary index (0x0000-0xFFFF)
            subindex: Object dictionary subindex (0x00-0xFF)
            data: Data to write (1, 2, or 4 bytes)
        
        Returns:
            Tuple of (CAN ID, data bytes)
        
        Raises:
            ValueError: If data length is invalid
        """
        if len(data) == 1:
            cmd = SDOCommand.WRITE_1BYTE
        elif len(data) == 2:
            cmd = SDOCommand.WRITE_2BYTE
        elif len(data) == 4:
            cmd = SDOCommand.WRITE_4BYTE
        else:
            raise ValueError(
                f'Invalid data length: {len(data)}. Must be 1, 2, or 4 bytes')
        
        # Build SDO frame
        frame = bytearray(8)
        frame[0] = cmd
        frame[1:3] = struct.pack('<H', index)  # Little-endian 16-bit index
        frame[3] = subindex
        frame[4:4+len(data)] = data
        
        return self.rsdo_id, bytes(frame)
    
    def encode_read_request(
        self,
        index: int,
        subindex: int
    ) -> Tuple[int, bytes]:
        """
        Encode SDO read request.
        
        Args:
            index: Object dictionary index
            subindex: Object dictionary subindex
        
        Returns:
            Tuple of (CAN ID, data bytes)
        """
        frame = bytearray(8)
        frame[0] = SDOCommand.READ
        frame[1:3] = struct.pack('<H', index)
        frame[3] = subindex
        
        return self.rsdo_id, bytes(frame)
    
    def decode_response(
        self,
        can_id: int,
        data: bytes
    ) -> Optional[SDOResponse]:
        """
        Decode SDO response.
        
        Args:
            can_id: CAN arbitration ID
            data: CAN frame data (8 bytes)
        
        Returns:
            SDOResponse object or None if not a valid response
        """
        # Verify this is an SDO response for our node
        if can_id != self.tsdo_id:
            return None
        
        if len(data) < 4:
            return None
        
        cmd = data[0]
        index = struct.unpack('<H', data[1:3])[0]
        subindex = data[3]
        
        # Check for abort
        if cmd == SDOCommand.ABORT:
            if len(data) >= 8:
                error_code = struct.unpack('<I', data[4:8])[0]
            else:
                error_code = 0
            
            return SDOResponse(
                index=index,
                subindex=subindex,
                data=None,
                success=False,
                error_code=error_code,
                timestamp=0.0  # Should be set by caller
            )
        
        # Success response - extract data
        response_data = None
        if len(data) >= 8:
            # Determine data length from command specifier
            if cmd == SDOCommand.WRITE_1BYTE or cmd == 0x60:
                response_data = data[4:5]
            elif cmd == SDOCommand.WRITE_2BYTE or cmd == 0x4B:
                response_data = data[4:6]
            elif cmd == SDOCommand.WRITE_4BYTE or cmd == 0x43:
                response_data = data[4:8]
            else:
                response_data = data[4:8]  # Default to 4 bytes
        
        return SDOResponse(
            index=index,
            subindex=subindex,
            data=response_data,
            success=True,
            error_code=0,
            timestamp=0.0
        )
    
    def get_abort_code_description(self, abort_code: int) -> str:
        """
        Get human-readable description of abort code.
        
        Args:
            abort_code: SDO abort code
        
        Returns:
            Description string
        """
        abort_descriptions = {
            SDOAbortCode.TOGGLE_BIT: 'Toggle bit not alternated',
            SDOAbortCode.SDO_TIMEOUT: 'SDO protocol timed out',
            SDOAbortCode.INVALID_CS: 'Invalid or unknown command specifier',
            SDOAbortCode.UNSUPPORTED_ACCESS: 'Unsupported access to object',
            SDOAbortCode.WRITE_ONLY: 'Attempt to read a write-only object',
            SDOAbortCode.READ_ONLY: 'Attempt to write a read-only object',
            SDOAbortCode.OBJECT_NOT_EXISTS: 'Object does not exist',
            SDOAbortCode.HARDWARE_ERROR: 'Hardware error',
            SDOAbortCode.DATA_TYPE_MISMATCH: 'Data type does not match',
            SDOAbortCode.DATA_TOO_LONG: 'Data length too long',
            SDOAbortCode.DATA_TOO_SHORT: 'Data length too short',
            SDOAbortCode.SUBINDEX_NOT_EXISTS: 'Subindex does not exist',
            SDOAbortCode.INVALID_VALUE: 'Invalid value for parameter',
            SDOAbortCode.VALUE_TOO_HIGH: 'Value too high',
            SDOAbortCode.VALUE_TOO_LOW: 'Value too low',
            SDOAbortCode.GENERAL_ERROR: 'General error',
            SDOAbortCode.DATA_TRANSFER_ERROR: 'Data cannot be transferred',
        }
        
        return abort_descriptions.get(
            abort_code,
            f'Unknown abort code: 0x{abort_code:08X}'
        )
    
    def validate_node_id(self, node_id: int) -> bool:
        """
        Validate CANopen node ID.
        
        Args:
            node_id: Node ID to validate
        
        Returns:
            True if valid, False otherwise
        """
        return 0x01 <= node_id <= 0x7F
    
    def calculate_cob_ids(self, node_id: int) -> dict:
        """
        Calculate all COB-IDs for a node.
        
        Args:
            node_id: CANopen node ID
        
        Returns:
            Dictionary of COB-ID types and values
        """
        return {
            'rsdo': 0x600 + node_id,
            'tsdo': 0x580 + node_id,
            'heartbeat': 0x700 + node_id,
            'nmt': 0x000,  # NMT is broadcast
            'sync': 0x080,  # SYNC is broadcast
            'emergency': 0x080 + node_id,
            'tpdo1': 0x180 + node_id,
            'tpdo2': 0x280 + node_id,
            'tpdo3': 0x380 + node_id,
            'tpdo4': 0x480 + node_id,
            'rpdo1': 0x200 + node_id,
            'rpdo2': 0x300 + node_id,
            'rpdo3': 0x400 + node_id,
            'rpdo4': 0x500 + node_id,
        }


def format_sdo_frame(can_id: int, data: bytes) -> str:
    """
    Format SDO frame for logging.
    
    Args:
        can_id: CAN arbitration ID
        data: Frame data
    
    Returns:
        Formatted string
    """
    data_hex = ' '.join(f'{b:02X}' for b in data)
    return f'ID: 0x{can_id:03X}  Data: {data_hex}'


def parse_object_dictionary_address(address_str: str) -> Tuple[int, int]:
    """
    Parse object dictionary address string.
    
    Args:
        address_str: Address in format '0x2000' or '0x2000:0x00'
    
    Returns:
        Tuple of (index, subindex)
    
    Raises:
        ValueError: If format is invalid
    """
    parts = address_str.split(':')
    
    if len(parts) == 1:
        index = int(parts[0], 16)
        subindex = 0
    elif len(parts) == 2:
        index = int(parts[0], 16)
        subindex = int(parts[1], 16)
    else:
        raise ValueError(f'Invalid OD address format: {address_str}')
    
    return index, subindex

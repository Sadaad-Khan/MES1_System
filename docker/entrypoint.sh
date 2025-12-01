#!/bin/bash
set -e

# Setup ROS 2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WORKSPACE}/install/setup.bash

# Setup CAN interface if CAN_INTERFACE is set
if [ -n "$CAN_INTERFACE" ]; then
    echo "Setting up CAN interface: $CAN_INTERFACE"
    
    # Check if interface exists
    if ip link show "$CAN_INTERFACE" &>/dev/null; then
        echo "Configuring $CAN_INTERFACE with bitrate ${CAN_BITRATE:-1000000}"
        
        # Bring down if up
        ip link set "$CAN_INTERFACE" down 2>/dev/null || true
        
        # Configure interface
        ip link set "$CAN_INTERFACE" type can bitrate "${CAN_BITRATE:-1000000}"
        ip link set "$CAN_INTERFACE" txqueuelen 1000
        
        # Bring up
        ip link set "$CAN_INTERFACE" up
        
        echo "✓ CAN interface $CAN_INTERFACE is ready"
        ip -details link show "$CAN_INTERFACE"
    else
        echo "⚠ Warning: CAN interface $CAN_INTERFACE not found"
        echo "Available interfaces:"
        ip link show | grep -E "^[0-9]+:"
    fi
fi

# Execute the main command
exec "$@"

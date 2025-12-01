#!/bin/bash
# SocketCAN setup script for gs_usb devices
# Configures CAN interface for production use

set -e

# Configuration
CAN_INTERFACE="${1:-can0}"
BITRATE="${2:-1000000}"
TXQUEUELEN=1000

echo "========================================="
echo "SocketCAN Setup Script"
echo "========================================="
echo "Interface: $CAN_INTERFACE"
echo "Bitrate: $BITRATE bps"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root (use sudo)"
    exit 1
fi

# Load kernel modules
echo "Loading kernel modules..."
modprobe can 2>/dev/null || true
modprobe can_raw 2>/dev/null || true
modprobe slcan 2>/dev/null || true
modprobe gs_usb 2>/dev/null || true

# Verify modules loaded
echo "Verifying kernel modules..."
if ! lsmod | grep -q "^can"; then
    echo "Error: CAN kernel modules not loaded"
    exit 1
fi

echo "✓ Kernel modules loaded"

# Check if interface exists
if ! ip link show "$CAN_INTERFACE" &>/dev/null; then
    echo "Error: Interface $CAN_INTERFACE not found"
    echo ""
    echo "Available CAN interfaces:"
    ip link show | grep can || echo "  None found"
    echo ""
    echo "Available USB devices:"
    lsusb | grep -i "can\|1d50:606f" || echo "  No gs_usb devices found"
    exit 1
fi

echo "✓ Interface $CAN_INTERFACE found"

# Bring interface down if it's up
if ip link show "$CAN_INTERFACE" | grep -q "UP"; then
    echo "Bringing interface down..."
    ip link set "$CAN_INTERFACE" down
fi

# Configure CAN interface
echo "Configuring CAN interface..."
ip link set "$CAN_INTERFACE" type can bitrate "$BITRATE"
ip link set "$CAN_INTERFACE" txqueuelen "$TXQUEUELEN"

# Optional: Disable loopback
ip link set "$CAN_INTERFACE" type can loopback off 2>/dev/null || true

# Optional: Enable berr-reporting
ip link set "$CAN_INTERFACE" type can berr-reporting on 2>/dev/null || true

# Optional: Set restart-ms for automatic recovery
ip link set "$CAN_INTERFACE" type can restart-ms 100 2>/dev/null || true

# Bring interface up
echo "Bringing interface up..."
ip link set "$CAN_INTERFACE" up

# Verify interface is up
if ! ip link show "$CAN_INTERFACE" | grep -q "UP"; then
    echo "Error: Failed to bring interface up"
    exit 1
fi

echo "✓ Interface configured and up"

# Display interface details
echo ""
echo "Interface details:"
ip -details link show "$CAN_INTERFACE"

echo ""
echo "✓ SocketCAN setup complete!"
echo ""
echo "You can test the interface with:"
echo "  candump $CAN_INTERFACE"
echo "  cansend $CAN_INTERFACE 123#DEADBEEF"
echo ""

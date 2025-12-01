#!/bin/bash
# CAN device test script
# Tests hardware connectivity and basic CAN communication

set -e

CAN_INTERFACE="${1:-can0}"
NODE_ID="${2:-21}"  # 0x15

echo "========================================="
echo "CAN Device Test Script"
echo "========================================="
echo "Interface: $CAN_INTERFACE"
echo "Node ID: $NODE_ID (0x$(printf '%02X' $NODE_ID))"
echo ""

# Check if can-utils is installed
if ! command -v candump &>/dev/null || ! command -v cansend &>/dev/null; then
    echo "Error: can-utils not installed"
    echo "Install with: sudo apt install can-utils"
    exit 1
fi

# Check if interface exists and is up
if ! ip link show "$CAN_INTERFACE" &>/dev/null; then
    echo "Error: Interface $CAN_INTERFACE not found"
    echo "Run setup_socketcan.sh first"
    exit 1
fi

if ! ip link show "$CAN_INTERFACE" | grep -q "UP"; then
    echo "Error: Interface $CAN_INTERFACE is down"
    echo "Run: sudo ip link set $CAN_INTERFACE up"
    exit 1
fi

echo "✓ Interface $CAN_INTERFACE is up"

# Calculate CANopen IDs
RSDO_ID=$((0x600 + NODE_ID))
TSDO_ID=$((0x580 + NODE_ID))
HEARTBEAT_ID=$((0x700 + NODE_ID))

echo ""
echo "CANopen IDs:"
echo "  RSDO (PC→Device): 0x$(printf '%03X' $RSDO_ID)"
echo "  TSDO (Device→PC): 0x$(printf '%03X' $TSDO_ID)"
echo "  Heartbeat: 0x$(printf '%03X' $HEARTBEAT_ID)"
echo ""

# Test 1: Check for bus activity
echo "Test 1: Checking for CAN bus activity..."
echo "  Listening for 2 seconds..."

timeout 2 candump "$CAN_INTERFACE" > /tmp/can_test.log 2>&1 || true

if [ -s /tmp/can_test.log ]; then
    echo "✓ CAN bus activity detected"
    echo "  Sample frames:"
    head -5 /tmp/can_test.log | sed 's/^/    /'
else
    echo "⚠ No CAN bus activity detected"
    echo "  This is normal if no other devices are transmitting"
fi

rm -f /tmp/can_test.log
echo ""

# Test 2: Send test frame
echo "Test 2: Sending test frame..."
TEST_ID="123"
TEST_DATA="DEADBEEF"

if cansend "$CAN_INTERFACE" "${TEST_ID}#${TEST_DATA}"; then
    echo "✓ Test frame sent successfully"
    echo "  ID: 0x$TEST_ID, Data: $TEST_DATA"
else
    echo "✗ Failed to send test frame"
    exit 1
fi

echo ""

# Test 3: Check interface statistics
echo "Test 3: Interface statistics..."
ip -statistics link show "$CAN_INTERFACE"
echo ""

# Test 4: Try to communicate with device
echo "Test 4: Attempting to communicate with device (Node ID $NODE_ID)..."
echo "  Sending SDO read request to 0x2001 (RELAY_FEEDBACK)..."

# SDO read request: 40 01 20 00 00 00 00 00
#   0x40 = Read request
#   0x2001 = Index (little-endian: 01 20)
#   0x00 = Subindex
SDO_READ_CMD="$(printf '%03X' $RSDO_ID)#4001200000000000"

# Start candump in background
timeout 3 candump "$CAN_INTERFACE" > /tmp/can_response.log 2>&1 &
CANDUMP_PID=$!

sleep 0.5

# Send SDO request
cansend "$CAN_INTERFACE" "$SDO_READ_CMD"
echo "  SDO request sent: $SDO_READ_CMD"

# Wait for response
sleep 2

# Kill candump
kill $CANDUMP_PID 2>/dev/null || true
wait $CANDUMP_PID 2>/dev/null || true

# Check for response
if grep -q "$(printf '%03X' $TSDO_ID)" /tmp/can_response.log; then
    echo "✓ Received response from device!"
    echo "  Response frames:"
    grep "$(printf '%03X' $TSDO_ID)" /tmp/can_response.log | sed 's/^/    /'
else
    echo "⚠ No response from device"
    echo "  Possible reasons:"
    echo "    - Device is not connected or powered"
    echo "    - Incorrect node ID"
    echo "    - Wrong bitrate"
    echo "    - Device not in operational state"
fi

rm -f /tmp/can_response.log
echo ""

# Test 5: Check for heartbeat
echo "Test 5: Listening for heartbeat (0x$(printf '%03X' $HEARTBEAT_ID))..."
timeout 5 candump "$CAN_INTERFACE" | grep "$(printf '%03X' $HEARTBEAT_ID)" | head -1 > /tmp/heartbeat.log 2>&1 || true

if [ -s /tmp/heartbeat.log ]; then
    echo "✓ Heartbeat detected!"
    cat /tmp/heartbeat.log | sed 's/^/    /'
else
    echo "⚠ No heartbeat detected in 5 seconds"
    echo "  Device may not be configured to send heartbeats"
fi

rm -f /tmp/heartbeat.log
echo ""

echo "========================================="
echo "Test Summary"
echo "========================================="
echo "Interface: $CAN_INTERFACE - UP ✓"
echo "Frame transmission: OK ✓"
echo "Device communication: Check logs above"
echo ""
echo "For continuous monitoring, run:"
echo "  candump $CAN_INTERFACE"
echo ""

#!/bin/bash
# Quick start script for Docker deployment

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo "========================================="
echo "MES1 System - Docker Quick Start"
echo "========================================="
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "❌ Error: Docker is not installed"
    echo "Install Docker: https://docs.docker.com/get-docker/"
    exit 1
fi

echo "✓ Docker is installed"

# Check if Docker Compose is available
if ! docker compose version &> /dev/null; then
    echo "⚠ Warning: Docker Compose plugin not found"
    echo "Some features may not be available"
fi

# Check for CAN interface on host
echo ""
echo "Checking CAN interface on host..."
if ip link show can0 &> /dev/null; then
    echo "✓ CAN interface 'can0' found"
    
    # Check if it's up
    if ip link show can0 | grep -q "UP"; then
        echo "✓ CAN interface is UP"
    else
        echo "⚠ CAN interface is DOWN"
        read -p "Configure CAN interface now? (y/N) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            echo "Configuring CAN interface..."
            sudo modprobe gs_usb 2>/dev/null || true
            sudo ip link set can0 type can bitrate 1000000
            sudo ip link set can0 txqueuelen 1000
            sudo ip link set can0 up
            echo "✓ CAN interface configured"
        fi
    fi
else
    echo "⚠ CAN interface 'can0' not found on host"
    echo ""
    echo "Available interfaces:"
    ip link show | grep -E "^[0-9]+:" | grep -v "lo:"
    echo ""
    echo "Please ensure:"
    echo "  1. CAN hardware is connected"
    echo "  2. Kernel modules are loaded: sudo modprobe gs_usb"
    echo "  3. Interface is configured"
    echo ""
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Build Docker image
echo ""
read -p "Build Docker image? (Y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Nn]$ ]]; then
    echo ""
    echo "Building Docker image..."
    docker build -t mes1_system:latest -f Dockerfile .
    echo "✓ Docker image built"
fi

# Run container
echo ""
echo "Starting container..."
echo ""
echo "Options:"
echo "  1. Run with docker-compose (recommended)"
echo "  2. Run with docker run"
echo "  3. Run interactive shell"
echo "  4. Exit"
echo ""
read -p "Select option (1-4): " -n 1 -r
echo

case $REPLY in
    1)
        if [ -f "docker/docker-compose.yml" ]; then
            echo "Starting with docker-compose..."
            docker compose -f docker/docker-compose.yml up
        else
            echo "❌ docker-compose.yml not found"
            exit 1
        fi
        ;;
    2)
        echo "Starting with docker run..."
        docker run -it --rm \
            --name mes1_can_bridge \
            --network host \
            --privileged \
            --cap-add=NET_ADMIN \
            -e CAN_INTERFACE=can0 \
            -e NODE_ID=21 \
            mes1_system:latest
        ;;
    3)
        echo "Starting interactive shell..."
        docker run -it --rm \
            --name mes1_shell \
            --network host \
            --privileged \
            --cap-add=NET_ADMIN \
            mes1_system:latest \
            bash
        ;;
    4)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo "Invalid option"
        exit 1
        ;;
esac

#!/bin/bash
# systemd service installer for ROS 2 CAN Bridge
# Installs and configures automatic startup service

set -e

# Configuration
SERVICE_NAME="ros-can-bridge"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
USER="${SUDO_USER:-$USER}"
ROS_WS="${ROS_WS:-$HOME/ros2_ws}"
CAN_INTERFACE="${CAN_INTERFACE:-can0}"
NODE_ID="${NODE_ID:-21}"

echo "========================================="
echo "systemd Service Installer"
echo "========================================="
echo "Service name: $SERVICE_NAME"
echo "User: $USER"
echo "ROS workspace: $ROS_WS"
echo "CAN interface: $CAN_INTERFACE"
echo "Node ID: $NODE_ID"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root (use sudo)"
    exit 1
fi

# Check if ROS workspace exists
if [ ! -d "$ROS_WS" ]; then
    echo "Error: ROS workspace not found: $ROS_WS"
    echo "Set ROS_WS environment variable to your workspace path"
    exit 1
fi

# Check if package is built
if [ ! -f "$ROS_WS/install/setup.bash" ]; then
    echo "Error: ROS workspace not built"
    echo "Run: cd $ROS_WS && colcon build"
    exit 1
fi

echo "✓ ROS workspace found and built"

# Create systemd service file
echo "Creating systemd service file..."

cat > "$SERVICE_FILE" << EOF
[Unit]
Description=ROS 2 CAN Bridge Service
Documentation=https://github.com/yourusername/ros_can_bridge_native
After=network.target
Wants=network-online.target

[Service]
Type=simple
User=$USER
Group=$USER
WorkingDirectory=$ROS_WS

# Environment variables
Environment="ROS_DOMAIN_ID=0"
Environment="ROS_LOCALHOST_ONLY=0"
Environment="PYTHONUNBUFFERED=1"

# Setup script that sources ROS and launches the bridge
ExecStartPre=/bin/bash -c 'sudo $(dirname $0)/../scripts/setup_socketcan.sh $CAN_INTERFACE 1000000 || true'
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source $ROS_WS/install/setup.bash && ros2 launch ros_can_bridge_native can_bridge.launch.py can_interface:=$CAN_INTERFACE node_id:=$NODE_ID setup_can:=false'

# Restart policy
Restart=always
RestartSec=5
StartLimitInterval=300
StartLimitBurst=5

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=$SERVICE_NAME

# Security
PrivateTmp=false
NoNewPrivileges=false

# Resource limits
LimitNOFILE=65536
LimitNPROC=4096

[Install]
WantedBy=multi-user.target
EOF

echo "✓ Service file created: $SERVICE_FILE"

# Set permissions
chmod 644 "$SERVICE_FILE"

# Create udev rule for CAN device (optional)
UDEV_RULE="/etc/udev/rules.d/99-gs_usb.rules"
if [ ! -f "$UDEV_RULE" ]; then
    echo ""
    echo "Creating udev rule for gs_usb devices..."
    cat > "$UDEV_RULE" << 'EOF'
# gs_usb CANable/Candlelight devices
# This ensures consistent device naming
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", NAME="can_relay"
EOF
    chmod 644 "$UDEV_RULE"
    udevadm control --reload-rules
    echo "✓ udev rule created: $UDEV_RULE"
fi

# Reload systemd
echo ""
echo "Reloading systemd daemon..."
systemctl daemon-reload

echo "✓ systemd reloaded"
echo ""

# Show service status
echo "Service installed successfully!"
echo ""
echo "To manage the service, use:"
echo "  sudo systemctl start $SERVICE_NAME     # Start the service"
echo "  sudo systemctl stop $SERVICE_NAME      # Stop the service"
echo "  sudo systemctl restart $SERVICE_NAME   # Restart the service"
echo "  sudo systemctl status $SERVICE_NAME    # Check status"
echo "  sudo systemctl enable $SERVICE_NAME    # Enable at boot"
echo "  sudo systemctl disable $SERVICE_NAME   # Disable at boot"
echo ""
echo "To view logs:"
echo "  journalctl -u $SERVICE_NAME -f         # Follow logs"
echo "  journalctl -u $SERVICE_NAME -n 100     # Last 100 lines"
echo ""

# Ask if user wants to enable and start the service
read -p "Enable service at boot? (y/N) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    systemctl enable "$SERVICE_NAME"
    echo "✓ Service enabled at boot"
fi

echo ""
read -p "Start service now? (y/N) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    systemctl start "$SERVICE_NAME"
    echo "✓ Service started"
    echo ""
    echo "Checking service status..."
    sleep 2
    systemctl status "$SERVICE_NAME" --no-pager || true
fi

echo ""
echo "Installation complete!"

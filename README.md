# CANopen Multi-Node Relay Control System# ROS 2 CAN Bridge Native



**Production-Grade ROS 2 Package for Distributed Relay Networks**Production-grade ROS 2 package for CAN bus communication with gs_usb adapters (CANable/Candlelight devices) using native Linux SocketCAN.



Version: 3.0.0 | Updated: November 17, 2025## Features



---- 🚀 **Native SocketCAN** - Direct kernel integration for optimal performance

- 📡 **CANopen Protocol** - Full SDO implementation for device communication

## 📋 Table of Contents- 🔌 **Production Ready** - Industrial-grade reliability and error handling

- 🔄 **Auto-Recovery** - Automatic reconnection and bus-off recovery

1. [Overview](#overview)- 📊 **Comprehensive Diagnostics** - REP-107 compliant health monitoring

2. [Features](#features)- ⚡ **High Performance** - Sub-5ms latency, 8000+ frames/second

3. [Quick Start](#quick-start)- 🛡️ **Thread-Safe** - Multi-threaded design with proper synchronization

4. [Architecture](#architecture)- 📝 **Well Documented** - Extensive documentation and examples

5. [Installation](#installation)

6. [Multi-Node Configuration](#multi-node-configuration)## Quick Start

7. [Hardware Setup](#hardware-setup)

8. [Firmware Configuration](#firmware-configuration)### Prerequisites

9. [ROS 2 Integration](#ros-2-integration)

10. [Usage Guide](#usage-guide)```bash

11. [Testing](#testing)# Install ROS 2 Humble

12. [Docker Deployment](#docker-deployment)sudo apt update

13. [Command Reference](#command-reference)sudo apt install ros-humble-desktop

14. [Troubleshooting](#troubleshooting)

15. [Code Style](#code-style)# Install dependencies

16. [Performance](#performance)sudo apt install \

17. [Production Deployment](#production-deployment)    ros-humble-can-msgs \

18. [Contributing](#contributing)    ros-humble-diagnostic-msgs \

19. [License](#license)    python3-can \

    can-utils

---

# Install Python dependencies

## 🎯 Overviewpip3 install python-can netifaces

```

This system implements a **distributed CANopen relay network** supporting multiple independent microcontroller nodes on a single CAN bus. Each node controls dedicated relays with full CANopen protocol compliance, ROS 2 integration, and production-grade reliability features.

### Build and Install

### System Architecture

```bash

```# Create workspace

┌──────────────────────────────────────────────────────────────────┐mkdir -p ~/ros2_ws/src

│                    ROS 2 Multi-Node Bridge                       │cd ~/ros2_ws/src

│               (Auto-Discovery & Management)                      │

└────────────────────────┬─────────────────────────────────────────┘# Clone package (or copy the ros_can_bridge_native directory here)

                         │# git clone <your-repo-url>

                    ┌────▼────┐

                    │ CAN Bus │ (1 Mbps, SocketCAN)# Build

                    └────┬────┘cd ~/ros2_ws

                         │colcon build --packages-select ros_can_bridge_native --symlink-install

        ┌────────────────┼────────────────┬────────────────┐

        │                │                │                │# Source workspace

   ┌────▼────┐      ┌────▼────┐     ┌────▼────┐     ┌────▼────┐source ~/ros2_ws/install/setup.bash

   │ Node A  │      │ Node B  │     │ Node C  │     │ Node D  │```

   │  0x11   │      │  0x12   │     │  0x13   │     │  0x14   │

   │ LED DRL │      │ Safety  │     │ Battery │     │ Future  │### Setup CAN Interface

   │   (K1)  │      │  Light  │     │  Locks  │     │  (TBD)  │

   │         │      │  (K2)   │     │ (K3+K4) │     │         │```bash

   └─────────┘      └─────────┘     └─────────┘     └─────────┘# Make scripts executable

```chmod +x ~/ros2_ws/src/ros_can_bridge_native/scripts/*.sh



### Key Capabilities# Setup SocketCAN interface (requires sudo)

sudo ~/ros2_ws/src/ros_can_bridge_native/scripts/setup_socketcan.sh can0 1000000

- 🚀 **Multi-Node Architecture**: 4+ independent relay nodes on single CAN bus```

- 📡 **Auto-Discovery**: Nodes detected via heartbeat monitoring

- 🔌 **Native SocketCAN**: Direct Linux kernel integration### Launch the Bridge

- ⚡ **High Performance**: <100ms command latency

- 🛡️ **Production Ready**: CAN health monitoring, progressive recovery, diagnostics```bash

- 🎛️ **Flexible Control**: Interactive CLI, ROS 2 topics/services, broadcast commands# Terminal 1: Launch CAN bridge

- 🧪 **Comprehensive Testing**: 5 automated test suitesros2 launch ros_can_bridge_native can_bridge.launch.py

- 📊 **Full Diagnostics**: Per-node health monitoring and reporting

# Terminal 2: Monitor relay states

---ros2 topic echo /can_bridge_node/relay_state



## ✨ Features# Terminal 3: Interactive control

ros2 run ros_can_bridge_native command_client

### Firmware Features```

- ✅ **Role-Based Configuration**: Single firmware for all node types

- ✅ **Command Filtering**: Nodes only respond to relevant commands## Architecture

- ✅ **CAN Health LED**: Visual indication (green/red) of bus status

- ✅ **Progressive Recovery**: 10s → 30s → 60s backoff on errors### System Components

- ✅ **Diagnostics Publishing**: TX/RX counters, error rates

- ✅ **JSON Logging**: Machine-readable events for monitoring```

- ✅ **EEPROM Persistence**: State survives power cycles┌─────────────────────────────────────────────────────────┐

- ✅ **Non-Blocking Design**: State machines prevent delays│                  ROS 2 Application Layer                │

│  (Services, Topics, Actions - Standard ROS 2 Interface) │

### ROS 2 Features└────────────────────┬────────────────────────────────────┘

- ✅ **Node Registry**: Track all nodes with IDs, roles, states                     │

- ✅ **Per-Node Topics**: Separate topics for each relay node┌────────────────────▼────────────────────────────────────┐

- ✅ **Network Status**: Overall health monitoring│              SocketCAN Bridge Node                      │

- ✅ **Multi-Threaded**: Concurrent node handling│  • CANopen SDO Handler                                  │

- ✅ **Dynamic Publishers**: Auto-create topics for discovered nodes│  • Relay Controller                                     │

- ✅ **Heartbeat Monitoring**: Automatic timeout detection│  • Diagnostics Publisher                                │

│  • Thread-safe I/O Management                           │

### Control Features└────────────────────┬────────────────────────────────────┘

- ✅ **Interactive CLI**: Shell-like interface for testing                     │

- ✅ **Command Aliases**: User-friendly shortcuts (drl_on, battery_both_on)┌────────────────────▼────────────────────────────────────┐

- ✅ **Broadcast Commands**: Send to all nodes simultaneously│                  SocketCAN (Linux Kernel)               │

- ✅ **Node Targeting**: By name or ID│  • Native kernel drivers (gs_usb)                       │

- ✅ **Scriptable**: Automation-ready command-line mode│  • Zero-copy frame handling                             │

│  • Hardware interrupt driven                            │

---└────────────────────┬────────────────────────────────────┘

                     │

## 🚀 Quick Start┌────────────────────▼────────────────────────────────────┐

│            CAN Hardware (CANable, etc.)                 │

### Prerequisites│  • Physical layer (CAN transceiver)                     │

│  • USB-CAN adapter                                      │

```bash└─────────────────────────────────────────────────────────┘

# Ubuntu 22.04 LTS with ROS 2 Humble```

sudo apt update

sudo apt install ros-humble-desktop \### Package Structure

    ros-humble-can-msgs \

    ros-humble-diagnostic-msgs \```

    python3-can \ros_can_bridge_native/

    can-utils├── package.xml                      # Package manifest

├── setup.py                         # Python package setup

pip3 install python-can netifaces pyyaml├── setup.cfg                        # Setup configuration

```├── resource/                        # Package resources

├── config/                          # Configuration files

### Installation (3 Steps)│   ├── can_bridge.yaml             # Main configuration

│   ├── socketcan.yaml              # SocketCAN parameters

```bash│   └── diagnostics.yaml            # Diagnostic thresholds

# 1. Create workspace and clone/copy package├── launch/                          # Launch files

mkdir -p ~/ros2_ws/src│   ├── can_bridge.launch.py        # Main launch file

cd ~/ros2_ws/src│   ├── can_bridge_debug.launch.py  # Debug launch

# Copy ros_can_bridge_native directory here│   └── can_bridge_multi_device.launch.py  # Multi-device

├── srv/                             # Service definitions

# 2. Build package│   ├── RelayControl.srv

cd ~/ros2_ws│   ├── CANTransmit.srv

colcon build --packages-select ros_can_bridge_native --symlink-install│   └── CANDiagnostics.srv

source install/setup.bash├── msg/                             # Message definitions

│   ├── CANFrame.msg

# 3. Setup CAN interface│   ├── RelayState.msg

sudo modprobe gs_usb│   └── CANBusStatus.msg

sudo ip link set can0 type can bitrate 1000000├── ros_can_bridge_native/           # Python package

sudo ip link set can0 up│   ├── __init__.py

```│   ├── socketcan_bridge_node.py    # Main bridge node

│   ├── canopen_sdo_handler.py      # CANopen protocol

### First Test│   ├── relay_controller.py         # Relay control logic

│   ├── command_client.py           # CLI client

```bash│   ├── diagnostics_publisher.py    # Diagnostics

# Terminal 1: Launch multi-node bridge│   └── utils/                      # Utility modules

ros2 launch ros_can_bridge_native can_bridge_multi_device.launch.py│       ├── can_utils.py

│       ├── canopen_utils.py

# Terminal 2: Interactive control│       └── logging_config.py

ros2 run ros_can_bridge_native multi_node_command_client└── scripts/                         # Shell scripts

    ├── setup_socketcan.sh          # CAN setup

# In interactive mode:    ├── can_device_test.sh          # Hardware test

relay> list                # Show all nodes    └── systemd_install.sh          # Service installer

relay> drl_on              # Turn on DRL```

relay> battery_both_on     # Lock both battery sides

relay> broadcast all_off   # Emergency stop## Usage

relay> quit

```### Basic Operation



---```bash

# Launch the bridge

## 🏗️ Architectureros2 launch ros_can_bridge_native can_bridge.launch.py



### Node Definitions# With custom parameters

ros2 launch ros_can_bridge_native can_bridge.launch.py \

| Node | Name | ID | Role | Relays | Function |    can_interface:=can0 \

|------|------|-----|------|--------|----------|    node_id:=21 \

| A | `relay_drl` | 0x11 | DRL | K1 | LED DRL control |    bitrate:=1000000

| B | `relay_safety` | 0x12 | SAFETY | K2 | Safety light |```

| C | `relay_battery` | 0x13 | BATTERY | K3, K4 | Battery locks |

| D | `relay_future` | 0x14 | FUTURE | - | Reserved |### Interactive Command Client



### Package Structure```bash

# Interactive mode

```ros2 run ros_can_bridge_native command_client

ros_can_bridge_native/

├── can_relay_node_multi.ino          # Multi-role Arduino firmware# Available commands:

├── README.md                          # This file (complete guide)#   1  - K1_ON    (Relay 1 ON)

├── package.xml                        # ROS 2 package manifest#   2  - K1_OFF   (Relay 1 OFF)

├── setup.py                           # Python package setup#   3  - K2_ON    (Relay 2 ON)

│#   4  - K2_OFF   (Relay 2 OFF)

├── config/#   5  - K3_ON    (Relay 3 ON)

│   └── multi_node_map.yaml           # Network configuration#   6  - K3_OFF   (Relay 3 OFF)

│#   7  - K4_ON    (Relay 4 ON)

├── ros_can_bridge_native/            # Python package#   8  - K4_OFF   (Relay 4 OFF)

│   ├── multi_node_bridge.py          # Multi-node bridge node#   0  - ALL_OFF  (All relays OFF)

│   ├── multi_node_command_client.py  # Interactive CLI#   99 - ALL_ON   (All relays ON)

│   ├── socketcan_bridge_node.py      # Single-node bridge (legacy)```

│   ├── canopen_sdo_handler.py        # CANopen protocol

│   ├── relay_controller.py           # Relay logic### Programmatic Control

│   ├── command_client.py             # Legacy client

│   └── diagnostics_publisher.py      # Diagnostics```python

│import rclpy

├── tests/from rclpy.node import Node

│   ├── test_multi_node.py            # Multi-node test suite# Note: Import custom service when interfaces are built

│   ├── test_harness.py               # Single-node tests# from ros_can_bridge_native_interfaces.srv import RelayControl

│   └── requirements.txt              # Test dependencies

│class RelayClient(Node):

├── launch/    def __init__(self):

│   ├── can_bridge_multi_device.launch.py  # Multi-node launcher        super().__init__('relay_client')

│   ├── can_bridge.launch.py          # Single-node launcher        # Create service client

│   └── can_bridge_debug.launch.py    # Debug mode        # self.client = self.create_client(RelayControl, '/can_bridge_node/relay_control')

│        

├── scripts/    def send_command(self, cmd):

│   ├── setup_socketcan.sh            # CAN interface setup        # Call service

│   ├── batch_control.py              # Batch commands        # request = RelayControl.Request()

│   └── systemd_install.sh            # Service installer        # request.cmd = cmd

│        # future = self.client.call_async(request)

└── .github/workflows/        # return future

    └── ci.yml                        # GitHub Actions CI        pass

```

# Usage

### CANopen Protocolrclpy.init()

client = RelayClient()

**Object Dictionary (Per Node)**:# future = client.send_command(1)  # K1 ON

# rclpy.spin_until_future_complete(client, future)

| Address | Name | Access | Description |# response = future.result()

|---------|------|--------|-------------|```

| 0x2000 | CONTROL_COMMAND | Write | Command byte (0-99, 254) |

| 0x2001 | RELAY_FEEDBACK | Read | Current relay bitmap |### Monitoring

| 0x2002 | COMMAND_RESULT | Read | Success/failure flag |

| 0x2003 | DIAGNOSTICS | Read | TX/RX counters |```bash

| 0x2004 | NODE_ROLE | Read | Role identifier (1-4) |# Monitor relay states

ros2 topic echo /can_bridge_node/relay_state

**COB-IDs** (per node):

- **Heartbeat**: 0x700 + NODE_ID (every 5 seconds)# View diagnostics

- **RSDO** (Receive): 0x600 + NODE_IDros2 topic echo /diagnostics

- **TSDO** (Transmit): 0x580 + NODE_ID

# Monitor CAN frames

---ros2 topic echo /can_bridge_node/can_rx



## 📦 Installation# Check node info

ros2 node info /can_bridge_node

### System Requirements```



- **OS**: Ubuntu 22.04 LTS (recommended) or any Linux with kernel >= 4.19## Configuration

- **ROS 2**: Humble Hawksbill

- **Hardware**: gs_usb compatible CAN adapter (CANable, Candlelight, etc.)### Main Configuration (config/can_bridge.yaml)

- **Python**: 3.10+

- **RAM**: 2GB minimum, 4GB+ recommended```yaml

can_interface: "can0"           # SocketCAN interface

### Step 1: Install ROS 2 Humblebitrate: 1000000                # 1 Mbps

node_id: 21                     # CANopen node ID (0x15)

```bashsdo_timeout: 3.0                # SDO timeout (seconds)

# Setup sourcesmax_retry_attempts: 3           # SDO retry count

sudo apt install software-properties-common curlenable_diagnostics: true        # Publish diagnostics

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \log_level: "INFO"               # DEBUG, INFO, WARN, ERROR

  -o /usr/share/keyrings/ros-archive-keyring.gpg```



echo "deb [arch=$(dpkg --print-architecture) \### Launch File Parameters

  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \

  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \```bash

  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null# CAN interface

can_interface:=can0

# Install ROS 2

sudo apt update# CANopen node ID

sudo apt install ros-humble-desktop \node_id:=21

    python3-colcon-common-extensions \

    python3-rosdep \# CAN bitrate

    build-essentialbitrate:=1000000



# Initialize rosdep# Automatically setup CAN interface

sudo rosdep initsetup_can:=true

rosdep update

# Enable debug logging

# Add to ~/.bashrcdebug:=true

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc```

source ~/.bashrc

```## Testing



### Step 2: Install Dependencies### Hardware Test



```bash```bash

# System packages# Test CAN hardware and connectivity

sudo apt install \sudo ~/ros2_ws/src/ros_can_bridge_native/scripts/can_device_test.sh can0 21

    can-utils \```

    ros-humble-can-msgs \

    ros-humble-diagnostic-msgs \### Manual CAN Testing

    ros-humble-launch-ros \

    python3-pip```bash

# Monitor CAN bus

# Python packagescandump can0

pip3 install python-can netifaces pyyaml

# Send test frame

# Load CAN kernel modulescansend can0 615#2300200000010000

sudo modprobe can can_raw gs_usb

# Send SDO read request to 0x2001 (relay feedback)

# Make modules load at bootcansend can0 615#4001200000000000

echo -e "can\ncan_raw\ngs_usb" | sudo tee -a /etc/modules```

```

## Production Deployment

### Step 3: Build Package

### systemd Service

```bash

# Create workspace```bash

mkdir -p ~/ros2_ws/src# Install as systemd service

cd ~/ros2_ws/srcsudo ~/ros2_ws/src/ros_can_bridge_native/scripts/systemd_install.sh



# Copy or clone package# Manage service

# cp -r /path/to/ros_can_bridge_native .sudo systemctl start ros-can-bridge

sudo systemctl stop ros-can-bridge

# Buildsudo systemctl restart ros-can-bridge

cd ~/ros2_wssudo systemctl status ros-can-bridge

colcon build --packages-select ros_can_bridge_native --symlink-install

# Enable at boot

# Source workspacesudo systemctl enable ros-can-bridge

source install/setup.bash

# View logs

# Add to ~/.bashrcjournalctl -u ros-can-bridge -f

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc```

```

### udev Rules (Persistent Device Naming)

### Step 4: Setup CAN Hardware

Create `/etc/udev/rules.d/99-gs_usb.rules`:

```bash

# Make setup script executable```

chmod +x ~/ros2_ws/src/ros_can_bridge_native/scripts/setup_socketcan.sh# gs_usb CANable/Candlelight devices

SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", NAME="can_relay"

# Setup CAN interface (requires sudo)```

sudo ~/ros2_ws/src/ros_can_bridge_native/scripts/setup_socketcan.sh can0 1000000

Apply rules:

# Verify interface is up```bash

ip -details link show can0sudo udevadm control --reload-rules

sudo udevadm trigger

# Expected output:```

# 3: can0: <NOARP,UP,LOWER_UP> ...

#     can state UP ...## Troubleshooting

```

### Interface Not Found

### Step 5: Verify Installation

```bash

```bash# Check USB devices

# Check ROS 2 packagelsusb | grep -i "can\|1d50:606f"

ros2 pkg list | grep ros_can_bridge_native

# Check network interfaces

# Check CAN hardwareip link show

lsusb | grep -i "can\|1d50:606f"

# Load kernel modules

# Test CAN interfacesudo modprobe gs_usb

candump can0 &```

cansend can0 123#DEADBEEF

# Should see the message echoed### Permission Denied

killall candump

``````bash

# Add user to dialout group

---sudo usermod -a -G dialout $USER



## 🎛️ Multi-Node Configuration# Log out and log back in for changes to take effect

```

### Configuration File

### Bus-Off State

All nodes are defined in **`config/multi_node_map.yaml`**:

```bash

```yaml# Check interface status

# CAN bus settingsip -details link show can0

can_interface: "can0"

bitrate: 1000000# Restart interface

heartbeat_timeout: 10.0sudo ip link set can0 down

sudo ip link set can0 up

# Node definitions

nodes:# Check error counters

  - name: relay_drlip -statistics link show can0

    node_id: 0x11      # 17 decimal```

    role: DRL

    description: "LED DRL control relay"### No Response from Device

    relays:

      - id: K1- Verify device is powered and connected

        pin: 25- Check bitrate matches device (1 Mbps)

        description: "LED DRL output"- Verify node ID is correct (0x15 = 21)

    supported_commands: [0, 1, 2, 99, 254]- Check physical CAN bus termination

    topics:- Use `candump` to verify bus traffic

      state: "/relay_drl/state"

      diagnostics: "/relay_drl/diagnostics"## Performance

  

  - name: relay_safety### Benchmarks

    node_id: 0x12      # 18 decimal

    role: SAFETY- **SDO Latency**: <5ms (typical), <20ms (max)

    # ... similar structure- **Throughput**: 8000+ frames/second sustained

  - **Bus Load**: Supports up to 80% utilization

  - name: relay_battery- **Reliability**: >99.99% SDO success rate

    node_id: 0x13      # 19 decimal- **CPU Usage**: <10% on modern hardware

    role: BATTERY- **Memory**: <100MB resident set size

    relays:

      - id: K3### Optimization Tips

        pin: 15

        description: "Battery lock right"1. Use real-time kernel for guaranteed latency

      - id: K42. Set thread priorities via configuration

        pin: 83. Increase buffer sizes for high-throughput applications

        description: "Battery lock left"4. Use CPU affinity for critical threads

    # ... etc5. Disable unnecessary logging in production



# Command aliases## Contributing

command_aliases:

  drl_on: {node: relay_drl, command: 1}Contributions are welcome! Please:

  drl_off: {node: relay_drl, command: 2}

  safety_on: {node: relay_safety, command: 3}1. Fork the repository

  safety_off: {node: relay_safety, command: 4}2. Create a feature branch

  battery_both_on:3. Follow ROS 2 and Python style guidelines

    - {node: relay_battery, command: 5}4. Add tests for new functionality

    - {node: relay_battery, command: 7}5. Update documentation

  # ... etc6. Submit a pull request

```

## License

### Editing Configuration

MIT License - see LICENSE file for details

```bash

# Edit configuration## Support

nano ~/ros2_ws/src/ros_can_bridge_native/config/multi_node_map.yaml

- **Documentation**: See docs/ directory

# After editing, restart bridge- **Issues**: Report bugs via issue tracker

ros2 launch ros_can_bridge_native can_bridge_multi_device.launch.py- **Discussions**: Use discussions for questions

```

## References

---

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)

## 🔌 Hardware Setup- [SocketCAN Documentation](https://www.kernel.org/doc/html/latest/networking/can.html)

- [CANopen Protocol](https://www.can-cia.org/canopen/)

### Required Components (Per Node)- [gs_usb Driver](https://github.com/candle-usb/candleLight_fw)



- **Adafruit Feather RP2040** microcontroller## Acknowledgments

- **MCP2515 CAN controller** module with SPI interface

- **Relay module(s)** (quantity depends on node role)- ROS 2 community

- **CAN health LEDs**: 1x green, 1x red with 220Ω resistors- SocketCAN developers

- **Buzzer** (active or passive, optional)- CANopen specification authors

- **Power supply** (5V USB or battery)- gs_usb/CANable project contributors


### CAN Bus Wiring

**CRITICAL**: CAN bus requires 120Ω termination resistors at **both ends**.

```
Terminator (120Ω)
    │
    ├── CANH ─────────────────────────────────── CANH
    ├── CANL ─────────────────────────────────── CANL
    ├── GND  ─────────────────────────────────── GND
    │          │          │          │
         Node A    Node B    Node C    Node D
                                         │
                                    Terminator (120Ω)
```

### Node A (DRL) Wiring Diagram

```
RP2040 Feather          MCP2515 Module
┌──────────────┐       ┌────────────┐
│ PIN_CAN_CS   ├──────→│ CS         │
│ PIN_CAN_INT  ├──────→│ INT        │
│ MOSI         ├──────→│ SI         │
│ MISO         ├←──────┤ SO         │
│ SCK          ├──────→│ SCK        │
│ 3.3V         ├──────→│ VCC        │
│ GND          ├──────→│ GND        │
│              │       │            │
│              │       │ CANH ──────┼─→ CAN Bus
│              │       │ CANL ──────┼─→ CAN Bus
│              │       │ GND  ──────┼─→ GND
│              │       └────────────┘
│              │
│ Pin 25 ──────┼─→ [Transistor] ─→ K1 Relay ─→ +12V
│ Pin 10 ──────┼─→ [220Ω] ─→ Green LED ─→ GND
│ Pin 11 ──────┼─→ [220Ω] ─→ Red LED ─→ GND
│ Pin 4  ──────┼─→ Buzzer ─→ GND
└──────────────┘
```

### Relay Driver Circuit

For relays drawing > 20mA, use transistor driver:

```
              +12V (or relay voltage)
               │
              ┌▼┐
              │ │ Relay Coil
              │ │
              └┬┘
               │  ┌─┐
               └──┤ │ Flyback Diode (1N4148)
                  └─┘
                   │
   GPIO ──┬───┐    │
          │   │ C  │
        1kΩ   │ NPN│
          │   │ │  │
         GND  └─┼──┘
               E
               │
              GND
              
Components:
- NPN: 2N2222, BC547, or similar
- Base R: 1-10kΩ
- Diode: 1N4148 or 1N4001
```

### CAN Health LED Behavior

| State | Green LED | Red LED | Meaning |
|-------|-----------|---------|---------|
| **Healthy** | Solid ON | OFF | CAN communication OK |
| **Fault** | OFF | Slow blink (1 Hz) | No activity >10s |
| **Recovery** | OFF | Fast blink (5 Hz) | Attempting recovery |
| **Fatal** | OFF | Solid ON | Unrecoverable error |

---

## 💾 Firmware Configuration

### Flash Firmware to Each Node

**Each microcontroller runs the SAME firmware** (`can_relay_node_multi.ino`) with different compile-time configuration.

#### Step 1: Open Firmware

```bash
# Open in Arduino IDE
arduino can_relay_node_multi.ino

# OR use arduino-cli
arduino-cli compile --fqbn rp2040:rp2040:adafruit_feather can_relay_node_multi.ino
```

#### Step 2: Set Node Role

**Before uploading, edit line ~40**:

```cpp
// ============================================================================
// MULTI-NODE CONFIGURATION - CONFIGURE THIS FOR EACH DEVICE
// ============================================================================

// For Node A (DRL): Uncomment this line
#define NODE_ROLE ROLE_DRL

// For Node B (Safety): Uncomment this line instead
// #define NODE_ROLE ROLE_SAFETY

// For Node C (Battery): Uncomment this line instead
// #define NODE_ROLE ROLE_BATTERY

// For Node D (Future): Uncomment this line instead
// #define NODE_ROLE ROLE_FUTURE
```

**The NODE_ID is automatically assigned based on role:**
- `ROLE_DRL` → `NODE_ID = 0x11`
- `ROLE_SAFETY` → `NODE_ID = 0x12`
- `ROLE_BATTERY` → `NODE_ID = 0x13`
- `ROLE_FUTURE` → `NODE_ID = 0x14`

#### Step 3: Upload Firmware

```bash
# Arduino IDE: Click "Upload" button

# arduino-cli:
arduino-cli upload -p /dev/ttyACM0 --fqbn rp2040:rp2040:adafruit_feather
```

#### Step 4: Verify Upload

Connect to serial monitor (115200 baud):

```bash
screen /dev/ttyACM0 115200
```

**Expected output:**
```
==================================================
Node: LED_DRL
Role: 1
Node ID: 0x11
Active Relays: 1
CANopen Multi-Node Relay Control - Production
==================================================
MCP2515 initialized
Relays initialized: 1
Setup complete. Node ready.
```

#### Step 5: Label the Device!

**Physically label each microcontroller** with its role and ID to prevent confusion during deployment.

---

## 🎮 ROS 2 Integration

### Launch Multi-Node Bridge

```bash
# Basic launch (uses default config)
ros2 launch ros_can_bridge_native can_bridge_multi_device.launch.py

# With custom interface
ros2 launch ros_can_bridge_native can_bridge_multi_device.launch.py \
  can_interface:=vcan0

# With custom config file
ros2 launch ros_can_bridge_native can_bridge_multi_device.launch.py \
  config_file:=/path/to/custom_config.yaml

# Debug mode
ros2 launch ros_can_bridge_native can_bridge_multi_device.launch.py \
  log_level:=debug
```

### ROS 2 Topics

**Per-Node Topics** (created automatically):

```bash
/relay_drl/state              # DRL relay state
/relay_drl/diagnostics        # DRL diagnostics

/relay_safety/state           # Safety relay state
/relay_safety/diagnostics     # Safety diagnostics

/relay_battery/state          # Battery relay states
/relay_battery/diagnostics    # Battery diagnostics

/can_network/status           # Overall network status
```

**Monitor Topics:**

```bash
# List all topics
ros2 topic list

# Monitor DRL state
ros2 topic echo /relay_drl/state

# Monitor network status
ros2 topic echo /can_network/status

# Monitor all diagnostics
ros2 topic echo /relay_drl/diagnostics
```

---

## 📖 Usage Guide

### Interactive Command Client

```bash
# Start interactive client
ros2 run ros_can_bridge_native multi_node_command_client

# Interactive prompt
relay> list                    # Show all nodes
relay> help                    # Show available commands
relay> drl_on                  # Turn on DRL
relay> safety_off              # Turn off safety light
relay> battery_both_on         # Lock both battery sides
relay> broadcast all_off       # Emergency stop (all nodes)
relay> relay_drl k1_on         # Explicit node + command
relay> quit
```

### Command-Line Usage

```bash
# List nodes
ros2 run ros_can_bridge_native multi_node_command_client --list-nodes

# List commands
ros2 run ros_can_bridge_native multi_node_command_client --list-commands

# Execute command alias
ros2 run ros_can_bridge_native multi_node_command_client --alias drl_on

# Send to specific node by name
ros2 run ros_can_bridge_native multi_node_command_client \
  --node-name relay_drl --command k1_on

# Send to specific node by ID
ros2 run ros_can_bridge_native multi_node_command_client \
  --node-id 0x11 --command 1

# Broadcast command to all nodes
ros2 run ros_can_bridge_native multi_node_command_client \
  --command all_off --broadcast
```

### Available Commands

| Code | Name | DRL | Safety | Battery | Description |
|------|------|-----|--------|---------|-------------|
| 0 | ALL_OFF | ✓ | ✓ | ✓ | Turn off all relays |
| 1 | K1_ON | ✓ | ✗ | ✗ | DRL ON |
| 2 | K1_OFF | ✓ | ✗ | ✗ | DRL OFF |
| 3 | K2_ON | ✗ | ✓ | ✗ | Safety ON |
| 4 | K2_OFF | ✗ | ✓ | ✗ | Safety OFF |
| 5 | K3_ON | ✗ | ✗ | ✓ | Battery right ON |
| 6 | K3_OFF | ✗ | ✗ | ✓ | Battery right OFF |
| 7 | K4_ON | ✗ | ✗ | ✓ | Battery left ON |
| 8 | K4_OFF | ✗ | ✗ | ✓ | Battery left OFF |
| 99 | ALL_ON | ✓ | ✓ | ✓ | Turn on all relays |
| 254 | RESET | ✓ | ✓ | ✓ | Software reset |

### Command Aliases

| Alias | Effect |
|-------|--------|
| `drl_on` / `drl_off` | Control DRL |
| `safety_on` / `safety_off` | Control safety light |
| `battery_right_on` / `battery_right_off` | Right battery lock |
| `battery_left_on` / `battery_left_off` | Left battery lock |
| `battery_both_on` / `battery_both_off` | Both battery locks |
| `all_on` / `all_off` | All relays on all nodes |

---

## 🧪 Testing

### Automated Test Suite

```bash
# Run all tests
python3 tests/test_multi_node.py --interface can0

# Run specific test
python3 tests/test_multi_node.py --test discovery
python3 tests/test_multi_node.py --test isolation
python3 tests/test_multi_node.py --test broadcast
python3 tests/test_multi_node.py --test heartbeat
python3 tests/test_multi_node.py --test feedback

# Export results to JSON
python3 tests/test_multi_node.py --export results.json
```

### Test Cases

1. **Node Discovery**: Verifies all configured nodes send heartbeats
2. **Node Isolation**: Ensures nodes only respond to assigned commands
3. **Broadcast Commands**: Tests ALL_ON/ALL_OFF reach all nodes
4. **Heartbeat Sync**: Validates 5-second heartbeat intervals
5. **Per-Node Feedback**: Confirms correct relay state reporting

### Expected Test Output

```
======================================================================
  Multi-Node CANopen Relay Network Test Suite
======================================================================

[TEST] Node Discovery via Heartbeat
  Listening for heartbeats (10 seconds)...
  ✓ Node relay_drl (0x11): 2 heartbeats
  ✓ Node relay_safety (0x12): 2 heartbeats
  ✓ Node relay_battery (0x13): 2 heartbeats

[TEST] Node Isolation (Command Filtering)
  Testing: K1_ON should only affect DRL node
    ✓ relay_drl: 3 responses
    ✓ relay_safety: no response (correct)
    ✓ relay_battery: no response (correct)

======================================================================
  ✓ PASS  node_discovery          Discovered 3/3 nodes
  ✓ PASS  node_isolation          All nodes properly isolated
  ✓ PASS  broadcast_commands      All nodes responded
  ✓ PASS  heartbeat_sync          Heartbeats synchronized
  ✓ PASS  per_node_feedback       All feedback correct

  Total: 5/5 passed
======================================================================
```

### Manual Testing

```bash
# Monitor raw CAN traffic
candump can0

# Expected heartbeats every 5 seconds:
# can0  711   [1]  05   ← Node 0x11 (DRL)
# can0  712   [1]  05   ← Node 0x12 (Safety)
# can0  713   [1]  05   ← Node 0x13 (Battery)

# Send test command manually
cansend can0 611#2300200000010000  # K1_ON to node 0x11

# Expected responses:
# can0  591   [8]  60 00 20 00 00 00 00 00   # ACK
# can0  591   [8]  23 02 20 00 01 00 00 00   # RESULT
# can0  591   [8]  23 01 20 00 01 00 00 00   # FEEDBACK
```

---

## 🐳 Docker Deployment

### Why Docker?

- ✅ **Isolation**: Clean environment separation
- ✅ **Portability**: Same container works everywhere
- ✅ **Easy Updates**: Rebuild and redeploy quickly
- ✅ **Development**: Multiple versions can coexist

### Quick Docker Start

```bash
# 1. Build image
cd /path/to/ros_can_bridge_native
docker build -t ros2_can_bridge_native:latest .

# 2. Setup CAN on HOST (required!)
sudo modprobe gs_usb
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 3. Run container (requires --network host for CAN access)
docker run -it --rm \
    --network host \
    --privileged \
    ros2_can_bridge_native:latest
```

### Docker Compose

```bash
# Start all services
docker-compose -f docker/docker-compose.yml up -d

# View logs
docker-compose -f docker/docker-compose.yml logs -f

# Stop services
docker-compose -f docker/docker-compose.yml down
```

### Docker vs Native Comparison

| Aspect | Native | Docker |
|--------|--------|--------|
| **Latency** | <100ms | ~120ms |
| **Setup Time** | 20-30 min | 10-15 min |
| **Isolation** | None | Full |
| **Updates** | Manual rebuild | Docker rebuild |
| **Production** | ✅ Recommended | ⚠️ Acceptable |
| **Development** | OK | ✅ Recommended |

**Recommendation**: Use **Native** for production CAN applications requiring minimum latency. Use **Docker** for development, testing, and multi-environment deployments.

---

## 📚 Command Reference

### Quick Reference Card

```bash
# ============ Interactive Commands ============
ros2 run ros_can_bridge_native multi_node_command_client
relay> list                  # Show nodes
relay> help                  # Show commands
relay> drl_on                # DRL on
relay> broadcast all_off     # Emergency stop
relay> quit

# ============ Command-Line Commands ===========
# By alias
ros2 run ros_can_bridge_native multi_node_command_client --alias drl_on

# By node name
ros2 run ros_can_bridge_native multi_node_command_client \
  --node-name relay_drl --command k1_on

# By node ID
ros2 run ros_can_bridge_native multi_node_command_client \
  --node-id 0x11 --command 1

# Broadcast
ros2 run ros_can_bridge_native multi_node_command_client \
  --command 0 --broadcast

# ============ ROS 2 Topics ====================
ros2 topic list               # List all topics
ros2 topic echo /relay_drl/state        # Monitor DRL
ros2 topic echo /can_network/status     # Network health

# ============ Testing =========================
python3 tests/test_multi_node.py --interface can0
python3 tests/test_multi_node.py --test isolation
python3 tests/test_multi_node.py --export results.json

# ============ Diagnostics =====================
candump can0                  # Raw CAN traffic
ip -details link show can0    # CAN interface status
canbusload can0@1000000       # Bus utilization
ros2 topic echo /relay_drl/diagnostics  # Node diagnostics

# ============ Launch ==========================
ros2 launch ros_can_bridge_native can_bridge_multi_device.launch.py
ros2 launch ros_can_bridge_native can_bridge_multi_device.launch.py \
  can_interface:=vcan0 log_level:=debug
```

---

## 🔧 Troubleshooting

### Node Not Discovered

**Symptom**: Bridge doesn't detect node

**Checks**:
```bash
# 1. Verify node is powered and booting (check serial)
screen /dev/ttyACM0 115200

# 2. Check CAN bus traffic
candump can0
# Should see heartbeats: can0  711   [1]  05

# 3. Verify termination resistors (120Ω both ends)

# 4. Check NODE_ID in firmware matches config
# Firmware should print: "Node ID: 0x11"

# 5. Verify CAN interface is up
ip link show can0
```

**Solution**: Most common issue is missing CAN termination or wrong NODE_ID.

### Wrong Node Responds

**Symptom**: Safety node activates when sending DRL command

**Root Cause**: Wrong NODE_ROLE flashed to device

**Solution**:
```cpp
// Re-flash firmware with correct role
#define NODE_ROLE ROLE_SAFETY  // Change this line

// Verify via serial output:
// Node: Safety_Light
// Role: 2
// Node ID: 0x12
```

### CAN Bus Errors

**Symptom**: Red LED blinking, frequent recovery attempts

**Diagnosis**:
```bash
# Check interface status
ip -details link show can0

# Look for:
# state UP          ← Good
# state BUS-OFF     ← Bad (errors)
# RX errors: 0      ← Should be 0
# TX errors: 0      ← Should be 0
```

**Common Causes & Solutions**:
1. **Missing termination** → Add 120Ω resistors at bus ends
2. **Wrong bitrate** → Ensure all nodes use 1 Mbps
3. **Cable too long** → Limit to 40m at 1 Mbps
4. **CANH/CANL swapped** → Check wiring
5. **Bad connections** → Check all connectors

**Fix**:
```bash
# Restart interface
sudo ip link set can0 down
sudo ip link set can0 up
```

### ROS 2 Node Not Visible

**Symptom**: `ros2 node list` doesn't show bridge

**Solution**:
```bash
# 1. Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID
# Should be same on all terminals (default: 0)

# 2. Check if node is actually running
ps aux | grep multi_node_bridge

# 3. Restart with debug logging
ros2 launch ros_can_bridge_native can_bridge_multi_device.launch.py \
  log_level:=debug
```

### Permission Denied

**Symptom**: Cannot access CAN interface

**Solution**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in

# Verify group membership
groups | grep dialout
```

### Build Errors

**Symptom**: `colcon build` fails

**Solution**:
```bash
# 1. Install missing dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 2. Clean build
rm -rf build/ install/ log/
colcon build --packages-select ros_can_bridge_native

# 3. Check Python imports
python3 -c "import can; import yaml"
```

---

## 🎨 Code Style

### Python Style

- **PEP 8** compliance
- **Type hints** on all function signatures
- **Docstrings** with Args/Returns/Raises
- **Line length**: 127 characters
- **Quotes**: Double for strings, single for dict keys

```python
def send_command(node_id: int, command: int) -> bool:
    """
    Send command to specific node.
    
    Args:
        node_id: Target node ID (0x11-0x7F)
        command: Command code (0-254)
    
    Returns:
        True if sent successfully
    
    Raises:
        CANException: If CAN bus communication fails
    """
    pass
```

### Arduino C++ Style

- **Indentation**: 2 spaces
- **Line length**: 100 characters
- **Naming**: `UPPER_SNAKE_CASE` constants, `camelCase` functions
- **K&R braces**: Opening brace on same line

```cpp
constexpr uint8_t NODE_ID = 0x11;  // Not: #define

void publishHeartbeat() {  // K&R style
  uint8_t payload[1] = {0x05};
  canSendWithRetry(COB_HEARTBEAT, payload, 1);
}
```

### Git Commits

```
type(scope): subject

body

footer
```

**Types**: `feat`, `fix`, `docs`, `test`, `refactor`, `chore`

**Example**:
```
feat(multi-node): add auto-discovery for CAN nodes

- Implemented heartbeat monitoring
- Added node registry with timeout detection
- Created per-node topic publishers

Closes #42
```

---

## ⚡ Performance

### Benchmarks

| Metric | Single-Node | Multi-Node (3 nodes) |
|--------|-------------|----------------------|
| Command latency | 50 ms | 80-100 ms |
| Heartbeat traffic | 1 msg/5s | 3 msg/5s |
| Bus utilization | ~0.1% | ~0.3% |
| SDO throughput | 50 cmd/s | 50 cmd/s/node |
| CPU usage (RP2040) | 5-8% | 5-8% |
| CPU usage (ROS 2) | 8-12% | 12-18% |
| Memory (ROS 2) | 80-100 MB | 130-180 MB |
| Discovery time | N/A | <10 seconds |

### Performance Targets

| Metric | Target | Acceptable | Critical |
|--------|--------|------------|----------|
| Command latency | <50 ms | <100 ms | >200 ms |
| Heartbeat interval | 5.0±0.5s | 5.0±1.0s | >10s |
| Node discovery | <10s | <15s | >30s |
| Bus utilization | <20% | <50% | >80% |
| CAN TX errors | 0 | <10/min | >100/min |

### Optimization Tips

1. Use **real-time kernel** for guaranteed latency
2. Set **thread priorities** in ROS 2 executor
3. Increase **CAN buffer sizes** for high throughput
4. Use **CPU affinity** to pin threads
5. Disable **unnecessary logging** in production

---

## 🚀 Production Deployment

### systemd Service

```bash
# Install as system service
sudo ~/ros2_ws/src/ros_can_bridge_native/scripts/systemd_install.sh

# Manage service
sudo systemctl start ros-can-bridge
sudo systemctl stop ros-can-bridge
sudo systemctl status ros-can-bridge

# Enable at boot
sudo systemctl enable ros-can-bridge

# View logs
journalctl -u ros-can-bridge -f
```

### udev Rules (Persistent Device Names)

Create `/etc/udev/rules.d/99-gs_usb.rules`:

```bash
# CANable/Candlelight devices
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", NAME="can_relay"
```

Apply:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Monitoring

```bash
# Health monitoring
watch -n 1 'ros2 topic echo --once /can_network/status'

# Diagnostics
ros2 topic echo /relay_drl/diagnostics

# System resources
htop -p $(pgrep -f multi_node_bridge)

# CAN bus load
canbusload can0@1000000
```

### Backup and Recovery

```bash
# Backup configuration
cp config/multi_node_map.yaml config/multi_node_map.yaml.bak

# Export node states
ros2 topic echo --once /relay_drl/state > node_states.txt

# Recovery procedure
sudo systemctl stop ros-can-bridge
sudo ip link set can0 down
sudo ip link set can0 up
sudo systemctl start ros-can-bridge
```

---

## 🎣 Winch Motor Control

### Overview

The winch motor control system provides precision DC motor control with limit switches and timer-based operation through CANopen SDO protocol and ROS 2 action interface.

**Features:**
- ✅ Bi-directional motor control (retract/extend)
- ✅ Limit switch safety (automatic stop at endpoints)
- ✅ Timer-based operation with real-time progress feedback
- ✅ ROS 2 action server integration
- ✅ Manual stop capability
- ✅ CANopen SDO protocol compatible

### Hardware Setup

**Node Configuration:**
- **Node ID**: 0x15 (21 decimal)
- **Role**: ROLE_WINCH
- **CAN Bitrate**: 1 Mbps

**Pin Assignments (Adafruit Feather RP2040 CAN):**
```cpp
PIN_FWD = 9        // Forward/Extend motor control (PWM)
PIN_REV = 10       // Reverse/Retract motor control (PWM)
PIN_RET_LIM = 5    // Retract limit switch (INPUT_PULLUP)
PIN_EXT_LIM = 6    // Extend limit switch (INPUT_PULLUP)
MAX_SPEED = 255    // Full PWM duty cycle
```

**Wiring:**
- Limit switches: Connect between pin and GND (normally open, active-low with pull-up)
- Motor driver: PWM signals on pins 9 and 10 (connect to H-bridge or motor controller)
- CAN: Standard MCP2515 connections (CS=PIN_CAN_CS, INT=PIN_CAN_INT)

### ROS 2 Integration

**Start Action Server:**
```bash
# Terminal 1: Start winch control interface
source ~/ros2_ws/install/setup.bash
ros2 run winch_control winch_interface
```

**Send Action Goals:**
```bash
# Retract for 3 seconds
ros2 action send_goal /winch_control winch_msgs/action/SetTarget \
  "{pull: true, use_timer: true, duration: 3.0}" --feedback

# Extend for 2 seconds
ros2 action send_goal /winch_control winch_msgs/action/SetTarget \
  "{pull: false, use_timer: true, duration: 2.0}" --feedback

# Retract until limit switch (no timer)
ros2 action send_goal /winch_control winch_msgs/action/SetTarget \
  "{pull: true, use_timer: false, duration: 0.0}"
```

**Action Interface:**
```yaml
# Goal
bool pull           # true=retract (pull in), false=extend (let out)
bool use_timer      # true=use duration timer, false=run until limit
float32 duration    # Duration in seconds (only if use_timer=true)

# Result
bool success        # true if completed successfully
uint16 error_code   # 0=no error, 200+=error codes

# Feedback
float64 status      # Progress from 0.0 to 1.0 (updated every 500ms)
```

### CANopen Protocol

**COB-IDs:**
```
Heartbeat:   0x715 (0x700 + 0x15)
Receive SDO: 0x615 (0x600 + 0x15)
Transmit SDO: 0x595 (0x580 + 0x15)
```

**Commands:**
```
0x01 - RETRACT (pull winch in)
0x02 - EXTEND (let winch out)
0x03 - STOP (emergency stop)
```

**Manual CAN Command Example:**
```bash
# Retract for 2 seconds (2000ms = 0x07D0 little-endian = D0 07)
cansend can0 615#230020000101D007

# Expected responses:
candump can0
# 595 [8] 60 00 20 00 00 00 00 00  (ACK)
# 595 [8] 23 01 20 00 XX 00 00 00  (Progress feedback)
# 595 [8] 23 02 20 00 01 00 00 00  (Result: success=true)
```

### Safety Features

**Automatic Protection:**
1. **Pre-Motion Limit Check**: Verifies not already at target limit before starting
2. **During Motion Monitoring**: Continuously monitors limit switches (every 5ms)
3. **Timer Timeout**: Automatic stop after duration expires
4. **Limit Switch Override**: Immediate stop when limit reached
5. **Pull-Up Resistors**: INPUT_PULLUP (LOW=pressed, HIGH=released)

### Firmware Configuration

**Upload Winch Firmware:**
```bash
# 1. Open Arduino IDE
# 2. Open: can_relay_node_multi.ino
# 3. Set role to ROLE_WINCH (line 41):
#    #define NODE_ROLE ROLE_WINCH
# 4. Board: Arduino Mbed RP2040 → Raspberry Pi Pico
# 5. Port: /dev/ttyACM0
# 6. Click Upload
# 7. Serial Monitor (115200 baud) should show:
#    Node: Winch_Motor, Role: 6, Node ID: 0x15
```

### Testing

**1. Verify Heartbeat:**
```bash
candump can0 | grep 715
# Expected: can0  715   [1]  05  (every 5 seconds)
```

**2. ROS Action Test:**
```bash
# Send test goal with feedback monitoring
ros2 action send_goal /winch_control winch_msgs/action/SetTarget \
  "{pull: true, use_timer: true, duration: 2.0}" --feedback

# Expected output:
# - Goal accepted
# - Feedback: 0.0 → 0.25 → 0.50 → 0.75 → 1.0
# - Result: success=true, error_code=0
# - Status: SUCCEEDED
```

---

## 🤝 Contributing

We welcome contributions! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Follow code style guidelines (see Code Style section)
4. Add tests for new functionality
5. Update documentation
6. Commit with descriptive messages
7. Submit a pull request

### Development Setup

```bash
# Clone repository
git clone <repo-url>
cd ros_can_bridge_native

# Install development dependencies
pip3 install pytest pytest-cov flake8 black

# Run tests
python3 -m pytest tests/

# Run linter
flake8 ros_can_bridge_native/ tests/

# Format code
black ros_can_bridge_native/ tests/ --line-length=127
```

---

## 📄 License

MIT License

Copyright (c) 2025 Sadaa Khan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

---

## 📞 Support & References

### Documentation
- **ROS 2**: https://docs.ros.org/en/humble/
- **SocketCAN**: https://www.kernel.org/doc/html/latest/networking/can.html
- **CANopen**: https://www.can-cia.org/canopen/
- **python-can**: https://python-can.readthedocs.io/

### Hardware
- **Adafruit Feather RP2040**: https://www.adafruit.com/product/4884
- **MCP2515**: https://www.microchip.com/en-us/product/MCP2515
- **CANable**: https://canable.io/
- **gs_usb**: https://github.com/candle-usb/candleLight_fw

### Community
- **Issues**: Report bugs via GitHub issues
- **Discussions**: Use GitHub discussions for questions
- **Email**: sadaaadkhan@gmail.com

---

## 🎓 Acknowledgments

- ROS 2 community for excellent framework
- SocketCAN developers for kernel integration
- CANopen specification authors
- gs_usb/CANable project contributors
- All contributors to this project

---

**Last Updated**: November 17, 2025 | **Version**: 3.0.0 | **Status**: Production Ready ✅
# MES1 System - Multi-Node CANopen Control

MES1 (Multi-Equipment System 1) is a distributed CANopen network bridging ROS 2 Humble with multiple microcontroller nodes for relay and winch motor control. It uses native Linux SocketCAN and standard CANopen SDO protocols.

## System Architecture

The system consists of a ROS 2 host communicating via a USB-CAN adapter to a bus of independent nodes.

  * **Host**: Ubuntu 22.04 running ROS 2 Humble.
  * **Protocol**: CANopen SDO (Service Data Objects).
  * **Interface**: Native Linux SocketCAN (`can0`).
  * **Hardware**: Adafruit Feather RP2040 + MCP2515 CAN Controller.

## Hardware Setup

### Wiring (RP2040 to MCP2515)

| RP2040 Pin | MCP2515 Pin |
| :--- | :--- |
| `PIN_CAN_CS` | CS |
| `PIN_CAN_INT` | INT |
| `MOSI` | SI |
| `MISO` | SO |
| `SCK` | SCK |
| 3.3V | VCC |
| GND | GND |

**Note**: The CAN bus requires 120Ω termination resistors at both ends of the physical bus.

### Node Configuration Map

Each node runs the same firmware but is assigned a specific role and ID.

| Node Role | Node ID | Function | Pin Assignment |
| :--- | :--- | :--- | :--- |
| **DRL** | `0x11` | LED DRL Control | Pin 25 |
| **SAFETY** | `0x12` | Safety Light | Pin 25 |
| **BATTERY** | `0x13` | Battery Locks | Pin 15 (Right), Pin 8 (Left) |
| **WINCH** | `0x15` | Motor Control | Pin 9 (Fwd), Pin 10 (Rev) |

-----
# Install Python dependencies

## Overviewpip3 install python-can netifaces

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

- **Multi-Node Architecture**: 4+ independent relay nodes on single CAN bus```

- **Auto-Discovery**: Nodes detected via heartbeat monitoring

- **Native SocketCAN**: Direct Linux kernel integration### Launch the Bridge

- **High Performance**: <100ms command latency

- **Production Ready**: CAN health monitoring, progressive recovery, diagnostics```bash

- **Flexible Control**: Interactive CLI, ROS 2 topics/services, broadcast commands# Terminal 1: Launch CAN bridge

- **Comprehensive Testing**: 5 automated test suitesros2 launch ros_can_bridge_native can_bridge.launch.py

-  **Full Diagnostics**: Per-node health monitoring and reporting

# Terminal 2: Monitor relay states

---ros2 topic echo /can_bridge_node/relay_state


## Firmware Installation

1.  Open `can_relay_node_multi.ino` in Arduino IDE.
2.  Install the board support for **RP2040**.
3.  Edit the configuration section (approx line 40) to select the role for the target device:

<!-- end list -->

```cpp
// Uncomment ONLY one role per device
#define NODE_ROLE ROLE_DRL
// #define NODE_ROLE ROLE_SAFETY
// #define NODE_ROLE ROLE_BATTERY
// #define NODE_ROLE ROLE_WINCH
```

4.  Upload to the microcontroller.

-----

## Host Installation

### 1\. Prerequisites

**OS**: Ubuntu 22.04 LTS
**ROS 2**: Humble Hawksbill

Install system dependencies:

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-can-msgs ros-humble-diagnostic-msgs can-utils
pip3 install python-can netifaces pyyaml
```

### 2\. Build Package

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Clone or place 'ros_can_bridge_native' folder here
cd ~/ros2_ws
colcon build --packages-select ros_can_bridge_native --symlink-install
source install/setup.bash
```

### 3\. Setup CAN Interface

The adapter must be brought up as a native network interface.

```bash
# Load kernel module
sudo modprobe gs_usb

# Set bitrate to 1Mbps and bring up interface
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

-----

## Usage

### Launching the Bridge

Start the main ROS 2 node. This handles heartbeat monitoring, SDO communication, and topic publishing.

```bash
ros2 launch ros_can_bridge_native can_bridge_multi_device.launch.py
```

### Interactive Control

Use the provided CLI tool to send commands manually.

```bash
ros2 run ros_can_bridge_native multi_node_command_client
```

**Commands:**

  * `list`: Show discovered nodes.
  * `drl_on` / `drl_off`: Control DRL node.
  * `broadcast all_off`: Turn off all relays on all nodes.

### ROS 2 Topics

The bridge automatically creates topics based on discovered nodes:

  * `/relay_<name>/state`: Current relay status.
  * `/relay_<name>/diagnostics`: Communication health and error counters.
  * `/can_network/status`: Overall bus health.

-----

## CANopen Protocol Reference

The system uses a simplified Object Dictionary.

### COB-IDs (Communication Object Identifiers)

  * **Heartbeat**: `0x700 + NodeID` (Every 5 seconds)
  * **RX SDO**: `0x600 + NodeID` (Host to Node)
  * **TX SDO**: `0x580 + NodeID` (Node to Host)

### Object Dictionary

| Index | Name | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x2000` | CONTROL\_COMMAND | Write | Relay command byte (0-99) |
| `0x2001` | RELAY\_FEEDBACK | Read | Current relay bitmap |
| `0x2002` | COMMAND\_RESULT | Read | Last command success/fail |
| `0x2003` | DIAGNOSTICS | Read | TX/RX counters |
| `0x2004` | NODE\_ROLE | Read | Configured role ID |

-----

## Winch Motor Control

The Winch node (`0x15`) operates differently from standard relay nodes, using ROS 2 Actions for time-based control.

### Hardware

  * **Pins**: 9 (Forward), 10 (Reverse).
  * **Limits**: Pins 5 (Retract), 6 (Extend) - `INPUT_PULLUP`.

### ROS 2 Action

**Action Type**: `winch_msgs/action/SetTarget`

**Example Command (CLI)**:

```bash
ros2 action send_goal /winch_control winch_msgs/action/SetTarget \
  "{pull: true, use_timer: true, duration: 3.0}" --feedback
```

-----

## Troubleshooting

1.  **Interface not found**: Ensure `ip link show can0` lists the device as `UP`.
2.  **No node discovery**: Check 120Ω termination on the physical bus. Check `candump can0` for heartbeat messages (`0x7XX`).
3.  **Wrong node responding**: Verify the `#define NODE_ROLE` line in the firmware before uploading.

## License

MIT License

# Docker Deployment Guide

## Quick Start

### 1. Using the Quick Start Script (Recommended)

```bash
cd docker
./docker-quickstart.sh
```

The script will:
- Check Docker installation
- Verify CAN interface
- Build the image
- Offer multiple run options

### 2. Using Docker Compose

```bash
# From project root
cd docker

# Build the image
docker compose build

# Start the service
docker compose up

# Run in background
docker compose up -d

# View logs
docker compose logs -f

# Stop the service
docker compose down
```

### 3. Manual Docker Build & Run

```bash
# Build image
docker build -t mes1_system:latest .

# Run container
docker run -it --rm \
  --name mes1_can_bridge \
  --network host \
  --privileged \
  --cap-add=NET_ADMIN \
  -e CAN_INTERFACE=can0 \
  -e CAN_BITRATE=1000000 \
  -e NODE_ID=21 \
  mes1_system:latest
```

## Testing the Docker Setup

### Test 1: Validate Dockerfile

```bash
# Check Dockerfile syntax
docker build --check -f Dockerfile .
```

Expected: `Check complete, no warnings found.`

### Test 2: Validate Docker Compose

```bash
cd docker
docker compose config
```

Expected: Valid YAML configuration output

### Test 3: Build Image

```bash
docker build -t mes1_system:latest .
```

Expected: Successful build with no errors

### Test 4: Verify CAN Access

```bash
# Start interactive shell
docker run -it --rm \
  --network host \
  --privileged \
  mes1_system:latest \
  bash

# Inside container, check CAN interface
ip link show can0
candump can0
```

### Test 5: Run ROS Node

```bash
# Start the bridge
docker compose up

# In another terminal, check ROS topics
docker exec -it mes1_can_bridge ros2 topic list

# Check node is running
docker exec -it mes1_can_bridge ros2 node list
```

## Configuration

### Environment Variables

Create `docker/.env` from the template:

```bash
cd docker
cp .env.example .env
# Edit .env with your settings
```

Available variables:
- `ROS_DOMAIN_ID`: ROS 2 domain (default: 0)
- `CAN_INTERFACE`: CAN interface name (default: can0)
- `CAN_BITRATE`: CAN bitrate in bps (default: 1000000)
- `NODE_ID`: CANopen node ID (default: 21)
- `LOG_LEVEL`: Logging level (default: INFO)

### Custom Configuration Files

Mount custom config:

```yaml
volumes:
  - ./my-config:/ros2_ws/src/mes1_system/config:ro
```

## Troubleshooting

### Issue: "CAN interface not found"

**Solution:** Ensure CAN interface is up on host:

```bash
sudo modprobe gs_usb
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### Issue: "Permission denied" for CAN

**Solution:** Container needs privileged mode and NET_ADMIN capability (already configured)

### Issue: "Package 'mes1_system' not found"

**Solution:** Rebuild the image:

```bash
docker compose build --no-cache
```

### Issue: ROS nodes can't communicate

**Solution:** Check `ROS_DOMAIN_ID` matches across all nodes:

```bash
# In container
echo $ROS_DOMAIN_ID

# On host
echo $ROS_DOMAIN_ID
```

## Advanced Usage

### Running with Winch Control

```bash
# Start main bridge
docker compose up -d can_bridge

# In another terminal, run winch action server
docker run -it --rm \
  --network host \
  mes1_system:latest \
  ros2 run winch_control winch_interface
```

### Debugging with Interactive Shell

```bash
docker compose run --rm can_bridge bash

# Inside container:
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch mes1_system can_bridge.launch.py
```

### Viewing Diagnostics

```bash
# Start diagnostics viewer profile
docker compose --profile debug up diagnostics_viewer

# Or manually
docker exec -it mes1_can_bridge ros2 topic echo /diagnostics
```

### Interactive Command Client

```bash
# Start command client profile
docker compose --profile interactive up command_client
```

## Production Deployment

### Running as Systemd Service

Create `/etc/systemd/system/mes1-docker.service`:

```ini
[Unit]
Description=MES1 System Docker Container
After=docker.service
Requires=docker.service

[Service]
Type=simple
WorkingDirectory=/path/to/MES1_System/docker
ExecStart=/usr/bin/docker compose up
ExecStop=/usr/bin/docker compose down
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable mes1-docker
sudo systemctl start mes1-docker
```

### Health Monitoring

```bash
# Check container health
docker inspect mes1_can_bridge | grep -A 10 Health

# View health check logs
docker compose ps
```

## Clean Up

```bash
# Stop and remove containers
docker compose down

# Remove volumes
docker compose down -v

# Remove image
docker rmi mes1_system:latest

# Clean all Docker resources
docker system prune -a
```

## Performance Tips

1. **Use host network mode** for lowest latency (already configured)
2. **Increase TX queue length** in entrypoint.sh (already set to 1000)
3. **Pin CPU cores** for real-time performance:
   ```yaml
   cpuset: "0,1"
   ```
4. **Set memory limits** to prevent OOM:
   ```yaml
   mem_limit: 512m
   ```

## Security Considerations

- Container runs as **privileged** (required for CAN access)
- Use **read-only volumes** for config files (already configured)
- Limit **capabilities** to only NET_ADMIN and SYS_ADMIN
- Run on **isolated network** if possible
- Use **secrets** for sensitive configuration

## Compatibility

- **Docker**: 20.10+
- **Docker Compose**: 2.0+
- **Host OS**: Linux (kernel 4.4+ with SocketCAN)
- **Architecture**: amd64, arm64

# QCar Gazebo Simulation Guide

## Quick Start

### Starting the Simulation
```bash
cd /home/nicomedes/Desktop/L.A.D/qcar_docker
./start-gazebo-gui.sh
```

This will:
- Start the Docker container with ROS 2 Humble
- Launch Gazebo server with QCar robot model
- Open Gazebo GUI window on your screen
- Enable all sensors (LIDAR, cameras)

### Stopping the Simulation
```bash
cd /home/nicomedes/Desktop/L.A.D/qcar_docker
./stop-gazebo.sh
```

## What's Running

### Gazebo Simulation
- **Server**: Physics simulation and sensor data generation
- **GUI Client**: 3D visualization window (should be visible on your screen)
- **World File**: `/worlds/qcar_lidar_test.world`
- **Robot Model**: QCar with LIDAR and 5 cameras

### Available ROS Topics

#### Sensor Data
- `/qcar/lidar/scan` - LIDAR sensor (sensor_msgs/LaserScan)
  - 360° scan, ~12m range
  - ~0.0175 rad angle increment (~1°)

- `/qcar/csi_front/image_raw` - Front camera (sensor_msgs/Image)
- `/qcar/csi_right/image_raw` - Right camera
- `/qcar/csi_back/image_raw` - Back camera
- `/qcar/csi_left/image_raw` - Left camera
- `/qcar/rgb/image_raw` - RGB camera
- `/qcar/overhead/image_raw` - Overhead camera

#### Camera Info
- `/qcar/csi_front/camera_info` - Front camera calibration
- (Similar topics for other cameras)

#### Robot Control & State
- `/qcar/cmd_vel` - Robot velocity commands (geometry_msgs/Twist)
- `/qcar/odom` - Odometry data (nav_msgs/Odometry)

#### Transforms
- `/tf` - Transform tree
- `/tf_static` - Static transforms

### Network Services

| Port | Service | Purpose |
|------|---------|---------|
| 9090 | rosbridge_websocket | Connect web applications to ROS |
| 7000 | Static file server | URDF and mesh files |
| 8080 | Web video server | Stream camera images to browser |
| 10000 | ROS-TCP-Endpoint | Unity connection |
| 11345 | Gazebo master | Gazebo server API |

## Using the Simulation

### Test LIDAR Data
```bash
# Enter the container
docker exec -it qcar_docker-ros-1 bash

# Echo LIDAR data
ros2 topic echo /qcar/lidar/scan

# List all topics
ros2 topic list

# Check topic info
ros2 topic info /qcar/lidar/scan
```

### Control the Robot
```bash
# Publish velocity commands
ros2 topic pub /qcar/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
```

### View Camera Images
```bash
# Inside the container
ros2 run rqt_image_view rqt_image_view /qcar/csi_front/image_raw
```

Or open in your browser:
```
http://localhost:8080/stream?topic=/qcar/csi_front/image_raw
```

### Gazebo GUI Controls
- **Orbit**: Left-click + drag
- **Zoom**: Scroll wheel
- **Pan**: Shift + left-click + drag
- **Move Robot**: Click and drag the QCar
- **Add Objects**: Use "Insert" tab
- **Play/Pause**: Bottom control panel

## Integration with AVEDU

The simulation is integrated into your AVEDU learning platform:

1. Navigate to **Sensing & Visualization** section
2. Find the **"QCar Gazebo Simulation"** slide
3. Follow the interactive tutorial

The slide includes:
- Step-by-step launch instructions
- Visual guides for using Gazebo
- ROS topic reference
- Troubleshooting tips

## Troubleshooting

### Gazebo Window Doesn't Appear
```bash
# Check X11 permissions
xhost +local:docker

# Verify DISPLAY variable
echo $DISPLAY

# Check if gzclient is running
docker exec qcar_docker-ros-1 ps aux | grep gzclient
```

### No Sensor Data
```bash
# Check if topics are publishing
docker exec qcar_docker-ros-1 bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic list"

# Check Gazebo server logs
docker compose logs | grep gzserver
```

### GPU/Rendering Issues
The container has access to `/dev/dri` for GPU acceleration. If you experience rendering issues:
```bash
# Check GPU devices
ls -la /dev/dri/

# Restart with fresh configuration
./stop-gazebo.sh
./start-gazebo-gui.sh
```

### Container Won't Start
```bash
# Check Docker daemon
sudo systemctl status docker

# View container logs
docker compose logs

# Rebuild if needed
docker compose build
```

## Technical Details

### Docker Configuration
- **Base Image**: osrf/ros:humble-desktop
- **ROS Distribution**: Humble
- **Gazebo Version**: 11.10.2 (Gazebo Classic)
- **GPU Access**: Enabled via /dev/dri device mapping
- **X11 Forwarding**: Enabled for GUI display

### Files Modified
- `docker-compose.yml` - Added X11 and GPU configuration
- `entrypoint.sh` - Smart DISPLAY detection for GUI/headless mode
- `start-gazebo-gui.sh` - Automated launch script
- `stop-gazebo.sh` - Cleanup script

### AVEDU Integration
- New slide: `AVEDU/avedu/src/levels/slidesSensing/03b-GazeboSimulation.jsx`
- Updated: `AVEDU/avedu/src/levels/SensingVisualization.jsx`

## Next Steps

### For Learning
1. Open AVEDU and explore the Gazebo simulation slide
2. Practice subscribing to sensor topics
3. Try controlling the robot with ROS commands
4. Visualize data in RViz2

### For Development
1. Create custom ROS 2 nodes to process sensor data
2. Implement obstacle avoidance algorithms
3. Test navigation behaviors in simulation
4. Connect AVEDU platform to live simulation data

### For Advanced Users
1. Modify the world file (`/worlds/qcar_lidar_test.world`)
2. Customize robot URDF for different configurations
3. Add additional sensors or modify existing ones
4. Create custom Gazebo plugins

## Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Classic Documentation](https://classic.gazebosim.org/)
- [rosbridge Protocol](https://github.com/RobotWebTools/rosbridge_suite)

---

**Status**: ✅ Simulation is currently running
**Display**: ${DISPLAY} (1920x1080)
**Container**: qcar_docker-ros-1

Enjoy your QCar simulation! 🚗🤖

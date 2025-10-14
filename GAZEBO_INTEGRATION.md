# Gazebo Integration for QCar Platform

This document explains how to use the Gazebo simulation integration in the L.A.D platform.

## Overview

Gazebo has been integrated into the QCar ROS 2 Docker environment, allowing you to simulate the QCar robot with:
- **Cameras**: RGB camera + 4 CSI cameras (front, right, back, left)
- **LIDAR**: 360-degree laser scanner
- **Differential Drive**: Control the robot with `/qcar/cmd_vel`
- **Odometry**: Published on `/qcar/odom`

## Architecture

The integration uses:
1. **ROS 2 Gazebo plugins** (updated for Humble compatibility)
2. **Headless Gazebo server** running in Docker via Xvfb
3. **React component** to visualize sensor data via rosbridge WebSocket

## Files Modified/Created

### Docker & ROS Configuration
- `qcar_docker/qcar/rosws/src/qcar_description/urdf/qcar.gazebo.xacro` - Updated to ROS 2 Gazebo plugin syntax
- `qcar_docker/qcar/rosws/src/qcar_bringup/launch/gazebo_sim.launch.py` - New standalone Gazebo launch file
- `qcar_docker/qcar/rosws/src/qcar_bringup/launch/web_viz.launch.py` - Updated to optionally include Gazebo
- `qcar_docker/entrypoint.sh` - Added Gazebo startup with Xvfb
- `qcar_docker/docker-compose.yml` - Added `ENABLE_GAZEBO` environment variable
- `qcar_docker/Dockerfile` - Already had required Gazebo packages

### React Frontend
- `AVEDU/avedu/src/components/gazebo/GazeboSimViewer.jsx` - React component for visualization
- `AVEDU/avedu/src/styles/components/GazeboSimViewer.css` - Styling for Gazebo viewer

## How to Use

### 1. Start Gazebo with Docker Compose

Gazebo is **enabled by default** in `docker-compose.yml`. To control it:

```bash
cd qcar_docker

# Start with Gazebo enabled (default)
docker compose up

# Or disable Gazebo by editing docker-compose.yml:
# Set ENABLE_GAZEBO=0
```

### 2. Verify Gazebo is Running

Check the Docker logs:

```bash
docker compose logs -f ros
```

You should see:
```
[entrypoint] Starting Xvfb for headless Gazebo...
[INFO] [gzserver-...] Gazebo multi-robot simulator, version ...
[INFO] [spawn_entity.py-...] Waiting for service /spawn_entity ...
```

### 3. Check ROS Topics

```bash
# Enter the container
docker exec -it qcar_docker-ros-1 bash

# List topics (you should see Gazebo topics)
ros2 topic list

# Expected topics:
# /qcar/cmd_vel              # Command velocity input
# /qcar/odom                 # Odometry output
# /qcar/rgb/image_color      # RGB camera
# /qcar/csi_front/image_raw  # Front CSI camera
# /qcar/csi_right/image_raw  # Right CSI camera
# /qcar/csi_back/image_raw   # Back CSI camera
# /qcar/csi_left/image_raw   # Left CSI camera
# /qcar/lidar/scan           # LIDAR data
```

### 4. Use in React App

Import and use the `GazeboSimViewer` component:

```jsx
import React from 'react';
import useRoslib from '../hooks/useRoslib';
import GazeboSimViewer from '../components/gazebo/GazeboSimViewer';

function GazeboLevel() {
  const { ros, connected } = useRoslib();

  return (
    <div className="gazebo-level">
      <h1>Gazebo Simulation</h1>
      <GazeboSimViewer ros={ros} connected={connected} />
    </div>
  );
}

export default GazeboLevel;
```

### 5. Control the Robot

You can control the robot by publishing to `/qcar/cmd_vel`:

```bash
# From inside the Docker container
ros2 topic pub /qcar/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Or from your React app:

```javascript
// In your React component with ros connection
const moveForward = () => {
  if (!ros?.advertise) return;

  const cmdVelTopic = ros.advertise('/qcar/cmd_vel', 'geometry_msgs/msg/Twist');

  cmdVelTopic.publish({
    linear: { x: 0.5, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  });
};
```

## Gazebo Plugin Details

### Differential Drive Plugin
- **Type**: `libgazebo_ros_diff_drive.so`
- **Left wheel joint**: `left_rear_axle`
- **Right wheel joint**: `right_rear_axle`
- **Wheel separation**: 0.112m
- **Wheel diameter**: 0.053m
- **Publishes**: Odometry on `/qcar/odom`
- **Subscribes**: Velocity commands on `/qcar/cmd_vel`

### Camera Plugins
- **Type**: `libgazebo_ros_camera.so`
- **RGB Camera**: 640x480 @ 30Hz on `/qcar/rgb/image_color`
- **CSI Cameras**: 1280x640 @ 30Hz with fisheye distortion

### LIDAR Plugin
- **Type**: `libgazebo_ros_ray_sensor.so`
- **Samples**: 360 rays
- **Range**: 0.12m to 12.0m
- **Update rate**: 10Hz
- **Topic**: `/qcar/lidar/scan`

## Troubleshooting

### Gazebo doesn't start
**Check Docker logs:**
```bash
docker compose logs ros | grep -i gazebo
```

**Verify Xvfb is running:**
```bash
docker exec -it qcar_docker-ros-1 ps aux | grep Xvfb
```

### No camera images in React
**Check if images are being published:**
```bash
docker exec -it qcar_docker-ros-1 bash
ros2 topic hz /qcar/rgb/image_color
```

**Issue**: If no data, the Gazebo simulation may not have spawned the robot correctly.

**Solution**: Check spawn_entity logs and ensure URDF is valid.

### Robot doesn't move
**Check cmd_vel topic:**
```bash
ros2 topic echo /qcar/cmd_vel
```

**Verify differential drive plugin loaded:**
```bash
docker compose logs ros | grep "differential_drive"
```

### Performance Issues
Gazebo can be resource-intensive. If experiencing lag:

1. **Reduce camera update rates** in `qcar.gazebo.xacro`:
   ```xml
   <update_rate>10.0</update_rate>  <!-- Instead of 30 -->
   ```

2. **Disable unused cameras** by commenting them out in the URDF

3. **Reduce LIDAR samples**:
   ```xml
   <samples>180</samples>  <!-- Instead of 360 -->
   ```

## Environment Variables

Configure Gazebo behavior via `docker-compose.yml`:

```yaml
environment:
  - ENABLE_GAZEBO=1           # Enable/disable Gazebo (1=on, 0=off)
  - GAZEBO_WORLD=             # Path to custom world file (empty=default)
  - DISPLAY=:99               # Virtual display for headless mode
```

## Advanced: Custom Worlds

To load a custom Gazebo world:

1. Create a world file (e.g., `my_world.world`)
2. Mount it in docker-compose.yml:
   ```yaml
   volumes:
     - ./worlds:/worlds
   ```
3. Set environment variable:
   ```yaml
   environment:
     - GAZEBO_WORLD=/worlds/my_world.world
   ```

## Next Steps

### Suggested Enhancements
1. **Add teleop controls** to the React component
2. **Create educational levels** using Gazebo simulation
3. **Implement obstacle avoidance** using LIDAR data
4. **Add more sensors** (IMU, GPS) to the URDF
5. **Create custom worlds** for different scenarios

## Resources

- [Gazebo ROS 2 Plugins Documentation](http://gazebosim.org/tutorials?tut=ros2_overview)
- [ROS 2 Humble Gazebo Integration](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [URDF in Gazebo Tutorial](http://gazebosim.org/tutorials?tut=ros_urdf)

## Support

If you encounter issues:
1. Check the Docker logs: `docker compose logs ros`
2. Verify ROS topics: `ros2 topic list`
3. Test with command line tools before React integration
4. Ensure rosbridge is connected in your browser console

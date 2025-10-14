# Gazebo Integration - Quick Start Guide

This guide will help you get Gazebo simulation running with your QCar platform in just a few steps.

## What Was Added

### Backend (ROS 2 Docker)
1. **Updated URDF with ROS 2 Gazebo plugins** (`qcar.gazebo.xacro`)
   - Differential drive controller
   - 5 camera sensors (RGB + 4 CSI with fisheye distortion)
   - LIDAR sensor
   - Wheel contact physics

2. **Launch files for Gazebo**
   - `gazebo_sim.launch.py` - Standalone Gazebo launcher
   - `web_viz.launch.py` - Updated to optionally include Gazebo

3. **Docker configuration**
   - `entrypoint.sh` - Starts Xvfb for headless Gazebo
   - `docker-compose.yml` - Enables Gazebo by default

### Frontend (React)
1. **GazeboSimViewer** - Component to display simulation data
   - Camera feeds from all 5 cameras
   - LIDAR visualization (2D top-down view)
   - Odometry tracking

2. **RobotTeleop** - Control component
   - Keyboard controls (WASD/Arrow keys)
   - Button controls for mobile
   - Adjustable speed settings

3. **GazeboSimLevel** - Complete example level
   - Combines viewer and teleop
   - Instructions and learning objectives

## Quick Start (5 Minutes)

### Step 1: Rebuild Docker Container

**On Windows:**
```cmd
cd qcar_docker
rebuild-gazebo.bat
```

**On Linux/Mac:**
```bash
cd qcar_docker
chmod +x rebuild-gazebo.sh
./rebuild-gazebo.sh
```

This will:
- Stop existing containers
- Rebuild with Gazebo support
- Start Gazebo simulation
- Show logs

### Step 2: Verify Gazebo is Running

Look for these lines in the logs:
```
[entrypoint] Starting Xvfb for headless Gazebo...
[INFO] [gzserver-...] Gazebo multi-robot simulator, version ...
[INFO] [spawn_entity.py-...] Successfully spawned entity 'qcar'
```

Press `Ctrl+C` to stop viewing logs (container keeps running).

### Step 3: Test ROS Topics

Open a new terminal and check the topics:

```bash
# Enter the container
docker exec -it qcar_docker-ros-1 bash

# List all topics
ros2 topic list

# You should see:
# /qcar/cmd_vel
# /qcar/odom
# /qcar/rgb/image_color
# /qcar/csi_front/image_raw
# /qcar/csi_right/image_raw
# /qcar/csi_back/image_raw
# /qcar/csi_left/image_raw
# /qcar/lidar/scan

# Test publishing a command
ros2 topic pub /qcar/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --once

# Check odometry
ros2 topic echo /qcar/odom --once
```

### Step 4: Add to Your React App

#### Option A: Use the Complete Level

Add to your routing (`App.js`):

```jsx
import GazeboSimLevel from './levels/GazeboSimLevel';

// In your routes:
<Route path="/gazebo-sim" element={<GazeboSimLevel />} />
```

#### Option B: Use Individual Components

```jsx
import React from 'react';
import useRoslib from './hooks/useRoslib';
import GazeboSimViewer from './components/gazebo/GazeboSimViewer';
import RobotTeleop from './components/gazebo/RobotTeleop';

function MyGazeboPage() {
  const { ros, connected } = useRoslib();

  return (
    <div>
      <h1>My Gazebo Simulation</h1>
      <GazeboSimViewer ros={ros} connected={connected} />
      <RobotTeleop ros={ros} connected={connected} />
    </div>
  );
}
```

### Step 5: Start Your React App

```bash
cd AVEDU/avedu
npm start
```

Navigate to your Gazebo page and you should see:
- Camera feeds (select from dropdown)
- LIDAR visualization
- Odometry data
- Teleop controls

## Test the Complete System

1. **Open React app** in browser (http://localhost:3000 or your IP)
2. **Navigate to Gazebo level**
3. **Verify "Connected" status** appears
4. **Use keyboard or buttons** to move the robot
5. **Watch the sensors update**:
   - Odometry position changes
   - LIDAR shows environment
   - Cameras show simulated view

## Controlling the Robot

### From React App
- Use the `RobotTeleop` component
- Keyboard: W/A/S/D or Arrow keys
- Mouse: Click and hold direction buttons
- Adjust speeds with sliders

### From Command Line
```bash
# Move forward
ros2 topic pub /qcar/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Turn left
ros2 topic pub /qcar/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Stop
ros2 topic pub /qcar/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Configuration Options

### Enable/Disable Gazebo

Edit `qcar_docker/docker-compose.yml`:

```yaml
environment:
  - ENABLE_GAZEBO=1  # Set to 0 to disable
```

Then restart:
```bash
docker compose restart
```

### Change Simulation Parameters

Edit `qcar_docker/qcar/rosws/src/qcar_description/urdf/qcar.gazebo.xacro`:

- **Camera update rate**: Change `<update_rate>30.0</update_rate>`
- **LIDAR samples**: Change `<samples>360</samples>`
- **Speed limits**: Adjust `<max_wheel_torque>` and `<max_wheel_acceleration>`

After changes, rebuild:
```bash
docker compose build && docker compose up -d
```

## Troubleshooting

### "Not connected to ROS" in React
1. Check rosbridge is running: `docker compose logs ros | grep rosbridge`
2. Verify WebSocket URL in browser console
3. Check CORS settings in docker-compose.yml

### Robot doesn't move
1. Check cmd_vel topic: `ros2 topic echo /qcar/cmd_vel`
2. Verify Gazebo spawned robot: `docker compose logs ros | grep spawn`
3. Check differential drive plugin loaded in logs

### No camera images
1. Verify topic publishing: `ros2 topic hz /qcar/rgb/image_color`
2. Check image encoding is supported (rgb8/bgr8)
3. Reduce camera update rate if performance is poor

### High CPU usage
1. Reduce camera update rates (30Hz → 10Hz)
2. Reduce LIDAR samples (360 → 180)
3. Disable unused cameras in URDF

## Next Steps

1. **Create custom worlds**: Add obstacles and environments
2. **Implement autonomous navigation**: Use LIDAR for obstacle avoidance
3. **Add more sensors**: IMU, GPS, depth cameras
4. **Create educational levels**: Use Gazebo for interactive learning
5. **Integrate with existing levels**: Combine with your current ROS widgets

## Additional Resources

- **Full documentation**: See `GAZEBO_INTEGRATION.md`
- **ROS 2 Gazebo docs**: http://gazebosim.org/tutorials?tut=ros2_overview
- **Differential drive plugin**: https://github.com/ros-simulation/gazebo_ros_pkgs

## Support

If you encounter issues:

1. **Check Docker logs**: `docker compose logs -f ros`
2. **Verify ROS topics**: `ros2 topic list` inside container
3. **Test with CLI first**: Use `ros2 topic pub/echo` before React integration
4. **Check browser console**: For WebSocket/rosbridge errors

---

**Congratulations!** You now have a fully functional Gazebo simulation integrated with your React frontend!

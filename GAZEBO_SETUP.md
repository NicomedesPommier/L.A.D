# Gazebo Simulation Level - Complete Setup Guide

## Overview

A complete Gazebo simulation interface has been created for autonomous vehicle control and visualization. This level includes:
- **3 Introduction Slides** explaining Gazebo, architecture, and sensors
- **Interactive Simulator** with real-time camera feed, sensor data, and keyboard control
- **ROS 2 Integration** via rosbridge WebSocket
- **Docker-based** Gazebo simulation environment

## Files Created

### 1. Main Level Component
- **Location:** `AVEDU/avedu/src/levels/GazeboSim.jsx`
- **Description:** Main component that handles intro slides and launches simulator
- **Features:**
  - Auto-imports intro slides
  - Slide navigation
  - "Launch Simulation" button on final slide
  - Switches between slides and simulator view

### 2. Introduction Slides

#### Slide 01: Introduction (`slidesGazeboIntro/01-Introduction.jsx`)
- What is Gazebo?
- Key features (physics, sensors, ROS integration)
- Why use simulation
- Level objectives

#### Slide 02: Architecture (`slidesGazeboIntro/02-GazeboArchitecture.jsx`)
- System architecture diagram (Browser → rosbridge → Docker → Gazebo)
- Gazebo components explanation
- ROS 2 topics and message types
- Data flow visualization

#### Slide 03: Sensors & Control (`slidesGazeboIntro/03-SensorsAndControl.jsx`)
- Camera sensor details
- LiDAR sensor specifications
- IMU sensor data
- Vehicle control via `/cmd_vel`
- Keyboard mapping guide
- Odometry tracking

### 3. Simulator Components

#### GazeboSimulator (`components/gazebo/GazeboSimulator.jsx`)
- **Main controller** for the simulation interface
- **ROS Integration:**
  - Subscribes to `/odom`, `/imu`, `/scan` topics
  - Publishes to `/cmd_vel` for vehicle control
  - Tracks connection status
- **State Management:**
  - Vehicle position and orientation
  - Velocity (linear, angular)
  - Sensor data (IMU, LiDAR, odometry)
- **Objectives Tracking:**
  - First drive detection
  - 10-second drive milestone
  - 30-second drive milestone
- **Layout:** 3-column grid (sensors | camera | controls)

#### CameraFeed (`components/gazebo/CameraFeed.jsx`)
- **Subscribes to:** `/camera/image_raw` (sensor_msgs/Image)
- **Features:**
  - Real-time video rendering on HTML5 canvas
  - Supports RGB8 and BGR8 encodings
  - FPS counter
  - Resolution display
  - Connection status indicator
- **Performance:** Throttled to ~30 FPS

#### KeyboardControl (`components/gazebo/KeyboardControl.jsx`)
- **Keyboard Mapping:**
  - `W` / `↑`: Forward
  - `S` / `↓`: Backward
  - `A` / `←`: Turn Left
  - `D` / `→`: Turn Right
  - `Space`: Stop
  - `+` / `-`: Adjust speed
- **Features:**
  - Visual keyboard display showing active keys
  - Speed adjustment (linear and angular)
  - Velocity bar graphs
  - Emergency stop button
  - Connection status
- **Safety Limits:**
  - Max linear: 2.0 m/s
  - Max angular: 1.5 rad/s

#### SensorPanel (`components/gazebo/SensorPanel.jsx`)
- **Generic sensor data display**
- **Features:**
  - Icon and title
  - Key-value table format
  - Empty state handling
- **Used for:**
  - Odometry data
  - IMU data
  - LiDAR data

### 4. Styling
- **File:** `styles/components/_gazebo.scss`
- **Features:**
  - Neon + glass morphism theme
  - Responsive 3-column grid
  - Active key highlighting
  - Status indicators with pulse animation
  - Camera feed placeholder
  - Velocity bar animations

### 5. Routing Integration
- **File Modified:** `pages/LearnLevel.jsx`
- **Level Slug:** `gazebo-sim`
- **Added to REGISTRY**

## Docker Setup

### Prerequisites

1. **Docker installed** on your system
2. **Docker Compose** (usually comes with Docker Desktop)
3. **ROS 2 Humble** Docker image
4. **Gazebo Garden** or **Gazebo Classic 11**

### Recommended Docker Configuration

Create a `docker-compose.yml` file in `qcar_docker/` or a new `gazebo_docker/` directory:

```yaml
version: '3.8'

services:
  gazebo-sim:
    image: osrf/ros:humble-desktop-full
    container_name: lad-gazebo-sim
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        apt-get update &&
        apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control &&
        apt-get install -y ros-humble-rosbridge-server &&
        ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
        sleep 5 &&
        ros2 launch your_vehicle_package vehicle_gazebo.launch.py
      "
    network_mode: host
    privileged: true
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - ROS_DOMAIN_ID=0
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./worlds:/root/worlds:ro
      - ./models:/root/models:ro
      - ./vehicle_description:/root/colcon_ws/src/vehicle_description:ro
    stdin_open: true
    tty: true
```

### Alternative: Separate Containers

```yaml
version: '3.8'

services:
  rosbridge:
    image: ros:humble
    container_name: lad-rosbridge
    command: ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
    ports:
      - "9090:9090"
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0

  gazebo:
    image: osrf/ros:humble-desktop-full
    container_name: lad-gazebo
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        ros2 launch gazebo_ros gazebo.launch.py world:=/root/worlds/vehicle_world.world
      "
    network_mode: host
    privileged: true
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./worlds:/root/worlds:ro
      - ./models:/root/models:ro

  vehicle-controller:
    image: ros:humble
    container_name: lad-vehicle-controller
    command: ros2 run your_package vehicle_controller_node
    network_mode: host
    depends_on:
      - gazebo
    environment:
      - ROS_DOMAIN_ID=0
```

## Vehicle Model Setup

### Required ROS 2 Package Structure

```
vehicle_description/
├── launch/
│   └── vehicle_gazebo.launch.py
├── urdf/
│   └── vehicle.urdf.xacro
├── meshes/
│   ├── chassis.stl
│   └── wheel.stl
├── config/
│   └── gazebo_plugins.yaml
└── worlds/
    └── test_track.world
```

### Example Launch File

```python
# launch/vehicle_gazebo.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('vehicle_description')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'world': os.path.join(pkg_dir, 'worlds', 'test_track.world')}.items()
    )

    # Spawn vehicle
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'autonomous_vehicle',
            '-file', os.path.join(pkg_dir, 'urdf', 'vehicle.urdf'),
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(os.path.join(pkg_dir, 'urdf', 'vehicle.urdf')).read()}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        robot_state_publisher,
    ])
```

### Minimal URDF Example

```xml
<?xml version="1.0"?>
<robot name="autonomous_vehicle" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="2.0 1.0 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.5 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="2.0 1.0 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1500.0"/>
      <inertia ixx="100.0" ixy="0.0" ixz="0.0" iyy="100.0" iyz="0.0" izz="100.0"/>
    </inertial>
  </link>

  <!-- Camera -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.1 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="1.0 0.0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- Camera Sensor Plugin -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <update_rate>30.0</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/</namespace>
          <remapping>~/image_raw:=/camera/image_raw</remapping>
          <remapping>~/camera_info:=/camera/camera_info</remapping>
        </ros>
        <camera_name>camera</camera_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- LiDAR Sensor -->
  <link name="lidar_link"/>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.8 0.0 0.4" rpy="0 0 0"/>
  </joint>

  <gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">
      <update_rate>10.0</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.0</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/</namespace>
          <remapping>~/out:=/scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Differential Drive Plugin -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <update_rate>50.0</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>1.0</wheel_separation>
      <wheel_diameter>0.6</wheel_diameter>
      <max_wheel_torque>200.0</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>/cmd_vel</command_topic>
      <odometry_topic>/odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
    </plugin>
  </gazebo>

  <!-- IMU Plugin -->
  <gazebo reference="base_link">
    <sensor name="imu_sensor" type="imu">
      <update_rate>100.0</update_rate>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <namespace>/</namespace>
          <remapping>~/out:=/imu</remapping>
        </ros>
        <frame_name>base_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Starting the Simulation

### Method 1: Docker Compose

```bash
cd qcar_docker/  # or gazebo_docker/
docker-compose up
```

### Method 2: Manual Docker Commands

```bash
# Start rosbridge
docker run -it --rm --network host \
  -e ROS_DOMAIN_ID=0 \
  ros:humble \
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090

# Start Gazebo (in another terminal)
docker run -it --rm --network host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  osrf/ros:humble-desktop-full \
  bash -c "source /opt/ros/humble/setup.bash && ros2 launch gazebo_ros gazebo.launch.py"
```

### Method 3: Using Existing qcar_docker

If you already have a qcar_docker setup, you can extend it:

```bash
# In qcar_docker container
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/world.world &
# Spawn your vehicle model
ros2 run gazebo_ros spawn_entity.py -entity vehicle -file /path/to/vehicle.urdf
```

## Frontend Configuration

### Update config/ip_config.json

Make sure your rosbridge WebSocket URL is correct:

```json
{
  "frontend_ip": "192.168.1.100",
  "backend_ip": "192.168.1.100",
  "ros_ip": "192.168.1.100",
  "ros_ws_port": 9090
}
```

### Environment Variables

Ensure `.env.local` has:

```env
REACT_APP_ROS_WS=ws://192.168.1.100:9090
```

Or it will auto-detect from `window.location.host`.

## Database Setup

### Create Unit and Level

```python
from apps.learning.models import Unit, Level, Objective

# Create or get unit
unit, _ = Unit.objects.get_or_create(
    slug="autonomous-vehicles",
    defaults={
        "title": "Autonomous Vehicles",
        "order": 20,
        "is_active": True
    }
)

# Create level
level, _ = Level.objects.get_or_create(
    slug="gazebo-sim",
    defaults={
        "unit": unit,
        "title": "Gazebo Simulation & Control",
        "order": 1,
        "is_active": True
    }
)

# Create objectives
objectives_data = [
    ("gazebo-slide-intro", "Complete Introduction to Gazebo", 10),
    ("gazebo-slide-arch", "Understand Gazebo Architecture", 10),
    ("gazebo-slide-sensors", "Learn About Sensors and Control", 10),
    ("gazebo-first-drive", "Drive the vehicle for the first time", 15),
    ("gazebo-drive-10s", "Drive for 10 seconds", 15),
    ("gazebo-drive-30s", "Drive for 30 seconds", 20),
]

for code, desc, points in objectives_data:
    Objective.objects.get_or_create(
        code=code,
        defaults={
            "level": level,
            "description": desc,
            "points": points
        }
    )

print("✓ Gazebo Sim level created!")
```

## Testing

### 1. Start Backend
```bash
cd LAD/lad
python manage.py runserver
```

### 2. Start Docker/Gazebo
```bash
cd qcar_docker
docker-compose up
```

### 3. Start Frontend
```bash
cd AVEDU/avedu
npm start
```

### 4. Access Level
Navigate to: `http://localhost:3000/learn/autonomous-vehicles/gazebo-sim`

## Troubleshooting

### Issue: "Connecting to ROS..." forever
**Solutions:**
- Check if rosbridge is running: `docker ps`
- Verify rosbridge port: `netstat -an | grep 9090`
- Check browser console for WebSocket errors
- Ensure `ROS_DOMAIN_ID` matches across all containers

### Issue: No camera feed
**Solutions:**
- Check if `/camera/image_raw` topic exists: `ros2 topic list`
- Verify camera plugin in URDF
- Check topic name in Gazebo plugin configuration
- Ensure image encoding is RGB8 or BGR8

### Issue: Vehicle doesn't move
**Solutions:**
- Check `/cmd_vel` topic: `ros2 topic echo /cmd_vel`
- Verify differential drive plugin in URDF
- Check wheel joint names match in plugin config
- Ensure vehicle has proper physics properties (mass, inertia)

### Issue: Keyboard doesn't work
**Solutions:**
- Make sure simulator view is active (not slides)
- Check browser console for JavaScript errors
- Click inside the simulator area to focus
- Try refreshing the page

## ROS Topics Summary

| Topic | Type | Purpose |
|-------|------|---------|
| `/cmd_vel` | geometry_msgs/Twist | Vehicle control input |
| `/camera/image_raw` | sensor_msgs/Image | Front camera feed |
| `/scan` | sensor_msgs/LaserScan | LiDAR distance measurements |
| `/odom` | nav_msgs/Odometry | Vehicle position and velocity |
| `/imu` | sensor_msgs/Imu | Acceleration and angular velocity |

## Future Enhancements

### Potential Additions:
1. **Multiple camera views** (rear, side cameras)
2. **LiDAR visualization** (point cloud or 2D plot)
3. **Path planning interface** (click to set waypoints)
4. **Obstacle detection display** (bounding boxes on camera)
5. **Recording functionality** (save rosbag for replay)
6. **Performance metrics** (lap times, collision detection)
7. **Multiple vehicles** (multi-agent simulation)
8. **Weather conditions** (rain, fog effects in Gazebo)

## Summary

You now have a complete Gazebo simulation level with:
- ✓ Educational intro slides
- ✓ Real-time camera feed from Gazebo
- ✓ Keyboard control interface
- ✓ Sensor data visualization (IMU, LiDAR, odometry)
- ✓ ROS 2 integration via rosbridge
- ✓ Docker-ready configuration
- ✓ Progress tracking and objectives

Access it at: `/learn/autonomous-vehicles/gazebo-sim`

Start simulating! 🚗🤖

# Unity Integration with ROS 2 Docker

This document explains how to connect Unity to the QCar ROS 2 Docker environment using ROS-TCP-Endpoint.

## What Was Added

### Docker Configuration

1. **ROS-TCP-Endpoint Package**: Added to `Dockerfile` - cloned from Unity's official repository
2. **Port Exposure**: Port `10000` exposed for Unity communication
3. **Environment Variables**:
   - `ENABLE_UNITY_TCP=1` - Enable/disable Unity TCP endpoint
   - `UNITY_TCP_PORT=10000` - Port for Unity connection (customizable)
4. **Entrypoint Script**: Automatically starts ROS-TCP-Endpoint when `ENABLE_UNITY_TCP=1`

### Files Modified

- `Dockerfile` - Added ROS-TCP-Endpoint clone and port exposure
- `docker-compose.yml` - Added port mapping and environment variables
- `entrypoint.sh` - Added Unity TCP endpoint startup logic

## Building and Running

### 1. Rebuild Docker Container

Since we modified the Dockerfile, you need to rebuild:

```bash
cd qcar_docker
docker compose down
docker compose build
docker compose up -d
```

### 2. Verify ROS-TCP-Endpoint is Running

Check the logs to confirm the endpoint started:

```bash
docker compose logs ros | grep -i "tcp"
```

You should see output like:
```
[entrypoint] Starting ROS-TCP-Endpoint for Unity on port 10000...
[entrypoint] ROS-TCP-Endpoint started in background
```

### 3. Test Connection from Command Line

You can test if the endpoint is accessible:

```bash
# From Windows (PowerShell or CMD)
Test-NetConnection -ComputerName localhost -Port 10000

# Or using telnet
telnet localhost 10000
```

## Unity Setup (Next Steps)

### Install Unity Packages

In your Unity project:

1. Open **Window → Package Manager**
2. Click **+ → Add package from git URL**
3. Add the ROS-TCP-Connector:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   ```
4. Add the URDF Importer:
   ```
   https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
   ```

### Configure ROS Settings in Unity

1. Go to **Robotics → ROS Settings**
2. Set the following:
   - **ROS IP Address**: Your Docker host IP (from `config/ip_config.json`, currently `192.168.100.119`)
   - **ROS Port**: `10000`
   - **Protocol**: `ROS 2`
   - **Serializer**: Leave default

### Import QCar URDF

1. In Unity, go to **Robotics → Import Robot from URDF**
2. The URDF is available at: `http://192.168.100.119:7000/qcar_description/urdf/robot_runtime.urdf`
3. You can download it first or import directly if Unity supports URLs
4. Unity will create a GameObject hierarchy matching the robot structure

## Available ROS Topics

Once connected, you can subscribe/publish to these topics from Unity:

### Control Topics (Publish from Unity)
- `/qcar/cmd_vel` (geometry_msgs/Twist) - Robot velocity commands

### Sensor Topics (Subscribe in Unity)
- `/qcar/odom` (nav_msgs/Odometry) - Robot odometry
- `/qcar/lidar/scan` (sensor_msgs/LaserScan) - LIDAR data
- `/qcar/rgb/image_raw` (sensor_msgs/Image) - RGB camera
- `/qcar/csi_front/image_raw` - Front CSI camera
- `/qcar/csi_right/image_raw` - Right CSI camera
- `/qcar/csi_back/image_raw` - Back CSI camera
- `/qcar/csi_left/image_raw` - Left CSI camera

**Note**: For better performance with camera images, use compressed topics:
- `/qcar/rgb/image_raw/compressed` (sensor_msgs/CompressedImage)

## Example Unity C# Scripts

### Subscribe to Odometry

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

public class QCarOdometrySubscriber : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<OdometryMsg>("/qcar/odom", OdometryCallback);
    }

    void OdometryCallback(OdometryMsg msg)
    {
        // Update Unity GameObject position based on ROS odometry
        // Note: Coordinate conversion needed (ROS Z-up to Unity Y-up)
        Vector3 position = new Vector3(
            (float)msg.pose.pose.position.x,
            (float)msg.pose.pose.position.z,
            (float)msg.pose.pose.position.y
        );

        transform.position = position;
    }
}
```

### Publish Velocity Commands

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class QCarController : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/qcar/cmd_vel";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        // Get input from keyboard
        float linear = Input.GetAxis("Vertical") * 1.0f;   // W/S or Up/Down
        float angular = Input.GetAxis("Horizontal") * 1.0f; // A/D or Left/Right

        // Create and publish Twist message
        TwistMsg msg = new TwistMsg
        {
            linear = new Vector3Msg { x = linear, y = 0, z = 0 },
            angular = new Vector3Msg { x = 0, y = 0, z = angular }
        };

        ros.Publish(topicName, msg);
    }
}
```

## Troubleshooting

### Connection Issues

**Problem**: Unity can't connect to ROS

**Solutions**:
1. Check that Docker container is running: `docker compose ps`
2. Verify port 10000 is exposed: `docker compose port ros 10000`
3. Check firewall settings - Windows Firewall might block port 10000
4. Verify IP address matches your Docker host IP

### ROS-TCP-Endpoint Not Starting

**Problem**: Logs don't show TCP endpoint startup

**Solutions**:
1. Ensure `ENABLE_UNITY_TCP=1` in `docker-compose.yml`
2. Rebuild container: `docker compose build`
3. Check for build errors: `docker compose logs ros`

### Message Type Errors

**Problem**: Unity complains about missing message types

**Solutions**:
1. In Unity, go to **Robotics → Generate ROS Messages**
2. Point to your ROS package source or msg files
3. Generate C# classes for custom messages

### Performance Issues

**Problem**: Camera images cause lag in Unity

**Solutions**:
1. Use compressed image topics instead of raw
2. Reduce image resolution in Gazebo camera plugins
3. Lower update rate using ROS topic throttling

## Coordinate System Differences

**Important**: ROS and Unity use different coordinate systems:

- **ROS**: Right-handed, Z-up (X-forward, Y-left, Z-up)
- **Unity**: Left-handed, Y-up (X-right, Y-up, Z-forward)

**Conversion for positions**:
```csharp
// ROS (x, y, z) → Unity (x, z, y)
Vector3 unityPos = new Vector3(rosX, rosZ, rosY);
```

**Conversion for rotations**:
```csharp
// ROS Quaternion (x, y, z, w) → Unity Quaternion
Quaternion unityRot = new Quaternion(-rosX, -rosZ, -rosY, rosW);
```

## Next Steps

1. **Step 3**: Import QCar URDF into Unity (see main README)
2. **Step 4**: Configure Unity-ROS connection settings
3. **Step 5**: Create Unity scripts for ROS topic communication
4. **Step 6**: Test bidirectional communication

## Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector Documentation](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [Unity URDF Importer](https://github.com/Unity-Technologies/URDF-Importer)
- [ROS 2 Unity Tutorial](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example)

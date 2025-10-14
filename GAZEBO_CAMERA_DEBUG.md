# Gazebo Camera Debugging Guide

## Problem: Connected to ROS but no camera feed

You're successfully connected (subscriptions work), but the camera isn't displaying. Here's how to debug:

## Quick Start Checklist

**Before diving into detailed debugging, try these quick fixes:**

1. **Open the Debug Panel** (top-left, click to expand)
   - Check if camera message count is increasing
   - If count = 0: Camera not publishing (see Step 2 below)
   - If count > 0: Rendering issue (see Step 3 below)

2. **Try different camera tabs**
   - Click "Front (Compressed)" instead of "Front Camera"
   - Try "World Camera" as alternative
   - Click "Grid View" to test all cameras at once

3. **Check browser console** (Press F12)
   - Look for `[CameraFeed]` messages
   - Errors will show data format/encoding issues

4. **Verify in Docker**
   ```bash
   ros2 topic hz /camera/image_raw
   # Should show ~30 Hz if working
   ```

## New Features Added

### 1. ROS Debug Panel
- **Location:** Top of left sensor panel
- **Click to expand** - Shows real-time statistics for all topics
- **Features:**
  - Message counts for each topic
  - Last message details (resolution, encoding, data format)
  - Warning if no messages received
  - Color-coded status (red/orange/green)

### 2. Camera Selector
- **Multiple camera tabs** - Switch between different camera views (raw and compressed)
- **Grid View button** - See all cameras at once
- **Auto-detection** - Automatically tries both raw and compressed formats
- **Better error messages** - Shows exactly what's wrong

### 3. Enhanced Camera Feed
- **Console logging** - Check browser console (F12) for detailed info
- **Better encoding support** - RGB8, BGR8, RGBA8, BGRA8, MONO8, MONO16
- **Compressed image support** - JPEG and PNG compressed streams
- **Format auto-detection** - Shows whether using raw or compressed format
- **Error display** - Shows specific error messages in the camera view with troubleshooting hints

## Step-by-Step Debugging

### Step 1: Check ROS Debug Panel

1. **Open the simulator**
2. **Expand the Debug Panel** (click the header)
3. **Check camera message count:**
   - **Red (0 msgs):** No data being published → Go to Step 2
   - **Orange (1-10 msgs):** Low rate, might work → Go to Step 3
   - **Green (10+ msgs):** Data flowing → Go to Step 3

### Step 2: Verify Camera is Publishing (Docker)

If message count is 0, the camera isn't publishing. Check in Docker:

```bash
# Enter your ROS container
docker exec -it lad-gazebo-sim bash

# List all topics
ros2 topic list

# Expected output should include:
# /camera/image_raw
# /world_camera/image_raw (if configured)

# Check if camera topic exists
ros2 topic info /camera/image_raw

# Check message rate
ros2 topic hz /camera/image_raw
# Should show ~30 Hz if working

# Echo one message to see structure
ros2 topic echo /camera/image_raw --once
```

**If topic doesn't exist:**
- Camera sensor not configured in URDF
- Gazebo plugin not loaded
- Camera node not running

**Fix:** Add camera sensor to your URDF (see GAZEBO_SETUP.md)

### Step 3: Check Image Data Format

If messages are flowing but no image, check the debug panel details:

**Look for:**
- `width x height` - Should be like 640x480
- `encoding` - Should be rgb8, bgr8, rgba8, or mono8
- `dataType` - Should be "string" (base64) or "object" (array)
- `dataLength` - Should match: width × height × bytes_per_pixel

**Common Issues:**

#### Issue: `dataLength` is 0 or very small
**Cause:** Camera plugin not configured to send data
**Fix:** Check Gazebo camera plugin config:

```xml
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
  <ros>
    <namespace>/</namespace>
    <remapping>~/image_raw:=/camera/image_raw</remapping>
  </ros>
  <camera_name>camera</camera_name>
  <frame_name>camera_link</frame_name>
  <hack_baseline>0.07</hack_baseline>
</plugin>
```

#### Issue: Encoding is "compressed" or unknown
**Cause:** Using compressed image topic
**Fix:** Subscribe to uncompressed topic:
- Use `/camera/image_raw` NOT `/camera/image_raw/compressed`
- Or modify camera plugin to publish uncompressed

#### Issue: `dataType` is "undefined"
**Cause:** Message structure mismatch
**Fix:** Check rosbridge encoding settings

### Step 4: Browser Console Debugging

Open browser console (F12) and look for:

```
[CameraFeed] Subscribing to /camera/image_raw
[CameraFeed] Received image: 640x480, encoding: rgb8, data length: 921600
[CameraFeed] Using array data: 921600 bytes
[CameraFeed] Canvas resized to 640x480
```

**Common Console Errors:**

#### Error: "Failed to decode base64"
**Cause:** Data is not properly base64 encoded
**Solution:** Check rosbridge configuration, ensure proper encoding

#### Error: "Data size mismatch"
**Cause:** Image data incomplete or wrong size
**Solution:**
- Check camera resolution in URDF matches what's published
- Verify network isn't dropping packets
- Try lower resolution (320x240)

#### Error: "Unknown image data format"
**Cause:** Unexpected data type from rosbridge
**Solution:** Check rosbridge WebSocket settings

### Step 5: Test with Simple Gazebo World

Create a minimal test to isolate the issue:

```bash
# In Docker container
source /opt/ros/humble/setup.bash

# Start just Gazebo with a simple camera
ros2 launch gazebo_ros gazebo.launch.py

# In Gazebo GUI: Insert -> Camera
# Right-click camera -> View Image

# Publish test image
ros2 run image_publisher image_publisher_node \
  --ros-args -p filename:=/path/to/test/image.jpg
```

## Quick Fixes

### Fix 1: Try Compressed Image Topic

If `/camera/image_raw` has issues, try compressed:

**In the Camera Selector tabs**, you'll now see both:
- "Front Camera" - Raw image topic (`/camera/image_raw`)
- "Front (Compressed)" - Compressed image topic (`/camera/image_raw/compressed`)

Simply click the compressed tab to test if it works better. Compressed images:
- Use less bandwidth (better for slow networks)
- Are easier to debug (standard JPEG/PNG format)
- May work even if raw image has encoding issues

**The camera feed now automatically detects and handles both formats!**

### Fix 2: Lower Camera Resolution

In your URDF camera sensor:

```xml
<camera>
  <horizontal_fov>1.047</horizontal_fov>
  <image>
    <width>320</width>  <!-- Lower from 640 -->
    <height>240</height> <!-- Lower from 480 -->
    <format>R8G8B8</format>
  </image>
  <clip>
    <near>0.1</near>
    <far>100.0</far>
  </clip>
</camera>
```

### Fix 3: Add World/Overhead Camera

This camera is easier to debug because it's fixed in the world:

```xml
<!-- Add to your .world file -->
<model name="world_camera">
  <static>true</static>
  <pose>0 0 10 0 1.57 0</pose>
  <link name="link">
    <sensor name="world_cam" type="camera">
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
      <plugin name="world_camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/</namespace>
          <remapping>~/image_raw:=/world_camera/image_raw</remapping>
        </ros>
        <camera_name>world_camera</camera_name>
        <frame_name>world</frame_name>
      </plugin>
    </sensor>
  </link>
</model>
```

### Fix 4: Check rosbridge Configuration

Your rosbridge might need specific settings:

```bash
# Launch with explicit settings
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  port:=9090 \
  websocket_ping_interval:=0 \
  websocket_ping_timeout:=30 \
  unregister_timeout:=10.0 \
  fragment_timeout:=600 \
  delay_between_messages:=0 \
  max_message_size:=10000000
```

## Expected Browser Console Output (Working)

When everything works, you should see:

```
[roslib] Connected to rosbridge
[CameraFeed] Subscribing to /camera/image_raw
[CameraFeed] Received image: 640x480, encoding: rgb8, data length: 921600
[CameraFeed] Using array data: 921600 bytes
[CameraFeed] Canvas resized to 640x480
(Repeated every ~33ms / 30 FPS)
```

## Useful ROS Commands

```bash
# List all topics
ros2 topic list

# Get topic info
ros2 topic info /camera/image_raw

# Check message type
ros2 topic type /camera/image_raw

# Check frequency
ros2 topic hz /camera/image_raw

# See one message
ros2 topic echo /camera/image_raw --once

# Check bandwidth
ros2 topic bw /camera/image_raw

# See all camera topics
ros2 topic list | grep camera

# Test publish to cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Still Not Working?

### Check These:

1. **Gazebo is actually running** - You should see the Gazebo window
2. **Vehicle is spawned** - You should see your vehicle in Gazebo
3. **Camera link exists** - Check TF tree: `ros2 run tf2_tools view_frames`
4. **Network** - Try `localhost` instead of IP address
5. **Firewall** - Disable temporarily to test
6. **rosbridge version** - Try updating: `apt update && apt upgrade ros-humble-rosbridge-*`

### Test Without Frontend:

```bash
# In Docker
ros2 topic echo /camera/image_raw --once

# Should output image data
# If this works, problem is in frontend/rosbridge
# If this doesn't work, problem is in Gazebo/camera plugin
```

## Summary Checklist

- [ ] ROS Debug Panel shows message count > 0 for camera
- [ ] Debug panel shows correct resolution (e.g., 640x480)
- [ ] Encoding is rgb8, bgr8, or similar (not "compressed")
- [ ] Data length matches expected size (width × height × 3)
- [ ] Browser console shows "Received image" messages
- [ ] No errors in browser console
- [ ] `ros2 topic hz /camera/image_raw` shows ~30 Hz
- [ ] Camera sensor configured in URDF with plugin
- [ ] rosbridge is running on correct port (9090)

---

Use the new Debug Panel and camera tabs to diagnose! The panel will tell you exactly what's wrong. 🔧

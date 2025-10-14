# Gazebo Camera Features - Ready to Test! 🎥

## What's New

Your Gazebo simulator now has comprehensive camera debugging and visualization tools. Here's what's been added to help you diagnose and fix the camera feed issue.

## New Features Overview

### 1. ROS Debug Panel 🔧

**Location:** Top of the left sensor panel

**How to use:**
1. Click the header to expand/collapse
2. Shows real-time statistics for ALL topics:
   - `/camera/image_raw` - Front camera
   - `/odom` - Position data
   - `/imu` - Inertial data
   - `/scan` - LiDAR data

**What it shows:**
- **Message count** - Color-coded (red/orange/green)
- **Last message details** - Resolution, encoding, data type, data length
- **Warnings** - If no messages received

**Color codes:**
- 🔴 Red (0 msgs) - No data being published
- 🟠 Orange (1-10 msgs) - Low rate, might have issues
- 🟢 Green (10+ msgs) - Data flowing normally

### 2. Multi-Camera Selector 📹

**Location:** Top of camera feed area

**Camera options:**
1. **Front Camera** - `/camera/image_raw` (raw format)
2. **Front (Compressed)** - `/camera/image_raw/compressed` (JPEG/PNG)
3. **World Camera** - `/world_camera/image_raw` (overhead view)
4. **World (Compressed)** - `/world_camera/image_raw/compressed`
5. **Rear Camera** - `/rear_camera/image_raw`
6. **Top Camera** - `/top_camera/image_raw`

**Features:**
- **Tab switching** - Click any camera tab to switch views
- **Grid View button** - See all cameras simultaneously
- **Format indicator** - Shows if using raw or compressed format

### 3. Universal Camera Feed 🎬

**Automatically handles:**
- Raw images (RGB8, BGR8, RGBA8, BGRA8, MONO8, MONO16)
- Compressed images (JPEG, PNG)
- Base64 encoding
- Array data format
- Uint8Array format

**Error handling:**
- Shows specific error messages
- Provides troubleshooting hints inline
- Console logging for detailed debugging

**Status bar shows:**
- Topic name
- Resolution (e.g., 640x480)
- Encoding format
- Message count
- FPS (frames per second)
- Format type (raw/compressed)

## How to Debug Your Camera Issue

### Step 1: Open Debug Panel

1. Launch your Gazebo simulation
2. Look at top-left panel
3. Click "🔧 ROS Debug Panel" header to expand
4. Check the camera message count

**If message count is 0:**
- Camera is not publishing in Gazebo
- Check your URDF has camera sensor configured
- Verify Gazebo is actually running
- See GAZEBO_CAMERA_DEBUG.md for detailed steps

**If message count is increasing:**
- Camera is publishing, but rendering has issues
- Look at the details: resolution, encoding, data length
- Try switching to compressed format (see Step 2)

### Step 2: Try Different Camera Options

**Click through the camera tabs:**

1. Start with "Front (Compressed)"
   - Compressed images often work better
   - Use less bandwidth
   - Easier to debug

2. Try "World Camera"
   - Fixed overhead view
   - Simpler to configure
   - Good for testing if camera system works

3. Use "Grid View"
   - See all cameras at once
   - Quickly identify which cameras are working
   - Compare different formats side-by-side

### Step 3: Check Browser Console

1. Press **F12** to open developer tools
2. Click **Console** tab
3. Look for messages starting with `[CameraFeed]`

**Expected output (working camera):**
```
[CameraFeed] Subscribing to /camera/image_raw
[CameraFeed] Received image: 640x480, encoding: rgb8, data length: 921600
[CameraFeed] Using array data: 921600 bytes
[CameraFeed] Canvas resized to 640x480
```

**Look for errors:**
- "Failed to decode base64" - Data encoding issue
- "Data size mismatch" - Image data incomplete
- "Unknown encoding" - Unsupported format

### Step 4: Verify in Docker

Open terminal in your Docker container:

```bash
# Check if topic exists
ros2 topic list | grep camera

# Check message rate (should be ~30 Hz)
ros2 topic hz /camera/image_raw

# See one message to verify data
ros2 topic echo /camera/image_raw --once

# Check compressed topic
ros2 topic hz /camera/image_raw/compressed
```

## Common Issues and Solutions

### Issue: "No messages received" in Debug Panel

**Cause:** Camera not publishing in Gazebo

**Solution:**
1. Check camera sensor in your URDF
2. Verify Gazebo camera plugin is loaded
3. Check vehicle is spawned in Gazebo
4. See GAZEBO_SETUP.md for camera configuration

### Issue: Debug Panel shows messages but no image

**Cause:** Rendering or encoding issue

**Solutions:**
1. Try compressed format tab
2. Check console for specific error
3. Verify encoding is supported (see debug panel details)
4. Check data length matches expected size

### Issue: Low FPS or stuttering

**Cause:** Network or performance issue

**Solutions:**
1. Use compressed format (less bandwidth)
2. Lower camera resolution in URDF
3. Check Docker container resources
4. Reduce throttle_rate in camera feed

### Issue: Some cameras work, others don't

**Cause:** Some cameras not configured in Gazebo

**Solution:**
- Working cameras: Use these!
- Non-working cameras: Need to add sensors to URDF/world file
- See GAZEBO_SETUP.md for camera configuration examples

## Testing Your Setup

### Quick Test Procedure

1. **Launch simulation**
   ```bash
   # In your Docker container
   ros2 launch your_package gazebo.launch.py
   ```

2. **Open simulator in browser**
   - Navigate to Gazebo Sim level
   - Click "Launch Simulation"

3. **Check connection**
   - Status should show "ROS Connected"
   - Debug panel should appear

4. **Expand debug panel**
   - Click the header
   - Watch message counts

5. **Try camera tabs**
   - Click "Front (Compressed)" first
   - Then try "World Camera"
   - Use "Grid View" to test all

6. **Check browser console**
   - Press F12
   - Look for camera feed messages

### Expected Results

**✅ Everything working:**
- Debug panel shows green (10+ msgs) for camera
- Camera feed displays image
- FPS shows 20-30 FPS
- Console shows "Received image" messages

**⚠️ Partially working:**
- Debug panel shows orange (1-10 msgs)
- Image might be choppy or slow
- Try compressed format
- Consider lowering resolution

**❌ Not working:**
- Debug panel shows red (0 msgs)
- No image displayed
- Follow Step 2 in GAZEBO_CAMERA_DEBUG.md
- Verify camera configuration in Gazebo

## What to Check in Your Code

### Your URDF should have camera sensor:

```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/</namespace>
      <remapping>~/image_raw:=/camera/image_raw</remapping>
    </ros>
    <camera_name>camera</camera_name>
    <frame_name>camera_link</frame_name>
  </plugin>
</sensor>
```

### Your Docker setup should have:

- ROS 2 Humble
- Gazebo (Garden or Classic with ROS 2 plugins)
- rosbridge_server running on port 9090
- Camera plugins installed

## Files Modified

### New Files Created:
- `components/gazebo/CameraFeedUniversal.jsx` - Universal camera feed handler
- `components/gazebo/ROSDebugPanel.jsx` - ROS topic debug panel
- `GAZEBO_CAMERA_FEATURES.md` - This file!

### Modified Files:
- `components/gazebo/CameraSelector.jsx` - Added compressed format options
- `components/gazebo/GazeboSimulator.jsx` - Integrated debug panel
- `styles/components/_gazebo.scss` - Styling for new components
- `GAZEBO_CAMERA_DEBUG.md` - Updated with new features

## Next Steps

1. **Test the simulator**
   - Launch your Docker setup
   - Open the simulator
   - Try the debug tools

2. **Check the results**
   - If camera works: Great! You're done.
   - If not: Follow GAZEBO_CAMERA_DEBUG.md step-by-step

3. **Report back**
   - What does the Debug Panel show?
   - What's the message count for camera?
   - Any errors in browser console?
   - Which camera format works (raw/compressed)?

## Summary

You now have:
- ✅ Real-time ROS topic monitoring
- ✅ Multiple camera view options
- ✅ Both raw and compressed format support
- ✅ Automatic format detection
- ✅ Grid view for testing multiple cameras
- ✅ Comprehensive error messages
- ✅ Detailed console logging
- ✅ Step-by-step debugging guide

**The debug tools will tell you exactly what's wrong!** 🎯

Open the Debug Panel, check the message count, and you'll immediately know if the problem is:
1. Camera not publishing (count = 0)
2. Wrong encoding/format (count > 0 but no image)
3. Network/bandwidth issue (low FPS)

Good luck! The camera feed should work now, and if not, you have all the tools to diagnose why. 🚀

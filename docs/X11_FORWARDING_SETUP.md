# X11 Forwarding Setup for Windows (VcXsrv)

## Step-by-Step Guide

### Step 1: Install VcXsrv (Windows X Server)

1. **Download VcXsrv**:
   - Go to: https://sourceforge.net/projects/vcxsrv/
   - Download the latest installer (vcxsrv-x.x.x.x.installer.exe)
   - Run the installer and follow the prompts

2. **Launch XLaunch** (VcXsrv configuration):
   - Find "XLaunch" in your Start Menu
   - Or run: `C:\Program Files\VcXsrv\xlaunch.exe`

3. **Configure XLaunch** (IMPORTANT - follow exactly):

   **Screen 1: Display settings**
   - Select: **"Multiple windows"**
   - Display number: **0**
   - Click "Next"

   **Screen 2: Client startup**
   - Select: **"Start no client"**
   - Click "Next"

   **Screen 3: Extra settings** (CRITICAL!)
   - ☑ **Check "Disable access control"** (REQUIRED!)
   - ☐ Uncheck "Clipboard" (optional, but can cause issues)
   - ☑ Check "Native opengl" (REQUIRED for GPU!)
   - ☐ Uncheck "Primary Selection" (optional)
   - Click "Next"

   **Screen 4: Finish**
   - Optionally save configuration for future use
   - Click "Finish"

4. **Verify VcXsrv is Running**:
   - Look for **"VcXsrv"** icon in system tray (bottom-right)
   - Should see an "X" icon
   - If not running, launch XLaunch again

**IMPORTANT**: VcXsrv must be running BEFORE starting Docker containers!

---

### Step 2: Get Your Windows IP Address

We need your Windows host IP address for Docker to connect to VcXsrv.

**Open PowerShell or Command Prompt** and run:

```powershell
ipconfig
```

Look for your **main network adapter** (usually "Ethernet" or "Wi-Fi"):
```
Ethernet adapter Ethernet:
   IPv4 Address. . . . . . . . . . . : 192.168.1.100  ← THIS ONE
```

**Or in WSL2 terminal**:
```bash
cat /etc/resolv.conf | grep nameserver | awk '{print $2}'
```

This gives you the Windows host IP as seen from WSL2.

**Note this IP address** - we'll use it as `WINDOWS_HOST_IP` below.

---

### Step 3: Update docker-compose.yml

Now we'll configure Docker to use X11 forwarding.

**Edit `qcar_docker/docker-compose.yml`:**

```yaml
services:
  ros:
    build: .
    image: qcar_docker-ros

    # GPU acceleration for Gazebo rendering
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu, compute, utility]

    ports:
      - "9090:9090"  # rosbridge
      - "7000:7000"  # static (URDF/meshes)
      - "8080:8080"  # WVS
      - "11345:11345" # Gazebo master

    environment:
      # NVIDIA GPU environment variables
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all

      # X11 forwarding to Windows VcXsrv
      - DISPLAY=WINDOWS_HOST_IP:0.0  # ← REPLACE with your Windows IP
      - LIBGL_ALWAYS_INDIRECT=0      # Use direct rendering (GPU)
      - QT_X11_NO_MITSHM=1           # Fix Qt X11 issues

      # ROS/Gazebo configuration
      - CORS_ALLOW_ORIGIN=http://localhost:3000,http://127.0.0.1:3000,http://${EXPOSED_IP:-192.168.56.1}:3000
      - ENABLE_JSP=1
      - ENABLE_ROSAPI=1
      - ENABLE_WVS=1
      - ENABLE_TURTLESIM=1
      - ENABLE_GAZEBO=1
      - ENABLE_DUMMY_CAMERAS=0       # Disable - use real Gazebo cameras
      - GAZEBO_WORLD=/ros2_ws/src/qcar_bringup/worlds/simple_track.world
      - IP_CONFIG_PATH=/config/ip_config.json

    volumes:
      - ../config/ip_config.json:/config/ip_config.json:ro
```

**REPLACE `WINDOWS_HOST_IP`** with your actual Windows IP from Step 2.

**Example**:
If your Windows IP is `192.168.1.100`, the line should be:
```yaml
- DISPLAY=192.168.1.100:0.0
```

---

### Step 4: Restart Docker Container

```bash
cd qcar_docker
docker compose down
docker compose up -d
```

Wait 30-40 seconds for Gazebo to initialize.

---

### Step 5: Test X11 Connection

**Test if X11 is working:**

```bash
# Test basic X11 app
docker compose exec ros bash -c "xeyes"
```

**Expected**: A window with eyes should appear on your Windows desktop!
- If you see the eyes window → X11 is working! ✅
- If error "cannot open display" → Check VcXsrv is running and DISPLAY variable is correct ❌

**Close the eyes window** (Ctrl+C in terminal) before continuing.

---

### Step 6: Verify Gazebo Camera Rendering

**Check if Gazebo cameras are rendering:**

```bash
# Check for camera sensor errors
docker compose logs ros | grep -i "CameraSensor"
```

**Expected**: NO "Unable to create CameraSensor" errors

**Check camera publishers:**

```bash
docker compose exec ros bash -c "source /opt/ros/humble/setup.bash && ros2 topic info /qcar/rgb/image_color"
```

**Expected**: `Publisher count: 1`

**Check camera publishing rate:**

```bash
docker compose exec ros bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic hz /qcar/rgb/image_color"
```

**Expected**: Rate of 10-20 Hz (slower than GPU but faster than broken)

---

### Step 7: Test in React Application

1. **Start your React app** (if not already running):
   ```bash
   cd AVEDU/avedu
   npm start
   ```

2. **Navigate to Gazebo simulation level**

3. **Check camera feeds**:
   - Should see REAL simulation views (not test patterns)
   - Overhead: Bird's eye view of QCar and world
   - RGB: Forward view from robot
   - CSI cameras: Ground-level fisheye views

4. **Drive the robot**:
   - Use WASD or arrow keys
   - Camera views should update showing movement

---

## Troubleshooting

### Issue: "cannot open display: WINDOWS_IP:0.0"

**Causes**:
1. VcXsrv not running
2. Wrong Windows IP address
3. Firewall blocking connection

**Solutions**:

**Check 1: VcXsrv running?**
- Look for "X" icon in Windows system tray
- If not there, launch XLaunch again

**Check 2: Correct IP?**
```bash
# In WSL2, get Windows host IP
cat /etc/resolv.conf | grep nameserver | awk '{print $2}'
# Use this IP in docker-compose.yml
```

**Check 3: Firewall**
- Windows Firewall might be blocking X11 (port 6000)
- Try temporarily disabling firewall to test
- Or add firewall rule to allow VcXsrv

---

### Issue: "Cameras still show 'Unable to create CameraSensor'"

**Cause**: X11 connection works but OpenGL not available

**Solution**:
Check "Native opengl" was enabled in XLaunch settings. If not:
1. Close VcXsrv (right-click tray icon → Exit)
2. Launch XLaunch again
3. On "Extra settings" screen: ☑ Check "Native opengl"
4. Finish and restart Docker: `docker compose restart`

---

### Issue: "Connection refused" or timeout

**Cause**: Windows Firewall blocking port 6000

**Solution 1 - Add Firewall Rule** (Recommended):
```powershell
# Run in PowerShell as Administrator
New-NetFirewallRule -DisplayName "VcXsrv" -Direction Inbound -Program "C:\Program Files\VcXsrv\vcxsrv.exe" -Action Allow
```

**Solution 2 - Temporary Test** (Not secure):
- Open Windows Defender Firewall
- Turn off "Private network" firewall temporarily
- Test if X11 works
- Turn firewall back on and add proper rule

---

### Issue: Gazebo starts but cameras still don't render

**Check GPU is being used:**

```bash
docker compose exec ros nvidia-smi
```

Should show `gzserver` in the processes list using GPU memory.

**If gzserver NOT in the list:**

Check `LIBGL_ALWAYS_INDIRECT`:
```yaml
environment:
  - LIBGL_ALWAYS_INDIRECT=0  # Should be 0 for direct rendering
```

---

### Issue: Performance is very slow (< 5 FPS)

**Expected**: X11 forwarding is slower than native (~10-20 FPS typical)

**Improvements**:
1. **Close other programs** using GPU
2. **Reduce camera resolution** in gazebo.xacro (e.g., 640x480 → 320x240)
3. **Use compressed topics** (already configured)
4. **Consider native Linux** for better performance long-term

---

## Performance Expectations

| Metric | X11 Forwarding | Native GPU | Software (Xvfb) |
|--------|----------------|------------|-----------------|
| Camera FPS | 10-20 FPS | 30-60 FPS | 0-5 FPS (broken) |
| Latency | ~50-100ms | ~16ms | N/A |
| GPU Usage | Yes | Yes | No |
| Reliability | Medium | High | Low |
| Setup Difficulty | Moderate | Easy (on Linux) | Easy but broken |

---

## Save XLaunch Configuration

To avoid reconfiguring XLaunch every time:

1. Run XLaunch once with correct settings
2. On final screen, click **"Save configuration"**
3. Save as `X11-for-Docker.xlaunch` on your desktop
4. **Next time**: Just double-click the .xlaunch file!

---

## Auto-start VcXsrv with Windows (Optional)

**Create a batch file**:

1. Open Notepad
2. Paste:
   ```batch
   @echo off
   "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -multiwindow -clipboard -wgl -ac
   ```
3. Save as `start-vcxsrv.bat`
4. Place in Windows Startup folder:
   - Press `Win+R`
   - Type: `shell:startup`
   - Put the .bat file there

Now VcXsrv starts automatically when Windows boots!

---

## Quick Reference: XLaunch Settings

```
Display: Multiple windows, Display 0
Client: Start no client
Extra settings:
  ☑ Disable access control  (REQUIRED!)
  ☑ Native opengl           (REQUIRED!)
  ☐ Clipboard               (Optional)
  ☐ Primary Selection       (Optional)
```

---

## Success Checklist

Before testing in React app:

- [ ] VcXsrv running (X icon in system tray)
- [ ] Windows IP address noted
- [ ] `docker-compose.yml` updated with correct DISPLAY
- [ ] Docker container restarted
- [ ] `xeyes` test works (eyes window appears)
- [ ] No "CameraSensor" errors in logs
- [ ] Camera topics show `Publisher count: 1`
- [ ] `ros2 topic hz` shows 10-20 Hz rate

If all checked → You should have real Gazebo cameras! 🎉

---

## Next Steps After Success

1. **Test all camera views** in your React app
2. **Drive the robot** and watch real-time updates
3. **Develop computer vision** algorithms with real simulation data
4. **Consider native Linux** if you need better performance long-term

---

## Alternative: Docker Desktop Display Setting

If VcXsrv doesn't work well, Docker Desktop has built-in WSLg support:

```yaml
environment:
  - DISPLAY=:0
  - WAYLAND_DISPLAY=wayland-0
  - XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
  - PULSE_SERVER=/mnt/wslg/PulseServer
```

But VcXsrv with GPU is usually better for Gazebo.

---

## Summary

X11 forwarding with VcXsrv provides a good middle ground:
- ✅ Real Gazebo camera rendering
- ✅ Works on Windows with WSL2
- ✅ Uses GPU acceleration
- ⚠️ Moderate setup effort
- ⚠️ ~10-20 FPS (slower than native)

Perfect for development and testing before moving to production Linux setup!

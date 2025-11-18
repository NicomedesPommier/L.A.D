# NVIDIA Docker Setup for WSL2 (Windows)

## Overview

This guide will help you set up GPU acceleration for Gazebo in Docker using NVIDIA Container Toolkit on Windows with WSL2.

## Prerequisites

### 1. Hardware Requirements
- ✅ **NVIDIA GPU** (GeForce, Quadro, or Tesla)
- ✅ Windows 10/11 (version 21H2 or later recommended)
- ✅ At least 8GB RAM (16GB recommended)

### 2. Software Requirements
- ✅ **WSL2** installed and set as default
- ✅ **Docker Desktop for Windows** with WSL2 backend
- ✅ **NVIDIA GPU Driver** (Windows driver ≥ 470.76)

## Step-by-Step Setup

### Step 1: Verify NVIDIA GPU on Windows

Open **PowerShell** or **Command Prompt** on Windows:

```powershell
nvidia-smi
```

**Expected output:**
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.xx       Driver Version: 535.xx       CUDA Version: 12.x    |
|-------------------------------+----------------------+----------------------+
| GPU  Name            TCC/WDDM | Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ... WDDM  | 00000000:01:00.0  On |                  N/A |
| ... (your GPU info) ...
```

✅ If you see your GPU listed, **continue to Step 2**.
❌ If not, install/update your NVIDIA GPU drivers from: https://www.nvidia.com/Download/index.aspx

---

### Step 2: Verify WSL2 GPU Support

Open **WSL2** (Ubuntu or your Linux distro):

```bash
# Check if GPU is accessible in WSL2
nvidia-smi
```

**Expected:** Same output as Windows showing your NVIDIA GPU.

**If you get "command not found" or no GPU:**

1. **Update Windows to latest version** (Windows Update)
2. **Update WSL2 kernel:**
   ```powershell
   # In PowerShell (Windows)
   wsl --update
   wsl --shutdown
   # Restart WSL
   ```

3. **Verify NVIDIA driver in WSL2:**
   ```bash
   # In WSL2
   ls /usr/lib/wsl/lib/nvidia-smi
   ```

   If this file exists, GPU support is available but not exposed properly.

---

### Step 3: Verify Docker Desktop WSL2 Backend

1. Open **Docker Desktop**
2. Go to **Settings** → **General**
3. Ensure **"Use the WSL 2 based engine"** is **checked**
4. Go to **Settings** → **Resources** → **WSL Integration**
5. Enable integration with your WSL2 distro (e.g., Ubuntu)

**Restart Docker Desktop** after changes.

---

### Step 4: Install NVIDIA Container Toolkit in WSL2

Open **WSL2 terminal**:

```bash
# 1. Add NVIDIA Container Toolkit repository
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 2. Update package list
sudo apt-get update

# 3. Install NVIDIA Container Toolkit
sudo apt-get install -y nvidia-container-toolkit

# 4. Configure Docker to use NVIDIA runtime
sudo nvidia-ctk runtime configure --runtime=docker

# 5. Restart Docker (in WSL2, we restart Docker Desktop from Windows)
```

**After Step 4, restart Docker Desktop from Windows.**

---

### Step 5: Test GPU Access in Docker

In **WSL2 terminal**:

```bash
# Test NVIDIA container runtime
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

**Expected output:** Same `nvidia-smi` output showing your GPU.

**If you get errors:**

1. **"could not select device driver"** or **"unknown runtime"**:
   ```bash
   # Check Docker daemon configuration
   cat /etc/docker/daemon.json

   # Should contain:
   # {
   #   "runtimes": {
   #     "nvidia": {
   #       "path": "nvidia-container-runtime",
   #       "runtimeArgs": []
   #     }
   #   }
   # }

   # If missing, add manually:
   sudo nvidia-ctk runtime configure --runtime=docker
   # Restart Docker Desktop from Windows
   ```

2. **"nvidia-smi not found"** in container:
   - Your Windows NVIDIA driver might be too old
   - Update to driver ≥ 470.76

---

### Step 6: Update docker-compose.yml for GPU

Navigate to your project in WSL2:

```bash
cd /mnt/c/Users/nicom/OneDrive/Documentos/GitHub/L.A.D/qcar_docker
```

**Edit `docker-compose.yml`:**

```yaml
services:
  ros:
    build: .
    image: qcar_docker-ros

    # ===== ADD GPU SUPPORT =====
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu, compute, utility]
    # ===========================

    ports:
      - "9090:9090"  # rosbridge
      - "7000:7000"  # static (URDF/meshes)
      - "8080:8080"  # WVS (optional)
      - "11345:11345" # Gazebo master

    environment:
      # ===== IMPORTANT: Remove DISPLAY=:99 (no more Xvfb) =====
      # - DISPLAY=:99  # REMOVE THIS LINE

      # ===== ADD NVIDIA ENVIRONMENT VARIABLES =====
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
      # ==========================================

      - CORS_ALLOW_ORIGIN=http://localhost:3000,http://127.0.0.1:3000,http://${EXPOSED_IP:-192.168.56.1}:3000
      - ENABLE_JSP=1
      - ENABLE_ROSAPI=1
      - ENABLE_WVS=1
      - ENABLE_TURTLESIM=1
      - ENABLE_GAZEBO=1
      - ENABLE_DUMMY_CAMERAS=0  # Disable dummy cameras (real Gazebo cameras will work)
      - GAZEBO_WORLD=/ros2_ws/src/qcar_bringup/worlds/simple_track.world
      - IP_CONFIG_PATH=/config/ip_config.json

    volumes:
      - ../config/ip_config.json:/config/ip_config.json:ro
```

**Key changes:**
1. ✅ Added `deploy.resources.reservations.devices` for GPU access
2. ✅ Added `NVIDIA_VISIBLE_DEVICES=all` and `NVIDIA_DRIVER_CAPABILITIES=all`
3. ✅ **REMOVED** `DISPLAY=:99` (no more Xvfb)
4. ✅ Set `ENABLE_DUMMY_CAMERAS=0` (use real Gazebo cameras)

---

### Step 7: Update entrypoint.sh (Remove Xvfb)

The entrypoint currently starts Xvfb for headless rendering. With GPU, we don't need it.

**Edit `qcar_docker/entrypoint.sh`:**

Find this section (around line 98):

```bash
# Start Xvfb for headless Gazebo if enabled
if [ "${ENABLE_GAZEBO}" = "1" ]; then
  echo "[entrypoint] Starting Xvfb for headless Gazebo..."
  Xvfb :99 -screen 0 1024x768x24 &
  export DISPLAY=:99

  # Set Gazebo model path so it can find meshes
  export GAZEBO_MODEL_PATH="${QCAR_DESC_SHARE}:${GAZEBO_MODEL_PATH}"
  echo "[entrypoint] GAZEBO_MODEL_PATH: ${GAZEBO_MODEL_PATH}"

  # Disable model database to speed up initialization
  export GAZEBO_MODEL_DATABASE_URI=""
  echo "[entrypoint] Disabled Gazebo model database for faster startup"

  sleep 2
fi
```

**Replace with:**

```bash
# Configure Gazebo for GPU rendering
if [ "${ENABLE_GAZEBO}" = "1" ]; then
  echo "[entrypoint] Configuring Gazebo with GPU acceleration..."

  # Set Gazebo model path so it can find meshes
  export GAZEBO_MODEL_PATH="${QCAR_DESC_SHARE}:${GAZEBO_MODEL_PATH}"
  echo "[entrypoint] GAZEBO_MODEL_PATH: ${GAZEBO_MODEL_PATH}"

  # Disable model database to speed up initialization
  export GAZEBO_MODEL_DATABASE_URI=""
  echo "[entrypoint] Disabled Gazebo model database for faster startup"

  # GPU rendering - no Xvfb needed
  echo "[entrypoint] Using GPU rendering (no Xvfb)"
fi
```

---

### Step 8: Rebuild and Test

In WSL2:

```bash
cd /mnt/c/Users/nicom/OneDrive/Documentos/GitHub/L.A.D/qcar_docker

# Rebuild with GPU support
docker compose down
docker compose build
docker compose up -d

# Wait 20 seconds for Gazebo to initialize
sleep 20

# Check logs for camera initialization
docker compose logs ros | grep -E "(camera|GPU|CUDA|rendering)"

# Verify camera topics are publishing
docker compose exec ros bash -c "source /opt/ros/humble/setup.bash && ros2 topic info /qcar/rgb/image_color"
```

**Expected:**
- Logs should show camera plugins initializing
- `Publisher count: 1` (Gazebo camera publishing)
- No more "Unable to create CameraSensor" errors

---

### Step 9: Test Camera Performance

```bash
# Check camera publishing rate (should be ~30 Hz)
docker compose exec ros bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic hz /qcar/rgb/image_color"

# Expected output:
# average rate: 29.xxx
#   min: 0.030s max: 0.035s std dev: 0.00xxx s window: 30
```

---

## Troubleshooting

### Issue: "could not select device driver" or "unknown flag: --gpus"

**Cause:** Docker daemon doesn't recognize NVIDIA runtime.

**Solution:**
```bash
# In WSL2
sudo nvidia-ctk runtime configure --runtime=docker
cat /etc/docker/daemon.json  # Verify configuration

# Restart Docker Desktop from Windows
# Then test again
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

---

### Issue: nvidia-smi not found in WSL2

**Cause:** GPU drivers not passed through to WSL2.

**Solution:**
1. Update Windows to latest version (21H2+)
2. Update NVIDIA driver on Windows (≥470.76)
3. Update WSL2: `wsl --update` in PowerShell
4. Restart: `wsl --shutdown`, then reopen WSL2

---

### Issue: Gazebo cameras still not working

**Check these:**

1. **Verify GPU is accessible in container:**
   ```bash
   docker compose exec ros nvidia-smi
   ```
   Should show your GPU.

2. **Check DISPLAY variable is NOT set to :99:**
   ```bash
   docker compose exec ros bash -c "echo \$DISPLAY"
   ```
   Should be empty or a real display.

3. **Check Gazebo logs:**
   ```bash
   docker compose logs ros | grep -i "camera\|rendering\|opengl"
   ```

4. **Verify gzserver is using GPU:**
   ```bash
   docker compose exec ros bash -c "nvidia-smi"
   ```
   Should show `gzserver` process using GPU memory.

---

### Issue: "Failed to create OpenGL context"

**Cause:** NVIDIA driver capabilities not properly set.

**Solution:**
Verify `NVIDIA_DRIVER_CAPABILITIES=all` in docker-compose.yml environment section.

---

## Performance Expectations

With NVIDIA Docker + WSL2:

| Metric | Before (Software) | After (GPU) |
|--------|-------------------|-------------|
| Camera FPS | 0-5 (inconsistent) | 25-30 (stable) |
| GPU Memory | 0 MB | ~200-500 MB |
| CPU Usage | High (~80%) | Low (~20%) |
| Gazebo Startup | 60-90s | 30-45s |
| Multiple Cameras | Doesn't work | All 6 work |

---

## Verification Checklist

Before testing in React app:

- [ ] `nvidia-smi` works in Windows
- [ ] `nvidia-smi` works in WSL2
- [ ] `docker run --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi` works
- [ ] `docker-compose.yml` has GPU configuration
- [ ] `entrypoint.sh` doesn't use Xvfb (DISPLAY=:99 removed)
- [ ] `ENABLE_DUMMY_CAMERAS=0` in docker-compose.yml
- [ ] Gazebo container shows GPU in `nvidia-smi`
- [ ] Camera topics show `Publisher count: 1`
- [ ] `ros2 topic hz /qcar/rgb/image_color` shows ~30 Hz

---

## Next Steps

Once GPU rendering is working:

1. **Test in React app** - Should see real Gazebo camera feeds
2. **Drive the robot** - Verify all sensors work simultaneously
3. **Enjoy smooth performance** - 6 cameras + LIDAR + physics at 30 FPS!

---

## Additional Resources

- [NVIDIA Container Toolkit Installation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- [Docker GPU Support](https://docs.docker.com/config/containers/resource_constraints/#gpu)
- [WSL2 GPU Support](https://docs.microsoft.com/en-us/windows/ai/directml/gpu-cuda-in-wsl)
- [Gazebo GPU Requirements](http://gazebosim.org/tutorials?tut=guided_i1)

---

## Summary

NVIDIA Docker with WSL2 provides:
- ✅ Native GPU acceleration
- ✅ Real camera rendering (all 6 cameras)
- ✅ Smooth 30 FPS performance
- ✅ Production-ready setup
- ✅ Better than X11 forwarding on Windows

This is the **recommended solution** for Windows development with Gazebo!

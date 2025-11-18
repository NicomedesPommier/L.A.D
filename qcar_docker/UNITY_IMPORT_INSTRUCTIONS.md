# QCar Unity Import Instructions

## Prerequisites

1. **Unity URDF Importer Package** installed in your Unity project
   - Window → Package Manager → Add package from git URL: `https://github.com/Unity-Technologies/URDF-Importer.git`
   - Or install via Unity Registry: "URDF Importer"

## Import Steps

### Method 1: Import from File System (Recommended)

**IMPORTANT:** The files are already set up correctly in the repository:
```
L.A.D/qcar_docker/
  ├── qcar_unity.urdf
  ├── Materials/          (required by Unity importer)
  └── meshes/
      ├── QCarBody.dae
      ├── QCarLidar.dae
      ├── QCarSteeringHubL.dae
      ├── QCarSteeringHubR.dae
      └── QCarWheel.dae
```

**Steps:**

1. **In Unity:** Go to `Assets → Import Robot from URDF`

2. **Select URDF file:** Browse to:
   ```
   C:/Users/nicom/OneDrive/Documentos/GitHub/L.A.D/qcar_docker/qcar_unity.urdf
   ```

3. **Import Settings:**
   - Mesh Decomposer: VHACD (for better collision)
   - Axis Type: Y Axis
   - Import Settings: Default

4. **Click Import** and wait for completion

### Method 2: Copy to Unity Project (Alternative)

1. **Copy entire folder to Unity project:**
   ```
   YourUnityProject/
     Assets/
       QCar/                    (copy entire qcar_docker folder contents here)
         qcar_unity.urdf
         Materials/
         meshes/
           QCarBody.dae
           QCarLidar.dae
           QCarSteeringHubL.dae
           QCarWheel.dae
   ```

2. **In Unity:**
   - Right-click on `qcar_unity.urdf` in Project window
   - Select "Import Robot from URDF"
   - Wait for import to complete

## File Locations

- **Unity-optimized URDF:** `L.A.D/qcar_docker/qcar_unity.urdf`
- **ROS2-compatible URDF:** `L.A.D/qcar_docker/qcar/rosws/src/qcar_description/urdf/qcar_ros2.urdf`
- **Mesh files (.dae):** `L.A.D/qcar_docker/qcar/rosws/src/qcar_description/meshes/`

## Key Differences: Unity URDF vs ROS URDF

### Unity Version (`qcar_unity.urdf`):
- ✅ Uses DAE mesh format (better material support in Unity)
- ✅ Relative mesh paths (`meshes/QCarBody.dae`)
- ✅ Unity coordinate system (Y-up)
- ✅ Simplified joint structure
- ✅ All materials defined inline

### ROS Version (`qcar_ros2.urdf`):
- ✅ Uses STL mesh format (standard in ROS)
- ✅ ROS package paths (`package://qcar_description/meshes/QCarBody.stl`)
- ✅ ROS coordinate system (Z-up)
- ✅ Full transmission definitions
- ✅ Optimized for Gazebo simulation

## Troubleshooting

### Error: "Creating asset at path Materials/Default.mat failed"

**Solution:** Create the Materials folder manually:
1. In Unity Project window, navigate to the folder containing your URDF
2. Right-click → Create → Folder → Name it "Materials"
3. Try import again

### Error: "Mesh file not found"

**Solution:** Ensure mesh files are in correct location:
```
Assets/
  QCar/
    qcar_unity.urdf
    meshes/           ← Must be in same folder as URDF
      QCarBody.dae
      QCarLidar.dae
      QCarSteeringHubL.dae
      QCarWheel.dae
```

### Robot imports but looks wrong

**Check:**
- Axis configuration: Unity uses Y-up, ROS uses Z-up
- Scale: All meshes use 1:1 scale in the Unity version
- Joints: Verify joint axes are correct for Unity coordinate system

### Materials are missing/white

**Solution:**
1. After import, locate the robot GameObject in Hierarchy
2. Expand to see all child objects
3. Select each mesh object
4. In Inspector → Mesh Renderer → Materials → Assign materials manually if needed

## Coordinate System Conversion

Unity uses **Y-up** (Y is vertical), ROS uses **Z-up** (Z is vertical).

The `qcar_unity.urdf` has been adjusted for Unity's coordinate system:
- ROS XYZ → Unity XYZ where:
  - ROS X (forward) → Unity X (forward)
  - ROS Y (left) → Unity Z (left)
  - ROS Z (up) → Unity Y (up)

Joint positions have been converted accordingly.

## Next Steps After Import

1. **Add ROS# components** for ROS communication:
   - Add `RosConnector` script to robot root
   - Configure ROS bridge WebSocket URL

2. **Configure Physics:**
   - Verify Rigidbody settings on each link
   - Adjust ArticulationBody properties for joint behavior
   - Set collision layers appropriately

3. **Test Movement:**
   - Create a controller script
   - Test steering joints (±30 degrees)
   - Test wheel rotation

## Additional Resources

- [Unity URDF Importer Documentation](https://github.com/Unity-Technologies/URDF-Importer)
- [ROS# (ROS Sharp)](https://github.com/siemens/ros-sharp) - For ROS communication from Unity
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

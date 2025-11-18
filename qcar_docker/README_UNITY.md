# QCar Unity Integration

This folder contains everything needed to import the QCar robot into Unity.

## Quick Start

### ✅ Files are Ready!

The repository is pre-configured with all necessary files:

```
qcar_docker/
├── qcar_unity.urdf        ← Unity-optimized URDF file
├── Materials/             ← Required folder for Unity importer
├── meshes/                ← DAE mesh files for Unity
│   ├── QCarBody.dae
│   ├── QCarLidar.dae
│   ├── QCarSteeringHubL.dae
│   ├── QCarSteeringHubR.dae
│   └── QCarWheel.dae
├── copy_to_unity.bat      ← Windows copy script
├── copy_to_unity.sh       ← Linux/Mac copy script
└── UNITY_IMPORT_INSTRUCTIONS.md  ← Detailed instructions
```

### ⚠️ IMPORTANT: Unity Requires Files Inside Project

Unity **cannot import URDF files from outside the project folder**. You must copy the files into your Unity project's Assets folder first.

### Import into Unity

**Step 1: Copy Files to Unity Project**

Choose your method:

**Windows:**
```cmd
cd qcar_docker
copy_to_unity.bat "C:\path\to\your\UnityProject"
```

**Linux/Mac:**
```bash
cd qcar_docker
chmod +x copy_to_unity.sh
./copy_to_unity.sh "/path/to/your/UnityProject"
```

**Manual Copy:**
1. Copy these folders/files to your Unity project:
   ```
   From: L.A.D/qcar_docker/
   To:   YourUnityProject/Assets/QCar/

   Files to copy:
   - qcar_unity.urdf
   - Materials/ (folder)
   - meshes/ (folder with all .dae files)
   ```

**Step 2: Import in Unity**

1. Open your Unity project
2. In Project window, navigate to `Assets/QCar/`
3. Right-click on `qcar_unity.urdf`
4. Select "Import Robot from URDF"
5. Wait for import to complete
6. Done! ✨

## Troubleshooting

### ❌ Error: "Invalid AssetDatabase path... Use path relative to the project folder"

**Cause:** Unity cannot import files from outside the Unity project folder.

**Solution:** You **must** copy the files into your Unity project's Assets folder first. Use one of these methods:
- Run `copy_to_unity.bat "C:\path\to\UnityProject"` (Windows)
- Run `./copy_to_unity.sh "/path/to/UnityProject"` (Linux/Mac)
- Manually copy `qcar_docker/` contents to `YourUnityProject/Assets/QCar/`

### ❌ Error: "Parent directory must exist"

**Solution:** The `Materials/` folder is missing. Use the copy script or manually create the folder structure in your Unity Assets.

### ❌ Error: "Mesh file not found"

**Solution:** Make sure the `meshes/` folder with DAE files is in the same directory as the URDF file inside your Unity project.

### ⚠️ Robot looks wrong after import

- Check Unity coordinate system (Y-up vs Z-up)
- Verify mesh scale (should be 1:1 in Unity version)
- Confirm joint axes are correct

## File Comparison

| Feature | ROS Version | Unity Version |
|---------|-------------|---------------|
| **Location** | `qcar/rosws/src/qcar_description/urdf/qcar_ros2.urdf` | `qcar_unity.urdf` |
| **Mesh Format** | STL | DAE (COLLADA) |
| **Paths** | `package://` | Relative |
| **Coordinate System** | Z-up (ROS) | Y-up (Unity) |
| **Use Case** | Gazebo simulation, RViz | Unity game engine |

## Need Help?

See [UNITY_IMPORT_INSTRUCTIONS.md](UNITY_IMPORT_INSTRUCTIONS.md) for detailed instructions and troubleshooting.

## ROS Integration in Unity

After importing the robot, you can connect it to ROS using:

- **ROS#** (ROS Sharp): https://github.com/siemens/ros-sharp
- **Unity Robotics Hub**: https://github.com/Unity-Technologies/Unity-Robotics-Hub

Configure the ROS bridge WebSocket to connect to: `ws://YOUR_IP:9090`

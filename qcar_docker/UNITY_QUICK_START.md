# QCar Unity Import - Quick Start Guide

## 🚨 Important: Unity Cannot Import from Outside Project Folder

Unity's URDF importer **only works with files inside your Unity project's Assets folder**.

## ✅ Solution: 3-Step Process

### Step 1: Find Your Unity Project Path

Your Unity project folder should look like this:
```
YourUnityProject/
├── Assets/         ← This is where files must go
├── Library/
├── Packages/
├── ProjectSettings/
└── ...
```

Example paths:
- Windows: `C:\Users\YourName\Documents\UnityProjects\MyRobotProject`
- Mac: `/Users/YourName/Documents/UnityProjects/MyRobotProject`
- Linux: `/home/yourname/UnityProjects/MyRobotProject`

### Step 2: Run the Copy Script

**Windows:**
1. Open Command Prompt
2. Navigate to L.A.D project:
   ```cmd
   cd C:\Users\nicom\OneDrive\Documentos\GitHub\L.A.D\qcar_docker
   ```
3. Run the copy script:
   ```cmd
   copy_to_unity.bat "C:\path\to\your\UnityProject"
   ```

**Linux/Mac:**
1. Open Terminal
2. Navigate to L.A.D project:
   ```bash
   cd ~/path/to/L.A.D/qcar_docker
   ```
3. Make script executable and run:
   ```bash
   chmod +x copy_to_unity.sh
   ./copy_to_unity.sh "/path/to/your/UnityProject"
   ```

### Step 3: Import in Unity

1. Open your Unity project
2. In the Project window (bottom), you should see:
   ```
   Assets/
   └── QCar/
       ├── qcar_unity.urdf
       ├── Materials/
       └── meshes/
   ```
3. Right-click on `qcar_unity.urdf`
4. Select **"Import Robot from URDF"**
5. Wait 10-30 seconds for import to complete
6. Your QCar robot will appear in the Hierarchy! 🎉

## 📋 What Gets Copied

The script copies these files to `Assets/QCar/`:

```
Assets/QCar/
├── qcar_unity.urdf           ← Main URDF file
├── Materials/                ← Unity will create materials here
│   └── (Unity generates files here)
└── meshes/                   ← 3D model files
    ├── QCarBody.dae
    ├── QCarLidar.dae
    ├── QCarSteeringHubL.dae
    ├── QCarSteeringHubR.dae
    └── QCarWheel.dae
```

## ❌ Common Errors & Solutions

### "Invalid AssetDatabase path"
- **Problem:** You tried to import from outside Unity project
- **Solution:** Use the copy script to copy files into `Assets/QCar/`

### "Parent directory must exist"
- **Problem:** `Materials/` folder missing
- **Solution:** The copy script creates it automatically

### "Mesh file not found"
- **Problem:** Mesh files not in correct location
- **Solution:** Ensure `meshes/` folder is alongside `qcar_unity.urdf`

### Script doesn't run (Windows)
- **Problem:** Batch files might be blocked
- **Solution:** Right-click script → Properties → Unblock (if button exists)

### Script permission denied (Linux/Mac)
- **Problem:** Script not executable
- **Solution:** Run `chmod +x copy_to_unity.sh`

## 🎯 After Import

Once imported successfully, you can:

1. **View the robot:**
   - Select `QCar` in Hierarchy
   - Zoom to see it in Scene view (press F with object selected)

2. **Test joints:**
   - Expand `QCar` in Hierarchy
   - Select joint objects
   - Modify ArticulationBody properties in Inspector

3. **Add ROS communication:**
   - Install ROS# (ROS Sharp) package
   - Add RosConnector component
   - Configure WebSocket: `ws://YOUR_IP:9090`

## 📚 More Information

- Full documentation: [README_UNITY.md](README_UNITY.md)
- Detailed instructions: [UNITY_IMPORT_INSTRUCTIONS.md](UNITY_IMPORT_INSTRUCTIONS.md)
- ROS integration: [UNITY_INTEGRATION.md](UNITY_INTEGRATION.md)

## 🆘 Still Having Issues?

1. Double-check your Unity project path has an `Assets/` folder
2. Make sure Unity URDF Importer package is installed:
   - Window → Package Manager → Add from git URL
   - `https://github.com/Unity-Technologies/URDF-Importer.git`
3. Check Unity console for detailed error messages
4. Verify all mesh files exist in `Assets/QCar/meshes/`

## 🔄 Need to Update?

If you make changes to the URDF or meshes:
1. Run the copy script again (it will overwrite)
2. In Unity: Right-click the robot in Hierarchy → Delete
3. Re-import the URDF file

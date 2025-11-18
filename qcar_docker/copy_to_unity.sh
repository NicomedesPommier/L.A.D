#!/bin/bash
# Script to copy QCar URDF and meshes to Unity project
# Usage: ./copy_to_unity.sh "/path/to/your/UnityProject"

echo "========================================"
echo "QCar Unity Import - File Copy Script"
echo "========================================"
echo

if [ -z "$1" ]; then
    echo "ERROR: No Unity project path provided!"
    echo
    echo "Usage: ./copy_to_unity.sh \"/path/to/your/UnityProject\""
    echo
    echo "Example: ./copy_to_unity.sh \"/home/user/MyUnityProject\""
    echo
    exit 1
fi

UNITY_PROJECT="$1"
DEST_FOLDER="$UNITY_PROJECT/Assets/QCar"

echo "Unity Project: $UNITY_PROJECT"
echo "Destination: $DEST_FOLDER"
echo

# Check if Unity project exists
if [ ! -d "$UNITY_PROJECT/Assets" ]; then
    echo "ERROR: Invalid Unity project path!"
    echo "The folder \"$UNITY_PROJECT/Assets\" does not exist."
    echo
    exit 1
fi

# Create QCar folder in Assets
echo "Creating QCar folder in Unity Assets..."
mkdir -p "$DEST_FOLDER"

# Create Materials folder
echo "Creating Materials folder..."
mkdir -p "$DEST_FOLDER/Materials"

# Create meshes folder
echo "Creating meshes folder..."
mkdir -p "$DEST_FOLDER/meshes"

# Copy URDF file
echo "Copying URDF file..."
cp -f "qcar_unity.urdf" "$DEST_FOLDER/qcar_unity.urdf"

# Copy mesh files
echo "Copying mesh files..."
cp -f meshes/*.dae "$DEST_FOLDER/meshes/"

echo
echo "========================================"
echo "SUCCESS! Files copied to Unity project"
echo "========================================"
echo
echo "Location: $DEST_FOLDER"
echo
echo "Next steps:"
echo "1. Open your Unity project"
echo "2. In Unity, go to Assets > Import Robot from URDF"
echo "3. Select: Assets/QCar/qcar_unity.urdf"
echo "4. Click Import"
echo
echo "OR"
echo
echo "1. In Unity Project window, navigate to Assets/QCar/"
echo "2. Right-click on qcar_unity.urdf"
echo "3. Select \"Import Robot from URDF\""
echo

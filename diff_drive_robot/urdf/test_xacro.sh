#!/bin/bash
# This script tests if the XACRO processing works with namespaces

# Get the package path
PACKAGE_PATH=$(ros2 pkg prefix diff_drive_robot)
URDF_PATH=${PACKAGE_PATH}/share/diff_drive_robot/urdf/robot.urdf.xacro

echo "Testing XACRO processing with namespace..."
echo "URDF path: ${URDF_PATH}"

# Process with robot1 namespace
echo "Processing with robot1 namespace:"
xacro ${URDF_PATH} namespace:=robot1 > /tmp/robot1.urdf
if [ $? -eq 0 ]; then
    echo "Success! Processed XACRO with robot1 namespace."
    echo "Check /tmp/robot1.urdf to verify the namespace was applied correctly."
    
    # Grep for the robot1 namespace in the output
    echo -e "\nChecking for robot1 namespace in output:"
    grep -n "robot1" /tmp/robot1.urdf | head -5
else
    echo "Error processing XACRO with robot1 namespace."
fi

echo -e "\nDone!"

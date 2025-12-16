#!/bin/bash

# Script to validate URDF files
# This script checks if the URDF file is well-formed and valid

echo "Validating URDF file..."

# Check if check_urdf command exists
if ! command -v check_urdf &> /dev/null; then
    echo "Error: check_urdf command not found. Please install ROS 2 urdf packages."
    echo "You can install it with: sudo apt install ros-<distro>-urdf"
    exit 1
fi

# Define the URDF file path
URDF_FILE="simple_humanoid.urdf"

# Check if the URDF file exists
if [ ! -f "$URDF_FILE" ]; then
    echo "Error: URDF file '$URDF_FILE' not found in current directory."
    exit 1
fi

# Validate the URDF file
echo "Validating $URDF_FILE..."
check_urdf "$URDF_FILE"

# Capture the exit code
exit_code=$?

if [ $exit_code -eq 0 ]; then
    echo "✓ URDF file is valid!"
    echo "Model name: $(grep -o 'name="[^"]*"' "$URDF_FILE" | head -1 | cut -d '"' -f 2)"
    echo "Number of links: $(grep -c '<link' "$URDF_FILE")"
    echo "Number of joints: $(grep -c '<joint' "$URDF_FILE")"
else
    echo "✗ URDF file has errors!"
    exit $exit_code
fi
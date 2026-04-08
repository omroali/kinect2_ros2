#!/bin/bash

# ============================================================================
# Helper script for launching individual Kinect v2 sensors
#
# This script works around the ROS 2 YAML parameter type conversion issue
# where serial numbers with leading zeros (like 001934470647) are incorrectly
# interpreted as octal numbers or doubles instead of strings.
#
# Usage:
#   ./launch_kinect.sh 1                    # Launch Kinect 1 (serial 001934470647)
#   ./launch_kinect.sh 2                    # Launch Kinect 2 (serial 007425354147)
#   ./launch_kinect.sh <serial> <namespace> # Custom serial and namespace
#
# To run both Kinects simultaneously:
#   Terminal 1: ./launch_kinect.sh 1
#   Terminal 2: ./launch_kinect.sh 2
#
# Note: Running two Kinects simultaneously requires sufficient USB bandwidth.
# Ideally, each Kinect should be on a separate USB 3.0 controller.
# ============================================================================

set -e

# Known Kinect serial numbers (update these for your setup)
KINECT1_SERIAL="001934470647"
KINECT2_SERIAL="007425354147"

print_usage() {
    echo "=============================================="
    echo "  Kinect v2 Single Launcher"
    echo "=============================================="
    echo ""
    echo "Usage: $0 [1|2|<serial> <namespace>]"
    echo ""
    echo "Options:"
    echo "  1                    Launch Kinect 1 (serial: $KINECT1_SERIAL) as kinect2_1"
    echo "  2                    Launch Kinect 2 (serial: $KINECT2_SERIAL) as kinect2_2"
    echo "  <serial> <ns>        Launch custom Kinect with given serial and namespace"
    echo ""
    echo "Examples:"
    echo "  $0 1                                    # Launch Kinect 1"
    echo "  $0 2                                    # Launch Kinect 2"
    echo "  $0 007425354147 my_kinect              # Custom serial and namespace"
    echo ""
    echo "To run both Kinects, open two terminals:"
    echo "  Terminal 1: $0 1"
    echo "  Terminal 2: $0 2"
    echo ""
    echo "After launching, verify with:"
    echo "  ros2 topic list | grep kinect"
    echo ""
}

# Check for help flag
if [[ "$1" == "-h" || "$1" == "--help" || -z "$1" ]]; then
    print_usage
    exit 0
fi

# Determine serial and namespace based on arguments
if [[ "$1" == "1" ]]; then
    SERIAL="$KINECT1_SERIAL"
    NAMESPACE="kinect2_1"
elif [[ "$1" == "2" ]]; then
    SERIAL="$KINECT2_SERIAL"
    NAMESPACE="kinect2_2"
elif [[ -n "$1" && -n "$2" ]]; then
    SERIAL="$1"
    NAMESPACE="$2"
else
    echo "Error: Invalid arguments"
    echo ""
    print_usage
    exit 1
fi

echo "=============================================="
echo "  Launching Kinect v2"
echo "=============================================="
echo "  Serial:    $SERIAL"
echo "  Namespace: $NAMESPACE"
echo "=============================================="
echo ""

# Source ROS 2 workspace if not already sourced
if [[ -z "$AMENT_PREFIX_PATH" ]]; then
    echo "Sourcing ROS 2 workspace..."
    if [[ -f ~/base_ws/install/setup.bash ]]; then
        source ~/base_ws/install/setup.bash
    elif [[ -f /opt/ros/humble/setup.bash ]]; then
        source /opt/ros/humble/setup.bash
        echo "Warning: Only sourced base ROS 2, kinect2_bridge may not be available."
    else
        echo "Error: Could not find ROS 2 setup.bash"
        exit 1
    fi
fi

echo "Starting kinect2_bridge..."
echo "Press Ctrl+C to stop."
echo ""

# Launch the Kinect using the single launch file
# The launch file uses quoted strings to avoid YAML type conversion issues
ros2 launch kinect2_bridge kinect2_single.launch.py \
    serial:="$SERIAL" \
    namespace:="$NAMESPACE"

#!/bin/bash

# Robot Dog Lower Control RL Mode Service Script
# Purpose: Start the lower control program and receive control commands from the upper reinforcement learning model

# Exit on error
set -e

# Get the absolute path of the script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"  # Automatically get workspace path

# Define cleanup function
cleanup() {
    echo "Shutting down all running robot control programs..."
    pkill -f "robot_control" || echo "No running robot control programs found"
    exit 0
}

# Capture SIGINT signal
trap cleanup SIGINT

# Check ROS environment
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
    else
        echo "Error: ROS environment configuration file not found (/opt/ros/noetic/setup.bash)"
        exit 1
    fi
fi

# Check workspace
if [ ! -d "${WORKSPACE_DIR}" ]; then
    echo "Error: Workspace directory does not exist, please modify the WORKSPACE_DIR variable in the script"
    exit 1
fi

# Check if workspace needs compilation
if [ ! -d "${WORKSPACE_DIR}/build" ] || [ ! -d "${WORKSPACE_DIR}/install" ]; then
    echo "Detected workspace not compiled, starting compilation..."
    
    # Save current directory
    CURRENT_DIR=$(pwd)
    
    # Switch to workspace directory
    cd "${WORKSPACE_DIR}"
    
    # Execute compilation
    if ! catkin_make; then
        echo "Error: Workspace compilation failed"
        cd "${CURRENT_DIR}"
        exit 1
    fi
    
    echo "Workspace compilation successful"
    
    # Return to original directory
    cd "${CURRENT_DIR}"
fi

# Set workspace environment
if [ -f "${WORKSPACE_DIR}/devel/setup.bash" ]; then
    source "${WORKSPACE_DIR}/devel/setup.bash"
else
    echo "Error: Cannot find workspace setup.bash file, compilation may be incomplete"
    exit 1
fi

# Check and enter control program directory
CONTROL_DIR="${WORKSPACE_DIR}/devel/lib/robot_control"
if [ ! -d "${CONTROL_DIR}" ]; then
    echo "Warning: Cannot find control program directory in devel directory (${CONTROL_DIR})"
    # Try to find in other possible locations
    CONTROL_DIR="${WORKSPACE_DIR}/install/lib/robot_control"
    if [ ! -d "${CONTROL_DIR}" ]; then
        # Try to find directly in source directory
        CONTROL_DIR="${WORKSPACE_DIR}/src/dev_robot_control_sdk"
        if [ ! -d "${CONTROL_DIR}" ]; then
            echo "Error: Cannot find control program directory"
            exit 1
        fi
    fi
fi
echo "Using control program directory: ${CONTROL_DIR}"
cd "${CONTROL_DIR}"

# Set library path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:./

# Check configuration file
if [ ! -f "config/devq.yaml" ]; then
    echo "Warning: Configuration file not found in current directory (config/devq.yaml)"
    # Try to find in other possible locations
    if [ -f "${WORKSPACE_DIR}/src/dev_robot_control_sdk/config/devq.yaml" ]; then
        echo "Found configuration file in source directory, copying to current directory"
        mkdir -p config
        cp "${WORKSPACE_DIR}/src/dev_robot_control_sdk/config/devq.yaml" config/
    else
        echo "Error: Cannot find configuration file (config/devq.yaml)"
        exit 1
    fi
fi

# Check model directory
if [ ! -d "model/devq" ] || [ ! -f "model/devq/policy.mnn" ]; then
    echo "Warning: Model file not found in current directory (model/devq/policy.mnn)"
    
    # Check singular model directory
    if [ -f "${WORKSPACE_DIR}/src/dev_robot_control_sdk/model/devq/policy.mnn" ]; then
        echo "Found model file in source directory, copying to current directory"
        mkdir -p model/devq
        cp "${WORKSPACE_DIR}/src/dev_robot_control_sdk/model/devq/policy.mnn" model/devq/
    # Check plural models directory (might be inconsistent naming)
    elif [ -f "${WORKSPACE_DIR}/src/dev_robot_control_sdk/models/devq/policy.mnn" ]; then
        echo "Found model file in models directory, copying to current directory"
        mkdir -p model/devq
        cp "${WORKSPACE_DIR}/src/dev_robot_control_sdk/models/devq/policy.mnn" model/devq/
    else
        echo "Error: Cannot find model file (model/devq/policy.mnn)"
        echo "Please ensure source directory contains model/devq/policy.mnn or models/devq/policy.mnn file"
        exit 1
    fi
fi

# Check executable file
if [ ! -f "./robot_control" ]; then
    echo "Warning: Control program executable not found in current directory (robot_control)"
    # Try to find in devel directory
    if [ -f "${WORKSPACE_DIR}/devel/lib/robot_control/robot_control" ]; then
        echo "Found executable in devel directory, using that file"
        ROBOT_EXEC="${WORKSPACE_DIR}/devel/lib/robot_control/robot_control"
    else
        echo "Error: Cannot find control program executable"
        exit 1
    fi
else
    ROBOT_EXEC="./robot_control"
fi

# Set ROS network environment variables - Ensure lower control can communicate with upper control
# Lower control IP (usually 10.10.10.10)
export ROS_IP=10.10.10.10
export ROS_MASTER_URI=http://10.10.10.10:11311

# Check if instance is already running
if pgrep -f "robot_control" > /dev/null; then
    echo "Warning: Robot control program already running"
    # Get existing process PID
    CONTROL_PID=$(pgrep -f "robot_control" | head -1)
    echo "Using existing control program (PID: ${CONTROL_PID})"
else
    # Start robot control program - Remote control mode
    echo "Starting robot control program (Remote control mode)..."
    # Add remote control mode parameters - For reinforcement learning model control
    ${ROBOT_EXEC} config/devq.yaml --remote_mode=true &
    CONTROL_PID=$!
    echo "Robot control program started (PID: ${CONTROL_PID})"
    
    # Wait a few seconds to ensure control program is fully started
    echo "Waiting for control program to start..."
    sleep 5
    
    # Check if process is still running
    if ! ps -p ${CONTROL_PID} > /dev/null 2>&1; then
        echo "Error: Control program failed to start or has crashed"
        echo "Please check logs for more information"
        exit 1
    fi
fi

# Check if ROS master node is running
if ! rostopic list > /dev/null 2>&1; then
    echo "Error: ROS master node not running"
    echo "Attempting to start ROS master node..."
    roscore &
    sleep 3
fi

# Set RL remote control mode
echo "Setting remote control mode to RL mode..."
# Send mode switch command - Assuming there is a set_remote_mode topic
rostopic pub -1 /robot_control/set_mode std_msgs/Int32MultiArray "data: [1, 1]" || echo "Warning: Unable to set remote control mode, manual setting may be required"

echo ""
echo "============================================="
echo "Robot Dog Lower Control RL Mode Started"
echo "Configured to receive control commands from upper reinforcement learning model"
echo "Supported topics:"
echo "- Status publish: /robot/state"
echo "- Command receive: /robot/command"
echo "============================================="
echo "Press Ctrl+C to stop service and exit"
echo ""

# Display some useful debug information
echo "Registered ROS topics:"
rostopic list

# Infinite loop, keep script running until user presses Ctrl+C
while true; do
    sleep 1
done

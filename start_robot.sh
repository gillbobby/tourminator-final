#!/bin/bash

# Start both launch files simultaneously
# Ctrl+C will stop both

# Launch hardware in background
roslaunch hardware_interface hardware.launch &
HARDWARE_PID=$!

# Give hardware a moment to initialize
sleep 3

# Launch navigation with exploration enabled
roslaunch indoor_mapping_robot navigation.launch enable_exploration:=true &
NAV_PID=$!

# Function to kill both processes on exit
cleanup() {
    echo "Shutting down robot..."
    kill $HARDWARE_PID $NAV_PID 2>/dev/null
    wait $HARDWARE_PID $NAV_PID 2>/dev/null
    exit 0
}

# Trap Ctrl+C and kill both processes
trap cleanup SIGINT SIGTERM

# Wait for both processes
wait
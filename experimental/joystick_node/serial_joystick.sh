#!/bin/bash

sudo chmod +x config.py

# Capture the output of the Python script
PORTS=$(python3 config.py)

# Split the output into separate variables
JOYSTICK_PORT=$(echo "$PORTS" | head -n 1)
WHEEL_PORT=$(echo "$PORTS" | tail -n 1)

# Check if pyserial is installed
if ! python3 -c "import serial" &>/dev/null; then
    echo "pyserial is not installed. Installing pyserial..."
    pip3 install pyserial
fi

# Check if the joystick device is available
if [ ! -e "$JOYSTICK_PORT" ]; then
    echo "Cannot find joystick at /dev/input/js0. Please set the correct joystick port in config.conf"
    exit 1
fi

# Set permissions for the joystick device
sudo chmod a+rw $JOYSTICK_PORT

# Check if the Arduino is connected
if [ ! -e $WHEEL_PORT ]; then
    echo "Wheel Arduino not found. Make sure the connection is set and configure the correct port in config.conf"
    exit 1
fi

# Source ROS environment
# Ensure this path is correct for your ROS workspace
source ../../devel/setup.bash

roscore &
rosparam set joy_node/dev "$JOYSTICK_PORT"
# Launch the joystick node
roslaunch joystick_node joystick_serial.launch

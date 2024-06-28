########### CONFIGURE THE JOYSTICK PORT AND ARDUINO PORTS FIRST #################
#################################################################################

JOYSTICK_PORT = "/dev/input/js0"  # REPLACE WITH CURRENT PORT WHERE JOYSTICK IS CONNECTED
WHEEL_PORT = "/dev/ttyUSB0"  # REPLACE WITH CURRENT PORT WHERE WHEEL ARDUINO IS CONNECTED

# Print each port separately
print(JOYSTICK_PORT)
print(WHEEL_PORT)

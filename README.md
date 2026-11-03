# serial_bridge_2026

ROS 2 package responsible for communication between the robot's computer and its embedded controllers over serial connections.

It handles the exchange of commands and sensor data between ROS 2 and the microcontrollers. The package also includes utilities for:

- Reading and writing serial data.
- Checking packet integrity using CRC-8.
- Processing odometry data.
- Multiplexing robot velocity commands.
- Handling joystick input and GPIO buttons.
- Teaching or recording setpoints.

The serial protocol configuration must match the firmware running on the robot's controllers.

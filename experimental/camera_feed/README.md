# Rover Camera Feed System

## Overview
This code contains a system for streaming video feed from multiple cameras connected to a network. The system consists of two main components: a broadcaster and a receiver. The broadcaster captures video from individual cameras that is on the Rover and broadcasts it over WebSockets, while the receiver connects to these WebSockets to receive and display the video feed. The camera used for this is ZED 2 (depth perception, motion tracking and spatial AI camera)

## Components
### 1. `broadcaster.py`
- Responsible for capturing video from cameras and broadcasting it over WebSockets.
- Utilizes OpenCV for camera handling, base64 for encoding frames, and websockets for communication.
- Supports multiple camera inputs specified in the `config.py` file.

### 2. `receiver.py`
- Connects to the WebSocket streams broadcasted by the `broadcaster.py`.
- Displays the received video feed in a graphical user interface (GUI) using tkinter and PIL.
- Handles errors gracefully by displaying an error image if the connection is closed :3
- ! (The ip and ports are hard-coded in this file so it needs to be modified as well) !

### 3. `config.py`
- IP: Assign the ip address of the Rover computer.
- PORTS: Set 4 different ports for the individual cameras.
- CAMERA_INDEX: Starting from 0 set the 4 camera indices.

## Dependencies
- OpenCV (`cv2`)

## Usage
1. **Setting Up Cameras**:
    - Ensure that cameras are connected and accessible on the network.
    - Update the `config.py` file with appropriate camera indices and network details (IP addresses and ports).

2. **Running the Broadcaster**:
    - Execute `broadcaster.py` with Python.
    - The broadcaster will start capturing video from the specified cameras and broadcasting it over WebSockets.

3. **Running the Receiver**:
    - Execute `receiver.py` with Python.
    - The receiver GUI will launch, displaying the video feeds from the cameras.

4. **Interacting with the Receiver**:
    - Use the GUI to view the video feeds from different cameras.
    - Press `Esc` to quit the receiver application.

## Notes
- ENSURE that both broadcaster and receiver are running on the same network to establish WebSocket connections.
- Customize the configuration in `config.py` according to your network setup and camera configurations.
- Additional error handling or features can be added as per specific requirements.

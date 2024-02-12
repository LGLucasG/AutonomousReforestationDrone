# 👨‍💻 Developer documentation

## Path Generation

### BFP Generation
This project contains a library of functions as well as several programs for computing the Back-and-Forth path.

1. The bfp.hpp and bfp.cpp files contain the specifications and implementations of the functions that will be called to generate the optimal path, respectively. The implementation is a different path from what we have seen in the bibliography section. Indeed, although we use a Back-and-Forth path, we have not implemented the FLPMW (Flat Lines Perpendicular to Minimum Width) but a slightly different version. Instead of drawing the flat lines perpendicular to the smallest width of the polygon, we made a simpler implementation that draws the flat lines parallel to the widest side of the polygon. This facilitates the implementation since finding the longest side in a polygon is trivial using the right libraries.

2. The main.cpp file is the main file, the one that will be compiled into a binary to generate the BFP from an image (aerial/satellite view extracted from an online map, for example). It searches for an image in the main folder of the project (bfp_generation/) to use as a basis for field recognition. The program also parses the gps_keypoints.csv file. The GPS coordinates of the points will be recorded in the bfp_gps.csv file.

3. The check_csv.cpp file is another file compiled into a binary that allows for the visualization of the points obtained by the trajectory generation algorithm. More precisely, the program will parse the generated bfp_gps.csv file and convert the GPS coordinates into local coordinates using the GeographicLib library. The points will then be displayed on an image, which allows for the preview of the drone's steps and the actual rendering.

### Drone controller (offboard.cpp)

This code defines a ROS 2 node for offboard control of a drone, specifically designed to work with PX4 autopilot software. The code's primary purpose is to enable a drone to autonomously follow a predefined trajectory based on GPS waypoints, which are read from a CSV file.

1. Initialization: It reads a CSV file containing GPS waypoints (bfp_gps.csv) and converts these waypoints into local coordinates. This conversion is necessary for the drone to follow the trajectory accurately in its local frame of reference.

2. Subscriptions: The node subscribes to the drone's local and global position topics to monitor its current position and adjust the flight path as needed.

3. Publishers: It publishes to topics controlling the drone's offboard control mode, trajectory setpoints, and vehicle commands. These topics are used to switch the drone into offboard mode, send it new positions to move towards, and command actions like arming, disarming, and landing.

4. Main Loop: The node enters a loop where it continually checks the drone's position against the next target position in the trajectory. If the drone is within a specified accuracy of the target position, the node updates the target to the next waypoint. This process involves converting the GPS coordinates of the waypoint into local coordinates, applying any necessary transformations, and publishing the new setpoint. The loop also includes mechanisms for arming the drone, switching to offboard control mode, and eventually landing the drone once all waypoints have been reached.

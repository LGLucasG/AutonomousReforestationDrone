# 📈 Project Report

As a reminder, the objectives of this project were as follows:
- Find a way to **precisely locate** the drone at all times during the planting process
- Make the drone **aware of its environment** using the proper sensors. (Is the terrain suitable for planting? Are there obstacles?)
- **Generate a trajectory** to accomplish the mission, ensuring it is as **efficient** as possible.

This report discusses the status of each objective and what could be done next.

## 📡 Drone localisation

The RTK protocol has been chosen for this project as it enables a more precise positionning of the drone. With the use of the GPS signal coupled with a precisly located radio antenna, we are able to obtain more accurate data than with just the GPS signal.

However, it has not been properly tested yet, as we did not have the opportunity to make experiments in real life (just simulations). This remains to be done.

## 〽️ Trajectory generation

The aim of this part is to control the drone autonomously and cover a field in an optimized way. From the user's point of view, the first step is to locate the field we want to reforest. We then need to find the best way to cover it. As we saw in the bibliography section, the best path is the Back-and-Forth path (BFP). Using image processing algorithms, our first program (`bfp_generation`) allows us to plot the optimal path to cover this field, then returns a file containing the successive positions of the path, in a global frame thanks to a few landmarks. 

We have also programmed the drone's controller, under ROS 2, which enables the drone to take off, follow a series of successive points and land, all autonomously and without requiring any user intervention once the drone has been launched. In this way, the drone controller can be given the file containing the coordinates corresponding to the Back-and-Forth path generated, so that it can cover the field in a simulation.

Currently, it is possible to control the drone in the simulation and to make it cover an entire field. However, the drone is only controlled by position and not by speed, which makes its flight very unstable. The controller tries to reach each position as quickly as possible, resulting in abrupt movements when a new point must be reached. Speed control would be preferable but would require modifying the implementation. A test in real-life conditions remains pending.

The detailed process of use is available in the [user documentation](docs/user).

The detailed implementation is available in the [developer documentation](docs/developer).

## 🎥 Environment awarness

The goal here was to try to install all the necessary dependencies for a RealSense D435 camera. Although we installed everything correctly (to our knowledge at least), the execution of the ROS wrapper continuously logs errors and warning as described in the [Developer Docs](docs/developer), thus making further progress impossible.

The next objective would be to find a fix for this issue, to be able to implement avoidance behaviours when detecting obstacles while following the trajectory defined (cf. above), using algorithms such as rapidly-exploring random trees (RTT).

# 📖 User documentation

[![Watch the video](../../img/previewPath.png)](https://youtu.be/yf7zY1jwji8)

This section describes the process to follow in order to obtain a result such as the one presented in [the video above](https://www.youtube.com/watch?v=FuNU0X7J_0g). 

⚠️ **Note :** Be aware that we are working with **global GPS coordinates**, which means that if you ask the drone to plant trees in Australia from France, it will probably do it. By default, in the simulation, the drone takes off in Zurich Switzerland, Irchel Park. It might be advised to do the tests in that area, or to change the take-off location accordingly.

## ✅ Prerequisites

- **OpenCV**: A library of programming functions mainly aimed at real-time computer vision.
- **GeographicLib**: A library for geographic projections and geodesic calculations.
- **PX4-Autopilot**: An open-source flight control software for drones and other unmanned vehicles.
- **QGroundControl**: A user-friendly ground control station for the configuration and operation of UAVs.

## 📁 Installing the project

## 〽️ Path Generation

### Compute an Optimal Back-and-Forth Path for a Field

This guide outlines the steps to compute an optimal back-and-forth (BFP) path for a specified field area. Follow these steps carefully to achieve accurate results.

#### Process Overview

1. **Prepare the Field Image**:
    - Obtain an image of the field. A screenshot from Google Maps can serve this purpose effectively. Place the image within the `bfp_generation/` directory. Note that all images located in `bfp_generation/img/` have undergone testing.
    - Identify and record the GPS coordinates of two distinct points on this image. Save these coordinates in a file named `gps_keypoints.csv`.

2. **Build the Software**:
    Open a new terminal and execute the following commands to compile the program:
    ```
    mkdir build/
    cd build/
    cmake ..
    make
    ```

3. **Run the Main Program**:
    Execute the compiled program by running:
    ```
    ./main
    ```

4. **Interact with the Program**:
    - Click on the corner of the field within the program's graphical interface.
    - Press `<Enter>` to initiate the computation of the BFP and verify its correctness.
    - Select the two points (previously noted in `gps_keypoints.csv`) on the image to compute the BFP accurately.

5. **Save the Computed Path**:
    - The program will generate and save the back-and-forth path points in a file named `bfp_gps.csv`.

6. **Transfer the Computed Path**:
    - Execute the following command to copy the `bfp_gps.csv` file into your workspace:
    ```
    ../cp_tows.sh
    ```

**CSV File Format**

Ensure that gps_keypoints.csv file adhere to the following format:

```
latitude, longitude
45.189..., 8.159...
```

## 🛫 Ready for Takeoff !

Once everything is set up, you will need 5 different terminals to run the simulation and visualize the drone's trajectory in real-time.

1. Start **MicroXRCEAgent** with the command: 
```
MicroXRCEAgent udp4 -p 8888
```

2. Launch the **QGroundControl** image
```
./QGroundControl.AppImage
```

3. Go to the **PX4-Autopilot** directory and launch it with the command:
```
make -j32 px4_sitl gz_x500
```

4. Launch the **ROS sensor_combined node**:
```
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

5. Wait for sensor data reception in terminal 4, then launch the **drone controller** with:
```
ros2 run px4_ros_com offboard_control
```
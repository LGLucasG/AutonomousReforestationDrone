# 📖 User documentation

[![Watch the video](../../img/previewPath.png)](https://youtu.be/yf7zY1jwji8)

This section describes the process to follow in order to obtain a result such as the one presented in [the video above](https://www.youtube.com/watch?v=FuNU0X7J_0g). 

⚠️ **Note :** Be aware that we are working with **global GPS coordinates**, which means that if you ask the drone to plant trees in Australia from France, it will probably do it. By default, in the simulation, the drone takes off in Zurich Switzerland, Irchel Park. It might be advised to do the tests in that area, or to change the take-off location accordingly.

## ✅ Prerequisites

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

Ensure that CSV files adhere to the following format:

```
latitude, longitude
45.189..., 8.159...
```

**This project requires the following dependencies**:

- **OpenCV**: A library of programming functions mainly aimed at real-time computer vision.
- **GeographicLib**: A library for geographic projections and geodesic calculations.

Ensure you have these libraries installed and properly configured in your environment to successfully compile and run the project.

## 🛫 Ready for Takeoff !
# TurtleBot3 Simulation with IMU Data Processing

## Overview

This project involves deploying a TurtleBot3 simulation in Gazebo, controlling it using the TurtleBot keyboard node, and processing IMU data with Gaussian noise and a Kalman filter. The processed data is then visualized using RViz and rqt_multiplot.

## Steps and Implementation

### 1. Deploy TurtleBot3 Simulation on Gazebo
- **Launch TurtleBot3 Simulation**
- **Control TurtleBot3 Using Keyboard**


### 2. Read IMU Data
- **Use the provided IMU reading node to read data from the TurtleBot3’s IMU sensor.**
- **Convert IMU Readings from Quaternion to Degree**

### 3. Add Gaussian Noise to IMU Data
- **Edit URDF File: Add a Gaussian noise plugin to the URDF file:**

### 4. Implement 1D-Kalman Filter on YAW Angle
- **The Kalman filter node fuses previous and latest YAW angles to produce a smoothed YAW angle.**

### 5. Visualize Filtered and Noisy Data
- **Use rqt_multiplot: Launch rqt_multiplot to visualize the filtered YAW data and the noisy YAW data**
- **Add the topics /filtered_yaw and /conversion to visualize the data.**

## Observations

- **IMU Data Conversion: Successfully converted IMU data from quaternion format to Euler angles (degrees) and published this data.**

- **Gaussian Noise Addition: Integrated noise into the IMU data via URDF configuration, which helps simulate real-world sensor inaccuracies.**

- **Kalman Filter Implementation: Applied a 1D-Kalman filter to the YAW angle data, effectively reducing noise and improving the reliability of the orientation information.**

## Conclusion
 **This project demonstrates an approach to simulating and analyzing TurtleBot3’s sensor data within a Gazebo environment. By incorporating noise into the IMU data and applying a Kalman filter, the project showcases the practical application of sensor fusion techniques to enhance data accuracy. The use of RViz and rqt_multiplot for visualization and analysis ensures effective monitoring and evaluation of the processed data.** 
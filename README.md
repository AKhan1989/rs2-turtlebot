# Robotics Studio 2 - Turtlebot Warehouse Project (rs2-turtlebot)
This git repo holds all the files and folders for Subsystem 2 - Sensors and Control for Localisation and Mapping

## Unit and Integration Tests this repository holds
###1	Unit Test – LiDAR Reading Raw Data Accuracy
  - Ensure correct raw LiDAR data readings for obstacle detection and localisation
  - **lidar/lidar_tests**
### 2	Integration Test – Sensor Data Fusion for Localisation
  - Use LiDAR and Depth Camera to improve localisation accuracy
  - **localisation/sensor_fusion_localisation_tests**
### 3	Unit Test – RGB Colour Perception Test
  - Test whether the Turtlebot3 can scan an object and detect its colour effectively to help with package detection in Subsystem 4
  - **rgb_camera_detect**
### 4	Integration Test – Real Time Obstacle Detection and Mapping
  - Test obstacle-aware SLAM and safety feedback in real time
  - **launch_sensor_fusion_slam**
### 5	Integration Test – Warehouse Map Consistency
  - Test whether the SLAM-generated map is stable and accurate for navigation
  - same as fourth test - **launch_sensor_fusion_slam**

## Final Whole Subsystem Test
Attempted to use the **rs2-turtlebot/launch_sensor_fusion_slam/launch/full_system_launch.py** file as the executable for running all the different sensors and slam toolbox. Slam toolbox had some bugs in terms of running.
### To run this
1. First ensure Turtlebot3 is connected to laptop
3. Setup ros2_ws
4. Git clone repository to device and symlink to the ros2_ws
5. Run RViz2:
      Add Image to the Displays panel
      Open RViz2 with the /camera/image_raw/compressed topic
      Ensure that under global options that the fixed base is “base_link”
      Add Map to the Displays panel
      Ensure topic goes to the one you created (for me I did /map)
      Under the LaserScan in the Displays panel ensure:
         Topic: /scan
         History Policy: Keep Last
         Reliability Policy: Best Effort
         Durability Policy: Volatile
      Ensure map type is an Occupancy Grid
7. In terminal enter:
      cd ros2_ws/
      colcon build --packages-select launch_sensor_fusion_slam
      source ~/ros2_ws/install/setup.bash
      ros2 launch launch_sensor_fusion_slam full_system_launch.py

Should be ready to go for SLAM while incorporating all the other sensors once all these steps are done

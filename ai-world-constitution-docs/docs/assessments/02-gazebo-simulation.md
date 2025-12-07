---
id: gazebo-simulation
title: Gazebo Simulation Assessment
---

# Assessment: Gazebo Simulation Project

This assessment evaluates your ability to create, configure, and interact with a simulated robot in Gazebo, integrated with ROS2.

## Objectives

Upon successful completion of this project, you will have demonstrated the ability to:
*   Create a custom Gazebo world file.
*   Integrate a custom robot model (from URDF) into Gazebo.
*   Configure physical properties for realistic simulation.
*   Implement basic sensor simulation (e.g., a camera or LIDAR).
*   Control the simulated robot using ROS2.
*   Visualize sensor data in RViz2.

## Requirements

Your project must include the following components:

1.  **Custom Gazebo World:**
    *   Create an `.sdf` file for a simple indoor or outdoor environment.
    *   Include at least two different static models (e.g., walls, furniture, obstacles).
    *   Ensure proper lighting and ground plane.

2.  **Custom Robot Model:**
    *   Provide a URDF file for a differential drive or a simple humanoid-like robot (can be a simplified version from Module 1).
    *   Define appropriate `<visual>`, `<collision>`, and `<inertial>` properties for all links.

3.  **Robot Spawning:**
    *   A ROS2 launch file that:
        *   Launches your custom Gazebo world.
        *   Spawns your robot model into the world.
        *   Starts `robot_state_publisher` and `joint_state_publisher` (if applicable).

4.  **Sensor Simulation:**
    *   Integrate at least one simulated sensor on your robot (e.g., `gazebo_ros_camera`, `gazebo_ros_ray_sensor` for LIDAR, or `gazebo_ros_imu_sensor` for IMU).
    *   Ensure the sensor is publishing data to a ROS2 topic.

5.  **Robot Control:**
    *   Implement a basic ROS2 control node (e.g., for teleoperation) that publishes `geometry_msgs/msg/Twist` commands to control your robot in Gazebo.
    *   Alternatively, demonstrate a simple autonomous behavior (e.g., moving in a square) using ROS2.

6.  **Visualization:**
    *   Provide an RViz2 configuration file that displays:
        *   Your robot model.
        *   The sensor data from your simulated sensor (e.g., camera image, LaserScan).

7.  **Documentation:**
    *   A `README.md` file in your package explaining:
        *   The purpose of your project.
        *   How to build and run your project (including launching Gazebo, spawning the robot, and teleoperating it).
        *   A description of your world, robot, and simulated sensor.

## Evaluation Criteria

Your project will be evaluated based on:
*   **Gazebo World Quality:** Realism and complexity of the environment.
*   **Robot Model Quality:** Correctness of URDF, physical properties, and visuals.
*   **ROS2 Integration:** Seamless integration of robot control and sensor simulation with ROS2.
*   **Functionality:** Can the robot be controlled? Does the sensor publish data correctly?
*   **Visualization:** Clarity and correctness of RViz2 setup.
*   **Code Quality:** Readability, comments, and adherence to best practices.
*   **Documentation:** Clarity and completeness of the `README.md`.

## Submission

Submit your ROS2 package containing all necessary files (URDF, SDF world, launch files, control node, RViz2 config) as a compressed archive (`.zip` or `.tar.gz`) or a link to a GitHub repository. Ensure your `README.md` is at the root of your package.

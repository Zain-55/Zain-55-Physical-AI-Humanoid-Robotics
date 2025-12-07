---
id: navigation-nav2
title: Navigation with Nav2
---

# Module 3: NVIDIA Isaac - Navigation with Nav2

Once we have a map of the environment, the next step is to enable the robot to navigate autonomously. The standard tool for this in ROS2 is the Navigation2 stack, also known as Nav2. In this section, we'll learn how to use Nav2 with our simulated robot in Isaac Sim.

## The ROS2 Navigation Stack (Nav2)

Nav2 is a powerful and flexible navigation framework for ROS2. It's a complete rewrite of the original ROS Navigation Stack, designed to be more modular, robust, and suitable for a wider range of robots.

### Key Components of Nav2

*   **Behavior Trees:** Nav2 uses Behavior Trees (BTs) to orchestrate the overall navigation logic. This makes it easy to customize and extend the navigation behavior.
*   **Planners:** Nav2 includes global planners (for finding a path from the robot's current position to a goal) and local planners (for avoiding obstacles while following the global path).
*   **Controllers:** The local planner sends velocity commands to a controller, which is responsible for executing them on the robot's hardware (or in the simulation).
*   **Costmaps:** Nav2 uses two costmaps: a global costmap for the global planner and a local costmap for the local planner. These costmaps represent the "cost" of traversing different parts of the environment, with obstacles having a high cost.
*   **Localization (AMCL):** The Adaptive Monte Carlo Localization (AMCL) package is used to determine the robot's position in the map.

## Setting up Nav2 with Isaac Sim

To use Nav2 with Isaac Sim, you need to:
1.  **Have a map:** You need a map of the environment, which you can generate using a SLAM algorithm as described in the previous section.
2.  **Configure Nav2:** You need to create a set of configuration files for Nav2 that are specific to your robot and environment. This includes setting up the costmaps, choosing planner and controller plugins, and tuning their parameters.
3.  **Launch Nav2:** You'll use a ROS2 launch file to start all the Nav2 nodes, as well as the localization (AMCL) node.
4.  **Connect to Isaac Sim:** The Nav2 stack will communicate with your simulated robot in Isaac Sim through the ROS2 bridge.

## Sending Navigation Goals

Once Nav2 is running, you can send navigation goals to the robot. A navigation goal is simply a target pose (position and orientation) that you want the robot to reach.

You can send navigation goals in several ways:
*   **From RViz2:** The Nav2 plugin for RViz2 provides a "2D Goal Pose" tool that allows you to set a navigation goal by clicking on the map.
*   **From the command line:** You can use the `ros2 action send_goal` command to send a navigation goal from a terminal.
*   **From a ROS2 node:** You can write your own ROS2 node that sends navigation goals programmatically. This is how you would implement a higher-level application, such as a robot that needs to visit a sequence of waypoints.

## Visualizing the Navigation Process

RViz2 is an essential tool for visualizing and debugging the navigation process. You can add displays for:
*   The global and local costmaps.
*   The global path planned by the global planner.
*   The local path planned by the local planner.
*   The robot's current position and orientation.

By visualizing these components, you can gain a deep understanding of how Nav2 is working and diagnose any problems that may arise.

Using Nav2 with Isaac Sim provides a complete, end-to-end simulation environment for developing and testing autonomous navigation. You can go from a completely unknown environment to a fully autonomous robot, all within the safe and realistic world of Isaac Sim.

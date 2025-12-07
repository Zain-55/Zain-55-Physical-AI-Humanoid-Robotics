---
id: gazebo-basics
title: Gazebo Basics
---

# Module 2: Digital Twin - Gazebo Basics

In this module, we shift our focus to creating a "digital twin" of our robot, a virtual representation that we can use for testing, development, and training. Our primary tool for this will be Gazebo, a powerful and widely-used robotics simulator.

## What is Gazebo?

Gazebo is an open-source 3D robotics simulator that allows you to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. It's tightly integrated with ROS2, making it an essential tool for robotics development.

## Why Use a Simulator?

Simulation is a critical part of modern robotics for several reasons:
*   **Safety:** You can test new algorithms and behaviors without risking damage to a physical robot or its environment.
*   **Cost-Effectiveness:** Developing in simulation is often cheaper and faster than working with physical hardware.
*   **Parallelization:** You can run many simulations in parallel to test different scenarios or train machine learning models.
*   **Accessibility:** Not everyone has access to a physical robot, but anyone can use a simulator.

## Gazebo Architecture

Gazebo has a client/server architecture:
*   **Gazebo Server (`gzserver`):** This is the core of Gazebo. It runs the physics simulation, generates sensor data, and accepts commands from clients. It can be run headless (without a graphical interface).
*   **Gazebo Client (`gzclient`):** This is the graphical interface for Gazebo. It visualizes the simulation and allows you to interact with it.

This separation allows you to run the computationally intensive simulation on a powerful machine, while viewing and interacting with it from a less powerful one.

## The Gazebo User Interface

When you launch Gazebo, you are presented with a 3D environment. Here are some of the key components of the UI:
*   **3D View:** The main window where you can see and interact with the simulated world.
*   **Scene Panel:** On the left, this panel shows a tree view of all the models in the world.
*   **Control Panel:** At the top, this panel has buttons to play, pause, and step the simulation, as well as tools to manipulate objects.

## Spawning a Robot in Gazebo

To use your robot in Gazebo, you need to "spawn" its model into the simulated world. This is typically done using a ROS2 launch file.

Here is an example of a launch file that spawns a robot from a URDF file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn the robot from a URDF file
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', '/path/to/your/robot.urdf'],
            output='screen'
        ),
    ])
```

## Controlling a Robot in Gazebo

Once your robot is in Gazebo, you can control it using ROS2. Gazebo provides a set of plugins that expose ROS2 interfaces for your robot.

For example, the `gazebo_ros_diff_drive` plugin allows you to control a differential drive robot by publishing `geometry_msgs/Twist` messages to a topic (usually `/cmd_vel`).

By combining the power of ROS2 for control and Gazebo for simulation, you can create a complete development environment for your robotics projects, all without needing physical hardware to get started.

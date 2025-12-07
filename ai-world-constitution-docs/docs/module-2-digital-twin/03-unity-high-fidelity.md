---
id: unity-high-fidelity
title: Unity for High-Fidelity Simulation
---

# Module 2: Digital Twin - Unity for High-Fidelity Simulation

While Gazebo is a powerful tool for physics simulation, sometimes you need a higher level of visual fidelity. This is where a game engine like Unity can be a valuable addition to your robotics toolkit. Unity is known for its stunning graphics, and it's increasingly being used for robotics simulation, especially for training vision-based machine learning models.

## Advantages of Unity for Simulation

*   **Photorealistic Graphics:** Unity's High Definition Render Pipeline (HDRP) enables you to create incredibly realistic scenes with advanced lighting, materials, and post-processing effects.
*   **Rich Asset Store:** The Unity Asset Store provides a vast library of 3D models, textures, and environments that you can use to build your simulation.
*   **Advanced Authoring Tools:** Unity's editor is a powerful and intuitive tool for designing and building complex scenes.
*   **Cross-Platform Support:** Unity can deploy to a wide range of platforms, which can be useful for creating robotics-related applications like virtual reality control interfaces.

## The Unity Robotics Hub

To make it easier to use Unity for robotics, the Unity team has created the **Unity Robotics Hub**. This is a set of packages that provide tools and utilities for robotics simulation in Unity.

### URDF Importer

The URDF Importer package allows you to import your robot's URDF file directly into Unity. It will automatically create the robot's links and joints, and you can then configure its physical properties within the Unity editor.

### ROS-TCP-Connector

The ROS-TCP-Connector package provides a bridge between Unity and ROS2. It allows your Unity simulation to communicate with ROS2 nodes using topics and services. This means you can use the same ROS2 control code for both your Gazebo simulation and your Unity simulation.

## Setting up a Unity Project for ROS2

Here is a general workflow for setting up a Unity project for robotics simulation:
1.  **Create a new Unity project** using the 3D (HDRP) template.
2.  **Install the Unity Robotics Hub packages** from the Unity Asset Store or GitHub.
3.  **Import your robot's URDF file** using the URDF Importer.
4.  **Configure the robot's joints and articulations** in the Unity editor.
5.  **Add the ROS-TCP-Connector** to your scene and configure it to connect to your ROS2 network.
6.  **Create C# scripts** in Unity to handle the communication with ROS2 (e.g., subscribing to `/cmd_vel` to control the robot).

## Example: Controlling a Robot from ROS2

Here is a simplified example of a C# script in Unity that subscribes to a `geometry_msgs/Twist` message and applies the corresponding forces to a robot's wheels.

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;
    public float maxLinearSpeed = 2.0f; // m/s
    public float maxAngularSpeed = 1.0f; // rad/s
    public float wheelRadius = 0.1f; // meters
    public float trackWidth = 0.5f; // meters

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>("/cmd_vel", OnCmdVelReceived);
    }

    void OnCmdVelReceived(TwistMsg twistMsg)
    {
        float linear = (float)twistMsg.linear.x;
        float angular = (float)twistMsg.angular.z;

        // Convert linear and angular velocities to wheel velocities
        float v_left = (linear - angular * trackWidth / 2.0f) / wheelRadius;
        float v_right = (linear + angular * trackWidth / 2.0f) / wheelRadius;
        
        // Apply the velocities to the wheels
        var leftDrive = leftWheel.xDrive;
        leftDrive.targetVelocity = v_left * Mathf.Rad2Deg;
        leftWheel.xDrive = leftDrive;

        var rightDrive = rightWheel.xDrive;
        rightDrive.targetVelocity = v_right * Mathf.Rad2Deg;
        rightWheel.xDrive = rightDrive;
    }
}
```

This script demonstrates the basic principle of using the ROS-TCP-Connector to bridge the gap between Unity's simulation environment and the ROS2 control system.

By leveraging Unity's high-fidelity rendering capabilities, you can create a digital twin that is not only physically accurate but also visually indistinguishable from reality. This opens up new possibilities for training and testing perception algorithms, as well as for creating compelling robotics demonstrations and user interfaces.
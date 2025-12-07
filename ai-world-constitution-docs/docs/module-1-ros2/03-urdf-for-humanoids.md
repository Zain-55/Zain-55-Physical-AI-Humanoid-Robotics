---
id: urdf-for-humanoids
title: URDF for Humanoids
---

# Module 1: ROS2 - URDF for Humanoids

A crucial part of robotics is modeling the robot itself. In ROS2, the standard for this is the Unified Robot Description Format (URDF). This section will introduce you to URDF and how to use it to model a humanoid robot.

## What is URDF?

URDF is an XML format used in ROS to describe all elements of a robot. It represents the robot's physical structure, including its links, joints, and sensors. A well-defined URDF is essential for simulation, visualization, and collision detection.

## Key Elements of URDF

A URDF file is made up of a few key tags:

*   `<robot>`: The root tag of the URDF file. It encloses the entire robot description.
*   `<link>`: A rigid body of the robot. It has physical properties like mass, inertia, and a visual representation.
*   `<joint>`: A connection between two links. It defines the type of motion allowed between the links (e.g., revolute, prismatic, fixed).

### Link Properties

A `<link>` tag can have several child elements:
*   `<visual>`: Defines the visual appearance of the link (shape, color, texture).
*   `<collision>`: Defines the collision geometry of the link, used for physics simulation.
*   `<inertial>`: Defines the dynamic properties of the link (mass and inertia).

### Joint Properties

A `<joint>` tag connects two links (`<parent>` and `<child>`) and defines their relationship:
*   `type`: The type of joint. Common types include:
    *   `revolute`: A rotational joint with a single axis of rotation.
    *   `continuous`: Similar to revolute, but with no angle limits.
    *   `prismatic`: A sliding joint with a single axis of translation.
    *   `fixed`: A joint that does not allow any motion.
*   `<origin>`: A transform that defines the position and orientation of the joint's frame relative to the parent link's frame.
*   `<axis>`: The axis of rotation or translation for revolute and prismatic joints.
*   `<limit>`: Defines the limits of motion for the joint (e.g., upper and lower angle limits for a revolute joint).

## Creating a Simple Humanoid URDF

Let's create a very simple URDF for a humanoid robot. This will include a torso, a head, and two arms.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Torso Link -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.4 0.2 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Head Link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting Torso and Head -->
  <joint name="neck" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
  </joint>

  <!-- Right Arm Link -->
  <link name="right_arm">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting Torso and Right Arm -->
  <joint name="right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_arm"/>
    <origin xyz="0 -0.15 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

</robot>
```

This simple URDF defines a robot with a torso, a head fixed to the torso, and a right arm connected with a revolute joint (allowing it to move up and down).

## Visualizing URDF with RViz2

RViz2 is the primary 3D visualization tool in ROS2. You can use it to display your robot's model, sensor data, and more.

To visualize your URDF model, you need to:
1.  **Create a ROS2 package** for your robot description.
2.  **Include the URDF file** in your package.
3.  **Create a launch file** that starts the `robot_state_publisher` and RViz2.

The `robot_state_publisher` reads the URDF file and publishes the state of the robot (the transforms between the links) to the `/tf` topic. RViz2 then subscribes to this topic to display the robot model.

## Using Xacro for More Complex Models

For more complex robots, writing URDF files by hand can be tedious and error-prone. **Xacro** (XML Macros) is a tool that allows you to create more modular and reusable URDF files.

With Xacro, you can:
*   Define constants and use them throughout your files.
*   Create macros for recurring elements (like an arm or a leg).
*   Include other xacro or URDF files.

Using Xacro is the standard practice for any non-trivial robot model in ROS2. It helps keep your robot description organized and easy to maintain.

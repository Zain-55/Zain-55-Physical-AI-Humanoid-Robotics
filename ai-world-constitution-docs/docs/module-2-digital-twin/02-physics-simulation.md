---
id: physics-simulation
title: Physics Simulation
---

# Module 2: Digital Twin - Physics Simulation

A key feature of a robotics simulator like Gazebo is its ability to simulate physics. This allows us to test not just the robot's control logic, but also its physical interaction with the world. In this section, we'll dive into how physics simulation works in Gazebo.

## Introduction to Physics Engines

A physics engine is the component of a simulator that calculates the motion and interaction of objects. It's responsible for things like gravity, collisions, and forces. Gazebo supports several different open-source physics engines, each with its own strengths and weaknesses.

## Gazebo's Physics Engines

Gazebo's default physics engine is **ODE (Open Dynamics Engine)**, but it also supports others like **Bullet**, **DART**, and **Simbody**. You can choose the physics engine that best suits your needs. For most common robotics applications, ODE provides a good balance of performance and accuracy.

## Configuring Physics Properties

To get a realistic simulation, you need to provide accurate physical properties for your robot and the objects in its environment. This is done within the URDF or SDF (Simulation Description Format) file.

### Inertial Properties

The `<inertial>` tag is used to define the mass and rotational inertia of a link.

```xml
<link name="my_link">
  <inertial>
    <mass value="1.0" />
    <inertia ixx="0.083" ixy="0.0" ixz="0.0" iyy="0.083" iyz="0.0" izz="0.083" />
  </inertial>
  ...
</link>
```
*   `mass`: The mass of the link in kilograms.
*   `inertia`: The 3x3 rotational inertia matrix. This describes how the link's mass is distributed around its center of mass.

Getting accurate inertial properties is crucial for simulating realistic robot dynamics.

### Collision Geometry

The `<collision>` tag defines the shape of the link for the physics engine. This can be a simple shape (like a box, cylinder, or sphere) or a more complex mesh. It's important to keep collision geometries as simple as possible to improve simulation performance.

```xml
<link name="my_link">
  <collision>
    <geometry>
      <box size="0.1 0.1 0.1" />
    </geometry>
  </collision>
  ...
</link>
```

### Contact Properties

You can also define the surface properties of a link's collision geometry. This is done within the `<collision>` tag using the `<surface>` element.

```xml
<collision>
  ...
  <surface>
    <contact>
      <ode>
        <kp>1000000.0</kp>
        <kd>1.0</kd>
        <max_vel>0.01</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```
*   `kp` and `kd`: Stiffness and damping of the contact.
*   `mu` and `mu2`: Coefficients of friction.

These parameters determine how the link interacts with other objects when they come into contact.

## Simulation Parameters

Gazebo has a number of global parameters that affect the physics simulation:
*   **Time Step:** The amount of time the simulation advances with each iteration. A smaller time step leads to a more accurate simulation, but requires more computation.
*   **Solver Iterations:** The number of iterations the physics solver runs for each time step. More iterations can improve the stability of the simulation, especially for complex scenes with many contacts.

These parameters can be configured in the Gazebo world file or through the Gazebo GUI.

## Example: Simulating a Falling Object

To see the physics engine in action, you can create a simple world file with a box and let it fall onto the ground plane.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <model name="falling_box">
      <pose>0 0 2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```
When you load this world in Gazebo, you will see the box start at a height of 2 meters and fall to the ground due to gravity.

By carefully defining the physical properties of your robot and its environment, you can create a highly realistic simulation that will be invaluable for developing and testing your robotics software.
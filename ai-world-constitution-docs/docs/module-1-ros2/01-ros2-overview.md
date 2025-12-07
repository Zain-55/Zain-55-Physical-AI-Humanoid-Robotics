---
id: ros2-overview
title: ROS2 Overview
---

# Module 1: ROS2 - Overview

Welcome to the first module of our journey into Physical AI and Humanoid Robotics. This module is dedicated to the Robot Operating System 2 (ROS2), the foundational software framework we will use to build our intelligent robotics applications.

## What is ROS2?

ROS2 is an open-source, flexible framework for writing robot software. It's a set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS2 has what you need for your next robotics project.

## Key Concepts in ROS2

ROS2 is built upon a few core concepts that work together to enable communication and data transfer between different parts of the robotics system.

### Data Distribution Service (DDS)

ROS2 is built on top of DDS, a middleware standard for data-centric publish-subscribe messaging. DDS provides a robust and efficient communication layer, which is one of the key improvements of ROS2 over ROS1. This allows for real-time communication, better performance, and the ability to use ROS2 in commercial and mission-critical systems.

### Nodes

A node is the smallest unit of computation in ROS2. Each node should be responsible for a single, well-defined task (e.g., controlling a wheel, reading a sensor, planning a path). By breaking down a complex system into many modular nodes, the system becomes easier to manage, debug, and reuse.

### Topics

Topics are named buses over which nodes exchange messages. Topics use a publish/subscribe model. A node can publish a message to a topic, and any number of nodes can subscribe to that topic to receive the message. This is a one-to-many communication mechanism.

### Services

Services are a request/response communication mechanism. One node (the client) sends a request to another node (the server) and waits for a response. This is a one-to-one communication pattern, ideal for remote procedure calls.

### Actions

Actions are used for long-running tasks. They are similar to services but provide feedback during execution and are preemptible (can be cancelled). This is useful for tasks like navigation, where the robot needs to provide updates on its progress and may need to stop or change its goal.

## ROS2 Architecture

The architecture of ROS2 is designed to be distributed and modular. A ROS2 system typically consists of several nodes, each running as a separate process. These nodes communicate with each other using topics, services, and actions.

This distributed nature allows for:
*   **Scalability:** You can add more computational power by adding more computers to the network.
*   **Fault Tolerance:** If one node crashes, the rest of the system can continue to function.
*   **Flexibility:** Different parts of the system can be written in different programming languages (like Python and C++).

## Differences between ROS1 and ROS2

ROS2 is a complete redesign of ROS1, with many significant improvements:
*   **Multi-robot support:** ROS2 is designed from the ground up to support systems with multiple robots.
*   **Real-time capabilities:** The use of DDS allows for real-time control and communication.
*   **Improved security:** ROS2 includes a comprehensive security system to protect against unauthorized access and modification of data.
*   **Support for smaller devices:** ROS2 can run on microcontrollers and other embedded systems.
*   **Official Windows support:** ROS2 is officially supported on Windows, in addition to Linux and macOS.

## Installation and Setup

Before you can start using ROS2, you need to install it on your system. The official ROS2 documentation provides detailed instructions for different platforms. We recommend using a supported LTS (Long Term Support) version of Ubuntu with the corresponding ROS2 release.

For this course, we will be using **ROS2 Humble Hawksbill**, which is the recommended version for Ubuntu 22.04.

In the next sections, we will dive deeper into the practical aspects of creating and using ROS2 nodes, topics, and services.

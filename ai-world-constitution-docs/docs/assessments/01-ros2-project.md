---
id: ros2-project
title: ROS2 Project Assessment
---

# Assessment: ROS2 Project

This assessment evaluates your understanding and practical application of fundamental ROS2 concepts, including nodes, topics, services, and message types.

## Objectives

Upon successful completion of this project, you will have demonstrated the ability to:
*   Create and manage ROS2 nodes.
*   Implement publish/subscribe communication using ROS2 topics.
*   Implement request/response communication using ROS2 services.
*   Define and utilize custom ROS2 message types.
*   Utilize ROS2 launch files to orchestrate multiple nodes.
*   Debug ROS2 applications using command-line tools.

## Requirements

Your project must include the following components:

1.  **Multiple Nodes:**
    *   At least three distinct ROS2 nodes working together.
    *   Each node should have a clear, single responsibility.

2.  **Topics:**
    *   Implement at least one publish/subscribe mechanism.
    *   Use a custom message type (defined by you) for communication on this topic.
    *   One node should publish data of your custom message type, and another should subscribe and process it.

3.  **Services:**
    *   Implement at least one ROS2 service.
    *   One node should act as a service server, and another node should act as a client calling this service.
    *   The service should perform a meaningful operation relevant to a robotics context (e.g., `calculate_distance`, `get_robot_status`).

4.  **Launch File:**
    *   Provide a single ROS2 launch file that starts all your nodes and any necessary static transforms or configurations.

5.  **Documentation:**
    *   A `README.md` file in your package explaining:
        *   The purpose of your project.
        *   How to build and run your project.
        *   A description of each node, topic, and service.
        *   An explanation of your custom message type.

## Example Project Ideas (Choose ONE or propose your own)

*   **Simple Robot Controller:** A node publishing robot commands, a node subscribing to sensor data, and a service to switch modes (e.g., "manual" vs. "autonomous").
*   **Data Logger/Analyzer:** A node publishing dummy sensor data, a node subscribing and logging it to a file, and a service to request data statistics.

## Evaluation Criteria

Your project will be evaluated based on:
*   **Correctness:** Does the code work as expected? Are ROS2 communication patterns correctly implemented?
*   **Modularity:** Are the nodes well-separated by responsibility?
*   **Custom Message Implementation:** Is the custom message type correctly defined and used?
*   **Launch File Correctness:** Does the launch file correctly start and configure all components?
*   **Code Quality:** Readability, comments (where necessary), adherence to Python/C++ best practices.
*   **Documentation:** Clarity and completeness of the `README.md`.

## Submission

Submit your ROS2 package as a compressed archive (`.zip` or `.tar.gz`) or a link to a GitHub repository. Ensure your `README.md` is at the root of your package.

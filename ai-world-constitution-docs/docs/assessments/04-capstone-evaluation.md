---
id: capstone-evaluation
title: Capstone Project Evaluation
---

# Assessment: Capstone Project Evaluation

The Capstone Project is the culmination of your learning throughout the course. It provides an opportunity to integrate knowledge and skills acquired from all modules (ROS2, Digital Twin, NVIDIA Isaac, and VLA) into a comprehensive project. This evaluation outlines the expectations for your final submission.

## Overall Goal

Design, implement, and demonstrate an autonomous humanoid robot system that leverages the tools and concepts learned, with a strong emphasis on integrating voice commands and intelligent decision-making.

## Objectives

Upon successful completion of the Capstone Project, you will have demonstrated the ability to:
*   Integrate multiple ROS2 components (nodes, topics, services, actions).
*   Utilize advanced simulation environments (Gazebo or Isaac Sim) for development and testing.
*   Implement perception and navigation capabilities for a humanoid robot.
*   Develop a Voice-to-Action (VLA) system that enables natural language interaction.
*   Apply cognitive planning techniques for multi-step tasks.
*   Present and document a complex robotics system effectively.

## Core Requirements (Minimum Viable Product)

Your Capstone Project must include the following:

1.  **Humanoid Robot Integration:**
    *   A simulated humanoid robot (or a robot with humanoid-like characteristics, e.g., a torso with an arm, or a mobile robot with a manipulator).
    *   The robot must be spawnable and controllable within a simulation environment (Gazebo or Isaac Sim).

2.  **Perception Capabilities:**
    *   The robot must be able to perceive its environment using simulated sensors (e.g., cameras, LIDAR).
    *   Demonstrate a basic perception task (e.g., object detection, obstacle avoidance).

3.  **Navigation/Mobility (if applicable):**
    *   If your robot has mobility, it must be able to navigate to a specified goal in the simulated environment.

4.  **Voice-to-Action (VLA) System:**
    *   Implement an end-to-end VLA pipeline that accepts a spoken command from a user.
    *   The VLA system must parse the command and translate it into a robot action or a sequence of actions.
    *   Demonstrate at least one multi-step task initiated by a high-level voice command (e.g., "Go to the blue table and pick up the red box").

5.  **ROS2 Architecture:**
    *   The entire system must be built using ROS2, with clear communication between different components (nodes, topics, services, actions).

## Deliverables

1.  **Project Report (PDF):**
    *   **Introduction:** Project overview, objectives, and problem statement.
    *   **System Architecture:** Detailed description of your ROS2 architecture (nodes, topics, services, data flow, LLM integration, etc.). Use diagrams.
    *   **Implementation Details:** Explain key algorithms, code structure, and any notable challenges or solutions.
    *   **Demonstration/Results:** Describe how your system fulfills the requirements, including screenshots/videos from the simulation.
    *   **Conclusion & Future Work:** Summarize your findings and suggest potential improvements or extensions.

2.  **Code Repository (GitHub Link):**
    *   A well-organized GitHub repository containing all source code (ROS2 packages, URDFs, Isaac Sim assets/scripts if applicable, launch files, etc.).
    *   A comprehensive `README.md` at the root of the repository with:
        *   Project title and description.
        *   Instructions on how to set up the environment and run your project.
        *   Dependencies.
        *   Explanation of commands the robot understands.

3.  **Video Demonstration (5-7 minutes):**
    *   A clear video demonstrating your robot performing the specified tasks in the simulated environment, responding to voice commands.
    *   The video should highlight the VLA interaction and the robot's autonomous behavior.
    *   Narration explaining what is happening.

## Evaluation Criteria

Your Capstone Project will be evaluated based on:
*   **Functionality (40%):** How well the system meets the core requirements and performs the demonstrated tasks.
*   **System Design & Architecture (20%):** Clarity, modularity, and correctness of the ROS2 design.
*   **Innovation & Complexity (15%):** Creativity in problem-solving, use of advanced techniques, and overall scope of the project.
*   **Documentation & Presentation (15%):** Quality of the project report, code readability, and video clarity.
*   **Voice-to-Action Integration (10%):** Effectiveness and robustness of the VLA system.

## Submission

Submit your Project Report (PDF) and the link to your GitHub repository (which should contain your video demonstration) through the designated submission portal.

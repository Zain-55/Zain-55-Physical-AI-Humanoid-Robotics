---
id: cognitive-planning
title: Cognitive Planning
---

# Module 4: VLA - Cognitive Planning

The simple keyword-based NLU we discussed in the previous section is limited to understanding very basic commands. To enable our robot to perform more complex, multi-step tasks, we need a more advanced approach: **cognitive planning**. This is where we can leverage the power of Large Language Models (LLMs).

## Beyond Simple Commands

Imagine telling your robot, "Go to the kitchen and get me a soda." A simple NLU would fail to understand this command. It requires:
1.  Breaking down the command into a sequence of steps.
2.  Knowing what "kitchen" and "soda" are, and where to find them.
3.  Having a plan for how to navigate to the kitchen, find the soda, pick it up, and bring it back.

This is the realm of cognitive planning.

## Using Large Language Models (LLMs) for Planning

LLMs like GPT-3 have shown remarkable abilities in planning and reasoning. We can use an LLM to take a high-level command and generate a sequence of robot actions to achieve it.

The process looks like this:
1.  The user gives a high-level command (e.g., "Get me a soda").
2.  We send this command to an LLM, along with a "prompt" that gives the LLM context about the robot and its capabilities.
3.  The LLM returns a plan, which is a sequence of actions (e.g., `["go_to('kitchen')", "find('soda')", "pick_up('soda')", "go_to('user')"]`).
4.  Our ROS2 system then executes this plan one action at a time.

## Prompt Engineering for Robotics

**Prompt engineering** is the art of designing the right input to an LLM to get the desired output. For a robotics task, a good prompt might include:
*   A description of the robot.
*   A list of the robot's available actions (e.g., `go_to(location)`, `pick_up(object)`).
*   Information about the environment (e.g., a list of known locations and objects).
*   A few examples of commands and the corresponding plans.

Here is a simplified example of a prompt:

```
You are a helpful robot assistant. You can perform the following actions:
- go_to(location): Navigates to a location.
- find(object): Looks for an object.
- pick_up(object): Picks up an object.
- put_down(object, location): Puts an object down at a location.

The user gives you a command. Your task is to generate a plan to execute the command.

Command: "Bring me the apple from the table."
Plan: ["go_to('table')", "find('apple')", "pick_up('apple')", "go_to('user')"]

Command: "Tidy up the living room."
Plan: ["go_to('living_room')", "find('toy')", "pick_up('toy')", "go_to('toy_box')", "put_down('toy', 'toy_box')"]

Command: "Get me a soda."
Plan:
```

We would then send this entire text to the LLM, and it would generate the plan for the "Get me a soda" command.

## Integrating an LLM with ROS2

To integrate an LLM with ROS2, you can create a ROS2 service that takes a command as input and returns a plan. This service would:
1.  Construct the prompt.
2.  Call the LLM's API with the prompt.
3.  Parse the LLM's response to extract the plan.
4.  Return the plan to the client.

The client would then be responsible for executing the plan, for example by calling the appropriate ROS2 action servers for each step in the plan.

By using an LLM for cognitive planning, we can create a much more powerful and flexible VLA system, one that can understand and execute a wide range of complex, multi-step commands. This is a key step towards creating truly intelligent and helpful robotic assistants.

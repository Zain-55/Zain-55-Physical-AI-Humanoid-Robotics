---
id: voice-to-action
title: Voice-to-Action (VLA)
---

# Module 4: VLA - Voice-to-Action

In this module, we will explore the exciting field of Voice-to-Action (VLA), which is all about enabling robots to understand and respond to human voice commands. This is a crucial step towards creating truly interactive and intelligent robots.

## What is a Voice-to-Action (VLA) System?

A VLA system is a pipeline that takes a spoken command from a user, understands the user's intent, and translates that intent into a concrete action for the robot to perform. This goes beyond simple speech recognition; it involves understanding the meaning behind the words.

## Key Components of a VLA System

A typical VLA system has three main components:

### 1. Speech-to-Text (STT)

The first step is to convert the user's spoken language into text. This is done by a Speech-to-Text (STT) engine. There are many STT engines available, from cloud-based services like Google Speech-to-Text to open-source libraries that can run locally.

### 2. Natural Language Understanding (NLU)

Once we have the text of the command, we need to understand what it means. This is the job of the Natural Language Understanding (NLU) component. The NLU's goal is to extract the user's **intent** and any relevant **entities**.

*   **Intent:** What the user wants the robot to do (e.g., `move`, `pick_up`, `go_to`).
*   **Entities:** The objects or parameters associated with the intent (e.g., `forward`, `the red block`, `the kitchen`).

### 3. Action Mapping

The final step is to map the extracted intent and entities to a specific action for the robot to perform. This could be publishing a message to a ROS2 topic, calling a ROS2 service, or sending a goal to a ROS2 action server.

## A Simple VLA Pipeline

Let's look at how to set up a basic VLA pipeline in Python.

### Using `SpeechRecognition` for STT

The `SpeechRecognition` library in Python is a convenient wrapper for various STT engines.

```python
import speech_recognition as sr

r = sr.Recognizer()
with sr.Microphone() as source:
    print("Say something!")
    audio = r.listen(source)

try:
    text = r.recognize_google(audio)
    print("You said: " + text)
except sr.UnknownValueError:
    print("Could not understand audio")
except sr.RequestError as e:
    print("Could not request results; {0}".format(e))
```

### Keyword-Based NLU

For a simple NLU, we can just look for keywords in the recognized text.

```python
def simple_nlu(text):
    if "forward" in text:
        return {"intent": "move", "direction": "forward"}
    elif "backward" in text:
        return {"intent": "move", "direction": "backward"}
    else:
        return {"intent": "unknown"}
```

### Mapping to ROS2 Actions

Finally, we can map the output of our NLU to a ROS2 action. In this case, we'll publish a `geometry_msgs/Twist` message to control the robot's movement.

```python
# (Assuming you have a ROS2 node and publisher set up)
# self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

def execute_action(nlu_result):
    if nlu_result["intent"] == "move":
        twist = Twist()
        if nlu_result["direction"] == "forward":
            twist.linear.x = 0.5
        elif nlu_result["direction"] == "backward":
            twist.linear.x = -0.5
        self.publisher_.publish(twist)
```

This simple example demonstrates the basic principles of a VLA system. In the following sections, we will explore more advanced techniques for NLU and planning, which will allow our robot to understand much more complex commands.

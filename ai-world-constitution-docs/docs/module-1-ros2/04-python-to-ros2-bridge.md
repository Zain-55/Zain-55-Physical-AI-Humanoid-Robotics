---
id: python-to-ros2-bridge
title: Python to ROS2 Bridge
---

# Module 1: ROS2 - Python to ROS2 Bridge

While ROS2 is a powerful framework, you may sometimes need to interface it with other existing Python applications or libraries that are not built on ROS2. The `ros2-bridge` provides a straightforward way to connect standard Python applications with the ROS2 ecosystem.

## Why Use a Bridge?

A bridge is necessary when you want to:
*   **Integrate a non-ROS2 Python application** with a ROS2 system. For example, a web server, a GUI, or a data analysis script.
*   **Leverage Python libraries** that are not available as ROS2 packages.
*   **Prototype and test** parts of your system in a standard Python environment before integrating them into a full ROS2 system.

The `ros2-bridge` allows you to publish and subscribe to ROS2 topics, and call ROS2 services, from a regular Python script without needing to create a full ROS2 node.

## How it Works

The `ros2-bridge` works by running a separate process that acts as a proxy between the ROS2 world and your Python application. Communication between your Python script and the bridge is typically done using a simple protocol over a socket connection.

This approach allows your Python application to remain independent of the ROS2 event loop (`rclpy.spin()`), which is often a blocking call.

## rosbridge_suite: The Standard Solution

The `rosbridge_suite` is a package for ROS that provides a JSON API to ROS functionality for non-ROS programs. It's the most common and well-supported way to create a bridge.

`rosbridge_suite` consists of several packages, but the most important one is `rosbridge_server`. This package starts a WebSocket server that allows you to send JSON messages to interact with ROS2.

### Installation

You can install `rosbridge_suite` for your ROS2 distribution using your system's package manager:

```bash
sudo apt-get install ros-humble-rosbridge-server
```

### Running the Bridge

To start the bridge, you run the `rosbridge_websocket` launch file:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

This will start a WebSocket server, typically on port 9090.

## Example: A Simple Python Client

Here is a simple example of how to publish to a ROS2 topic from a standard Python script using the `rosbridge_suite`. This example uses the `websocket-client` library.

First, make sure you have the library installed:
```bash
pip install websocket-client
```

Now, you can write a Python script to connect to the `rosbridge` WebSocket and publish a message:

```python
import websocket
import json

def publish_chatter():
    ws = websocket.create_connection("ws://localhost:9090")

    # The message to publish
    msg = {
        "op": "publish",
        "topic": "/chatter",
        "msg": {
            "data": "Hello from a Python script!"
        }
    }

    try:
        ws.send(json.dumps(msg))
        print("Message sent")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        ws.close()
        print("Connection closed")


if __name__ == "__main__":
    # Make sure you have a ROS2 listener running on the /chatter topic
    # For example: ros2 topic echo /chatter
    publish_chatter()

```

This script connects to the `rosbridge` server, constructs a JSON message to publish to the `/chatter` topic, and sends it.

### Subscribing to a Topic

Subscribing to a topic is also straightforward. You send a message with `op: "subscribe"` and the topic name. The `rosbridge` server will then start sending you messages from that topic.

```python
import websocket
import json
import threading

def on_message(ws, message):
    data = json.loads(message)
    print(f"Received from /chatter: {data['msg']['data']}")

def on_error(ws, error):
    print(f"Error: {error}")

def on_close(ws, close_status_code, close_msg):
    print("### closed ###")

def on_open(ws):
    def run(*args):
        # Subscribe to the /chatter topic
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/chatter"
        }
        ws.send(json.dumps(subscribe_msg))
        print("Subscribed to /chatter")
    
    thread = threading.Thread(target=run)
    thread.start()

if __name__ == "__main__":
    websocket.enableTrace(True)
    ws = websocket.WebSocketApp("ws://localhost:9090",
                              on_open=on_open,
                              on_message=on_message,
                              on_error=on_error,
                              on_close=on_close)

    ws.run_forever()
```

This more advanced example shows how to create a persistent WebSocket client that subscribes to a topic and prints the messages it receives.

By using the `ros2-bridge`, you can unlock a great deal of flexibility, allowing you to integrate ROS2 with a wide variety of other tools and applications in the Python ecosystem.

---
id: nodes-topics-services
title: Nodes, Topics, & Services
---

# Module 1: ROS2 - Nodes, Topics, and Services

In this section, we will explore the fundamental building blocks of a ROS2 application: nodes, topics, and services. Understanding these concepts is essential for building any robotics application with ROS2.

## ROS2 Nodes

A node is an executable that uses ROS2 to communicate with other nodes. In a real-world robot, you might have a node for the camera, a node for the wheel motors, a node for path planning, and so on. Each node is a small, independent program that performs a specific task.

### Creating a ROS2 Node in Python

Here is a simple example of a ROS2 node written in Python. This node initializes itself and then does nothing, but it's a good starting point.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('MyNode has been started')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
To run this node, you would save it as a Python file, and then run it using `ros2 run <package_name> <node_name>`.

## ROS2 Topics

Topics are a vital part of ROS2's communication system. They are named buses that nodes can use to exchange data. Topics work on a publish/subscribe model.

### Publisher/Subscriber Model

*   **Publisher:** A node that pushes data to a topic.
*   **Subscriber:** A node that receives data from a topic.

A topic can have multiple publishers and multiple subscribers. This allows for a flexible, decoupled architecture where nodes don't need to know about each other directly.

### Creating a Publisher in Python

Here is an example of a node that publishes a simple string message to a topic called `chatter`.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ChatterPublisher(Node):
    def __init__(self):
        super().__init__('chatter_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = ChatterPublisher()
    rclpy.spin(publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Subscriber in Python

And here is the corresponding subscriber node that listens to the `chatter` topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ChatterSubscriber(Node):
    def __init__(self):
        super().__init__('chatter_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    subscriber = ChatterSubscriber()
    rclpy.spin(subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS2 Services

Services are another way for nodes to communicate, but they use a request/response model. One node acts as a service server, and another acts as a service client.

### Request/Response Model

*   **Service Server:** A node that advertises a service and provides a response when it receives a request.
*   **Service Client:** A node that calls a service by sending a request and waiting for the response.

This is a one-to-one communication mechanism, suitable for remote procedure calls (RPCs).

### Creating a Service Server in Python

Here is an example of a service that adds two integers.

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AddTwoIntsServer(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        self.get_logger().info(f'Sending back response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    server = AddTwoIntsServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Client in Python

And here is the client that calls the `add_two_ints` service.

```python
import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: ros2 run <package_name> <client_node_name> <a> <b>")
        return

    client = AddTwoIntsClient()
    response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    client.get_logger().info(
        f'Result of add_two_ints: for {sys.argv[1]} + {sys.argv[2]} = {response.sum}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

By mastering nodes, topics, and services, you have the power to build complex and robust robotics applications with ROS2.

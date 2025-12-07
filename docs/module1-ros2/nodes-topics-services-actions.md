# Nodes, Topics, Services, Actions in ROS 2

## Deep Dive into ROS 2 Communication Mechanisms

In the previous chapter, we introduced the fundamental concepts of ROS 2 communication. Now, let's explore **Nodes**, **Topics**, **Services**, and **Actions** in greater detail, providing practical Python examples to illustrate their usage. Understanding these mechanisms is crucial for building any non-trivial ROS 2 application.

### 1. Nodes: The Building Blocks of Your Robot Application

As discussed, a node is an executable process that performs a specific computation. In ROS 2, applications are typically composed of many small, focused nodes rather than one monolithic program.

**Example: A Minimal ROS 2 Node in Python**

Let's create a simple node that just initializes itself and logs a message.

```python
# minimal_node.py
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node') # Initialize the node with the name 'minimal_node'
        self.get_logger().info('MinimalNode has been started!')

def main(args=None):
    rclpy.init(args=args) # Initialize the ROS 2 client library
    node = MinimalNode()
    rclpy.spin(node) # Keep the node alive until it's explicitly shutdown
    node.destroy_node() # Clean up when the node is destroyed
    rclpy.shutdown() # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()
```

To run this node, you would typically use `ros2 run <package_name> minimal_node`.

### 2. Topics: Asynchronous Data Streaming (Publish/Subscribe)

Topics are the most common way for nodes to exchange data asynchronously. One node publishes messages to a topic, and any number of other nodes can subscribe to that topic to receive those messages.

**Example: Publisher and Subscriber Nodes**

Let's create a talker (publisher) that sends "Hello ROS 2!" messages and a listener (subscriber) that receives them.

**Talker Node (`talker.py`):**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Standard message type for strings

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10) # Topic: 'chatter', QoS: 10
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker_node = Talker()
    rclpy.spin(talker_node)
    talker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Listener Node (`listener.py`):**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10) # Topic: 'chatter', QoS: 10
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    listener_node = Listener()
    rclpy.spin(listener_node)
    listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Services: Synchronous Request/Reply

Services are used for operations where a client needs an immediate response from a server.

**Example: Add Two Integers Service**

Let's define a service that adds two integers. First, you'd define a custom service message (e.g., in `<package_name>/srv/AddTwoInts.srv`):

```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

Then, generate Python interfaces (covered in next chapter). After generation:

**Service Server Node (`add_two_ints_server.py`):**

```python
import rclpy
from rclpy.node import Node
from your_package_name.srv import AddTwoInts # Custom service message

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add two ints service ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Sending back: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client Node (`add_two_ints_client.py`):**

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from your_package_name.srv import AddTwoInts
import sys

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future) # Wait for response
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client_node = AddTwoIntsClient()
    if len(sys.argv) != 3:
        client_node.get_logger().error('Usage: ros2 run your_package_name add_two_ints_client A B')
        rclpy.shutdown()
        sys.exit(1)
    
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    
    response = client_node.send_request(a, b)
    if response:
        client_node.get_logger().info(f'Result of {a} + {b} = {response.sum}')
    else:
        client_node.get_logger().error('Service call failed!')

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Actions: Asynchronous, Long-Running Tasks with Feedback

Actions are ideal for tasks like "move to a goal" where you need to track progress and potentially cancel the operation. Actions define a goal, result, and feedback.

**Example: Simple 'Count Up' Action**

First, you'd define a custom action message (e.g., in `<package_name>/action/CountUp.action`):

```
# CountUp.action
int32 target_number
---
int32 final_count
---
int32 current_count
```

**Action Server Node (`count_up_action_server.py`):**

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from your_package_name.action import CountUp # Custom action message
import time

class CountUpActionServer(Node):
    def __init__(self):
        super().__init__('count_up_action_server')
        self._action_server = ActionServer(
            self,
            CountUp,
            'count_up',
            self.execute_callback)
        self.get_logger().info('Count up action server ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = CountUp.Feedback()
        
        for i in range(goal_handle.request.target_number):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled!')
                return CountUp.Result()
            
            feedback_msg.current_count = i + 1
            self.get_logger().info(f'Feedback: {feedback_msg.current_count}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1) # Simulate work

        goal_handle.succeed()
        result = CountUp.Result()
        result.final_count = goal_handle.request.target_number
        self.get_logger().info('Goal succeeded!')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = CountUpActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Action Client Node (`count_up_action_client.py`):**

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from your_package_name.action import CountUp
import time

class CountUpActionClient(Node):
    def __init__(self):
        super().__init__('count_up_action_client')
        self._action_client = ActionClient(self, CountUp, 'count_up')

    def send_goal(self, target_number):
        goal_msg = CountUp.Goal()
        goal_msg.target_number = target_number

        self._action_client.wait_for_server()
        self.get_logger().info('Action server found. Sending goal...')
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return

        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: Final Count = {result.final_count}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: Current Count = {feedback_msg.current_count}')


def main(args=None):
    rclpy.init(args=args)
    client_node = CountUpActionClient()
    client_node.send_goal(5) # Send a goal to count up to 5
    rclpy.spin(client_node)

if __name__ == '__main__':
    main()
```

### Next Steps

In the next chapter, we will learn how to create your own ROS 2 package, define custom message, service, and action types, and compile your Python nodes to run within the ROS 2 environment.

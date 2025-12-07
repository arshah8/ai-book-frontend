# ROS 2 Nodes, Topics, and Services

## Nodes: The Building Blocks

Nodes are the fundamental building blocks of ROS 2. Each node is responsible for a specific function, such as reading sensor data, processing images, or controlling motors.

### Creating a Node in Python

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node started')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle

1. **Initialization**: Node is created and configured
2. **Active**: Node is running and processing
3. **Shutdown**: Node is stopped and cleaned up

## Topics: Asynchronous Communication

Topics enable nodes to communicate asynchronously through a publish-subscribe pattern.

### Publisher Example

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
```

### Subscriber Example

```python
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
    
    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Services: Synchronous Communication

Services provide synchronous request-response communication for on-demand operations.

### Service Server Example

```python
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)
    
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        return response
```

### Service Client Example

```python
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
    
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.cli.wait_for_service()
        self.future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Actions: Long-Running Tasks

Actions are ideal for long-running tasks that require feedback.

### Action Server Example

```python
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
    
    def execute_callback(self, goal_handle):
        # Execute long-running task
        # Send feedback periodically
        # Return result when complete
        pass
```

## Bridging Python Agents to ROS 2

### Using rclpy for AI Integration

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent')
        self.publisher = self.create_publisher(String, 'robot_commands', 10)
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.process_sensor_data,
            10)
    
    def process_sensor_data(self, msg):
        # AI processing logic here
        ai_decision = self.ai_model.predict(msg.data)
        
        # Publish command to robot
        command = String()
        command.data = ai_decision
        self.publisher.publish(command)
```

## Best Practices

1. **Error Handling**: Always handle exceptions in callbacks
2. **Logging**: Use `self.get_logger()` for debugging
3. **QoS Settings**: Configure Quality of Service appropriately
4. **Resource Management**: Clean up resources in destructors
5. **Thread Safety**: Be aware of threading in callbacks


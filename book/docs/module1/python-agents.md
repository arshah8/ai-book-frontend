# Bridging Python Agents to ROS 2 Controllers

## Integration Architecture

Bridging Python AI agents to ROS 2 controllers enables:
- **AI Decision Making**: Use ML models to make decisions
- **Robot Control**: Execute decisions through ROS 2
- **Sensor Integration**: Process sensor data with AI
- **Real-time Control**: Maintain low-latency communication

## Using rclpy for AI Integration

### Basic AI Agent Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import numpy as np
import torch  # or tensorflow, etc.

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent')
        
        # Load AI model
        self.model = self.load_model()
        
        # Subscribers
        self.sensor_sub = self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self.process_sensor_data,
            10)
        
        # Publishers
        self.command_pub = self.create_publisher(
            Float32MultiArray,
            'robot_commands',
            10)
        
        self.get_logger().info('AI Agent Node started')
    
    def load_model(self):
        # Load your trained model
        # model = torch.load('model.pth')
        return None
    
    def process_sensor_data(self, msg):
        # Convert ROS message to numpy array
        sensor_data = np.array(msg.data)
        
        # Run inference
        with torch.no_grad():
            prediction = self.model(torch.tensor(sensor_data))
        
        # Convert to ROS message
        command = Float32MultiArray()
        command.data = prediction.numpy().tolist()
        
        # Publish command
        self.command_pub.publish(command)
        self.get_logger().info(f'Published command: {command.data}')
```

## Reinforcement Learning Agent

### RL Agent with ROS 2

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import gym
import stable_baselines3

class RLAgentNode(Node):
    def __init__(self):
        super().__init__('rl_agent')
        
        # Load trained RL model
        self.model = stable_baselines3.PPO.load('rl_model')
        
        # State subscriber
        self.state_sub = self.create_subscription(
            Float32MultiArray,
            'robot_state',
            self.get_action,
            10)
        
        # Action publisher
        self.action_pub = self.create_publisher(
            Float32MultiArray,
            'robot_actions',
            10)
    
    def get_action(self, msg):
        # Get current state
        state = np.array(msg.data)
        
        # Get action from RL model
        action, _ = self.model.predict(state, deterministic=True)
        
        # Publish action
        action_msg = Float32MultiArray()
        action_msg.data = action.tolist()
        self.action_pub.publish(action_msg)
```

## Computer Vision Integration

### Vision Processing Node

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VisionAgentNode(Node):
    def __init__(self):
        super().__init__('vision_agent')
        self.bridge = CvBridge()
        
        # Image subscriber
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.process_image,
            10)
        
        # Detection publisher
        self.detection_pub = self.create_publisher(
            String,
            'object_detections',
            10)
    
    def process_image(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Run object detection
        detections = self.detect_objects(cv_image)
        
        # Publish detections
        detection_msg = String()
        detection_msg.data = str(detections)
        self.detection_pub.publish(detection_msg)
    
    def detect_objects(self, image):
        # Your object detection logic
        # Could use YOLO, TensorFlow, etc.
        return []
```

## Natural Language Processing

### NLP Command Processing

```python
from std_msgs.msg import String
import openai  # or other NLP library

class NLPAgentNode(Node):
    def __init__(self):
        super().__init__('nlp_agent')
        
        # Voice command subscriber
        self.voice_sub = self.create_subscription(
            String,
            'voice_commands',
            self.process_command,
            10)
        
        # Action sequence publisher
        self.actions_pub = self.create_publisher(
            String,
            'action_sequence',
            10)
    
    def process_command(self, msg):
        # Get natural language command
        command = msg.data
        
        # Use LLM to convert to action sequence
        action_sequence = self.llm_to_actions(command)
        
        # Publish action sequence
        action_msg = String()
        action_msg.data = action_sequence
        self.actions_pub.publish(action_msg)
    
    def llm_to_actions(self, command):
        # Use GPT or other LLM to convert command to ROS actions
        # Example: "Pick up the cup" -> ["navigate_to_cup", "grasp_cup", "lift_cup"]
        return "action_sequence"
```

## Best Practices

1. **Model Optimization**: Use optimized models for real-time performance
2. **Async Processing**: Use async callbacks for non-blocking operations
3. **Error Handling**: Handle model inference errors gracefully
4. **Resource Management**: Monitor CPU/GPU usage
5. **Latency**: Minimize processing time for real-time control


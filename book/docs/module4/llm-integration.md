# LLM Integration for Conversational Robotics

## Integrating GPT Models

GPT models enable conversational AI in robots, allowing natural language interaction.

### Conversational Interface

```python
class ConversationalRobot(Node):
    def __init__(self):
        super().__init__('conversational_robot')
        
        self.llm_client = openai.OpenAI()
        
        # Conversation history
        self.conversation_history = []
        
        # Subscribers and publishers
        self.user_input_sub = self.create_subscription(
            String, '/user_input', self.process_input, 10)
        self.response_pub = self.create_publisher(
            String, '/robot_response', 10)
        self.action_pub = self.create_publisher(
            String, '/robot_actions', 10)
    
    def process_input(self, msg):
        user_input = msg.data
        
        # Add to conversation
        self.conversation_history.append({
            "role": "user",
            "content": user_input
        })
        
        # Get LLM response
        response = self.llm_client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a helpful robot assistant. You can control a humanoid robot."},
                *self.conversation_history
            ]
        )
        
        llm_response = response.choices[0].message.content
        
        # Check if response contains action commands
        if self.contains_action(llm_response):
            actions = self.extract_actions(llm_response)
            self.execute_actions(actions)
        
        # Publish response
        response_msg = String()
        response_msg.data = llm_response
        self.response_pub.publish(response_msg)
        
        # Update history
        self.conversation_history.append({
            "role": "assistant",
            "content": llm_response
        })
```

## Multimodal Interaction

### Combining Speech, Gesture, and Vision

```python
class MultimodalInterface(Node):
    def __init__(self):
        super().__init__('multimodal_interface')
        
        # Multiple input sources
        self.speech_sub = self.create_subscription(
            String, '/speech', self.process_speech, 10)
        self.gesture_sub = self.create_subscription(
            String, '/gesture', self.process_gesture, 10)
        self.vision_sub = self.create_subscription(
            Image, '/camera/image', self.process_vision, 10)
        
        # Fused output
        self.command_pub = self.create_publisher(
            String, '/fused_command', 10)
    
    def fuse_modalities(self, speech, gesture, vision):
        # Combine inputs using LLM
        prompt = f"""
        Speech: {speech}
        Gesture: {gesture}
        Vision: {vision}
        
        Generate a unified command.
        """
        
        # Use LLM to fuse
        response = self.llm.generate(prompt)
        return response
```

## Best Practices

1. **Context Management**: Maintain conversation context
2. **Error Handling**: Handle LLM failures gracefully
3. **Safety**: Validate LLM outputs before execution
4. **Latency**: Optimize for real-time interaction


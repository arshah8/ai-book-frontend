# Cognitive Planning: LLMs for Robot Control

## Using LLMs to Translate Natural Language to Actions

Large Language Models can translate high-level natural language commands into sequences of robot actions.

### Basic LLM Integration

```python
import openai
from std_msgs.msg import String

class LLMPlanner(Node):
    def __init__(self):
        super().__init__('llm_planner')
        
        # OpenAI client
        self.client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        
        # Command subscriber
        self.command_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.plan_actions,
            10)
        
        # Action sequence publisher
        self.actions_pub = self.create_publisher(
            String,
            '/action_sequence',
            10)
    
    def plan_actions(self, msg):
        command = msg.data
        
        # Use LLM to generate action sequence
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a robot planner. Convert natural language commands to ROS 2 action sequences."},
                {"role": "user", "content": f"Command: {command}\n\nGenerate a sequence of ROS 2 actions to execute this command."}
            ]
        )
        
        action_sequence = response.choices[0].message.content
        
        # Publish action sequence
        action_msg = String()
        action_msg.data = action_sequence
        self.actions_pub.publish(action_msg)
```

## Action Sequence Execution

### Executing LLM-Generated Actions

```python
class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')
        
        self.actions_sub = self.create_subscription(
            String,
            '/action_sequence',
            self.execute_actions,
            10)
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manipulate_client = ActionClient(self, Manipulate, 'manipulate')
    
    def execute_actions(self, msg):
        # Parse action sequence
        actions = json.loads(msg.data)
        
        # Execute each action
        for action in actions:
            if action['type'] == 'navigate':
                self.navigate(action['goal'])
            elif action['type'] == 'grasp':
                self.grasp(action['object'])
            # Add more action types
```

## Using Gemini for Planning

### Gemini Integration

```python
import google.generativeai as genai

class GeminiPlanner(Node):
    def __init__(self):
        super().__init__('gemini_planner')
        
        genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
        self.model = genai.GenerativeModel('gemini-pro')
        
        self.command_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.plan_with_gemini,
            10)
    
    def plan_with_gemini(self, msg):
        command = msg.data
        
        prompt = f"""
        Convert this natural language command to a ROS 2 action sequence:
        Command: {command}
        
        Return a JSON array of actions, each with 'type' and 'parameters'.
        """
        
        response = self.model.generate_content(prompt)
        action_sequence = json.loads(response.text)
        
        # Publish actions
        self.publish_actions(action_sequence)
```

## Best Practices

1. **Prompt Engineering**: Design effective prompts for action generation
2. **Validation**: Validate LLM outputs before execution
3. **Safety**: Implement safety checks for generated actions
4. **Feedback**: Use execution feedback to improve planning


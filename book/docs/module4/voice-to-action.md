# Voice-to-Action: Using OpenAI Whisper

## OpenAI Whisper Integration

OpenAI Whisper converts spoken language into text, enabling voice commands for robot control.

### Setting Up Whisper

```python
import whisper
import speech_recognition as sr

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_processor')
        
        # Load Whisper model
        self.model = whisper.load_model("base")
        
        # Audio subscriber
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio/raw',
            self.process_audio,
            10)
        
        # Command publisher
        self.command_pub = self.create_publisher(
            String,
            '/voice_commands',
            10)
    
    def process_audio(self, msg):
        # Convert audio to text
        audio_data = msg.data
        result = self.model.transcribe(audio_data)
        text = result["text"]
        
        # Publish command
        command_msg = String()
        command_msg.data = text
        self.command_pub.publish(command_msg)
        
        self.get_logger().info(f'Recognized: {text}')
```

## Real-Time Voice Processing

### Streaming Audio

```python
import pyaudio
import numpy as np

class AudioStreamer(Node):
    def __init__(self):
        super().__init__('audio_streamer')
        
        # Audio configuration
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        
        # Initialize audio
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )
        
        # Publisher
        self.audio_pub = self.create_publisher(
            AudioData,
            '/audio/raw',
            10)
        
        # Timer for streaming
        self.timer = self.create_timer(0.1, self.stream_audio)
    
    def stream_audio(self):
        # Read audio chunk
        data = self.stream.read(self.CHUNK)
        
        # Publish
        audio_msg = AudioData()
        audio_msg.data = list(data)
        self.audio_pub.publish(audio_msg)
```

## Command Recognition

### Processing Voice Commands

```python
class CommandRecognizer(Node):
    def __init__(self):
        super().__init__('command_recognizer')
        
        self.command_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.process_command,
            10)
        
        self.action_pub = self.create_publisher(
            String,
            '/robot_actions',
            10)
    
    def process_command(self, msg):
        command = msg.data.lower()
        
        # Simple command mapping
        if "move forward" in command:
            self.send_action("move_forward")
        elif "turn left" in command:
            self.send_action("turn_left")
        elif "pick up" in command:
            self.send_action("pick_up")
        # Add more commands
    
    def send_action(self, action):
        action_msg = String()
        action_msg.data = action
        self.action_pub.publish(action_msg)
```

## Best Practices

1. **Noise Reduction**: Use noise cancellation for better accuracy
2. **Wake Word**: Implement wake word detection
3. **Confidence**: Check transcription confidence scores
4. **Latency**: Minimize processing delay for real-time control


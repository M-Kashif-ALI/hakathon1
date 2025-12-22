---
sidebar_position: 200
title: Speech Recognition with OpenAI Whisper
---

# Speech Recognition with OpenAI Whisper

This section covers implementing speech recognition in Vision-Language-Action (VLA) systems using OpenAI Whisper, a state-of-the-art automatic speech recognition (ASR) system. You'll learn how to integrate Whisper into your robotic systems for voice command processing.

## Overview

OpenAI Whisper is a general-purpose speech recognition model that excels at various speech recognition tasks. In the context of VLA systems, Whisper enables robots to understand human voice commands and convert them into text for further processing.

### Key Features of Whisper

- **Robustness**: Performs well on various accents, background noise, and technical speech
- **Multilingual**: Supports multiple languages for international applications
- **Open-source**: Allows for customization and integration into robotic systems
- **High accuracy**: State-of-the-art performance on speech recognition benchmarks

## Implementation Approach

There are two main approaches to integrating Whisper with your robotic system:

### 1. API-based Integration (Recommended for Development)

Using OpenAI's API provides a simple way to integrate Whisper capabilities:

```python
import openai
import base64
import io
from pydub import AudioSegment

def transcribe_speech_api(audio_file_path):
    """
    Transcribe speech using OpenAI Whisper API
    """
    with open(audio_file_path, "rb") as audio_file:
        transcript = openai.Audio.transcribe(
            model="whisper-1",
            file=audio_file,
            response_format="text"
        )
    return transcript

# Example usage
transcribed_text = transcribe_speech_api("user_command.wav")
print(f"Transcribed: {transcribed_text}")
```

### 2. Local Model Integration (Recommended for Production)

For production systems where privacy or latency is critical, you can run Whisper locally:

```python
import whisper

def transcribe_speech_local(audio_file_path):
    """
    Transcribe speech using local Whisper model
    """
    # Load model (models: tiny, base, small, medium, large)
    model = whisper.load_model("base")

    # Transcribe
    result = model.transcribe(audio_file_path)
    return result["text"]

# Example usage
transcribed_text = transcribe_speech_local("user_command.wav")
print(f"Transcribed: {transcribed_text}")
```

## Integration with ROS 2

To integrate speech recognition with your ROS 2 system, you'll typically create a speech recognition node:

```python
import rclpy
from rclpy.node import Node
import speech_recognition as sr
from std_msgs.msg import String

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        self.publisher_ = self.create_publisher(String, 'voice_command', 10)

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Start listening
        self.get_logger().info('Speech recognition node started')

    def listen_for_commands(self):
        """
        Listen for voice commands and publish them
        """
        try:
            with self.microphone as source:
                self.get_logger().info('Listening for command...')
                audio = self.recognizer.listen(source, timeout=5.0)

            # Recognize speech
            command_text = self.recognizer.recognize_google(audio)
            self.get_logger().info(f'Heard: {command_text}')

            # Publish command
            msg = String()
            msg.data = command_text
            self.publisher_.publish(msg)

        except sr.WaitTimeoutError:
            self.get_logger().info('No speech detected')
        except sr.UnknownValueError:
            self.get_logger().info('Could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    speech_node = SpeechRecognitionNode()

    # Create timer to periodically listen for commands
    timer = speech_node.create_timer(1.0, speech_node.listen_for_commands)

    rclpy.spin(speech_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Considerations

### Latency Optimization

For real-time applications, consider these optimizations:

1. **Preprocessing**: Reduce audio quality requirements for faster processing
2. **Model selection**: Use smaller Whisper models for faster inference
3. **Caching**: Cache common commands for faster recognition
4. **Edge processing**: Run Whisper locally to avoid network latency

### Accuracy Improvements

1. **Audio quality**: Use high-quality microphones in quiet environments
2. **Vocabulary constraints**: Limit recognized vocabulary to expected commands
3. **Language models**: Fine-tune Whisper for specific domains or accents
4. **Post-processing**: Implement custom post-processing for domain-specific terms

## Common Voice Commands

When designing voice interfaces for robots, consider these common command patterns:

- **Navigation**: "Go to the kitchen", "Move forward 2 meters", "Turn left"
- **Manipulation**: "Pick up the red cup", "Place the object on the table"
- **Interaction**: "Wave your hand", "Stand up", "Sit down"
- **Status**: "What can you do?", "Stop moving", "Return to base"

## Error Handling

Implement robust error handling for speech recognition:

```python
def robust_speech_recognition(recognizer, audio):
    """
    Perform speech recognition with multiple fallback strategies
    """
    strategies = [
        lambda: recognizer.recognize_google(audio),
        lambda: recognizer.recognize_google(audio, show_all=True),
        lambda: recognizer.recognize_sphinx(audio)  # Fallback to offline recognition
    ]

    for i, strategy in enumerate(strategies):
        try:
            result = strategy()
            return result
        except:
            if i == len(strategies) - 1:  # Last strategy
                raise Exception("All recognition strategies failed")

    return None
```

## Next Steps

After implementing speech recognition, you'll need to process the recognized text into structured commands. The next section covers converting voice commands into structured inputs that your robot can understand and execute.
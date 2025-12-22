---
sidebar_position: 400
title: Voice Command Processing Examples
---

# Voice Command Processing Examples

This section provides practical examples showing the complete flow from voice command to robot action. These examples demonstrate how to implement the voice command → text → structured input flow in real-world scenarios.

## Complete Voice-to-Action Pipeline

Here's a complete implementation of the voice-to-action pipeline:

```python
#!/usr/bin/env python3
"""
Complete Voice-to-Action Pipeline Example
Combines speech recognition, command processing, and robot execution
"""

import rclpy
from rclpy.node import Node
import speech_recognition as sr
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import re
import json


class VoiceToActionPipeline(Node):
    def __init__(self):
        super().__init__('voice_to_action_pipeline')

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, 'voice_status', 10)

        # Start the main processing loop
        self.get_logger().info('Voice-to-Action pipeline started')
        self.pipeline_timer = self.create_timer(2.0, self.process_voice_command)

    def process_voice_command(self):
        """
        Complete pipeline: Voice → Text → Structured Command → Robot Action
        """
        try:
            # Step 1: Listen for voice command
            self.get_logger().info('Listening for voice command...')
            status_msg = String()
            status_msg.data = "Listening for command..."
            self.status_publisher.publish(status_msg)

            with self.microphone as source:
                audio = self.recognizer.listen(source, timeout=5.0, phrase_time_limit=5.0)

            # Step 2: Convert speech to text
            self.get_logger().info('Processing audio...')
            status_msg.data = "Processing audio..."
            self.status_publisher.publish(status_msg)

            # Use Google Speech Recognition (you can also use Whisper API)
            command_text = self.recognizer.recognize_google(audio)
            self.get_logger().info(f'Heard: {command_text}')

            status_msg.data = f"Heard: {command_text}"
            self.status_publisher.publish(status_msg)

            # Step 3: Parse the command
            parsed_command = self.parse_command(command_text)

            if parsed_command:
                self.get_logger().info(f'Parsed command: {parsed_command}')

                # Step 4: Validate the command
                is_valid, reason = self.validate_command(parsed_command)
                if not is_valid:
                    self.get_logger().warn(f'Command validation failed: {reason}')
                    status_msg.data = f"Command invalid: {reason}"
                    self.status_publisher.publish(status_msg)
                    return

                # Step 5: Execute the command
                success = self.execute_command(parsed_command)
                if success:
                    self.get_logger().info(f'Command executed successfully: {command_text}')
                    status_msg.data = f"Command executed: {command_text}"
                    self.status_publisher.publish(status_msg)
                else:
                    self.get_logger().error(f'Command execution failed: {command_text}')
                    status_msg.data = f"Command failed: {command_text}"
                    self.status_publisher.publish(status_msg)
            else:
                self.get_logger().warn(f'Could not parse command: {command_text}')
                status_msg.data = f"Could not understand: {command_text}"
                self.status_publisher.publish(status_msg)

        except sr.WaitTimeoutError:
            # No speech detected, this is normal
            pass
        except sr.UnknownValueError:
            self.get_logger().info('Could not understand audio')
            status_msg.data = "Could not understand audio"
            self.status_publisher.publish(status_msg)
        except sr.RequestError as e:
            self.get_logger().error(f'Speech recognition error: {e}')
            status_msg.data = f"Recognition error: {e}"
            self.status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Pipeline error: {e}')
            status_msg.data = f"Pipeline error: {e}"
            self.status_publisher.publish(status_msg)

    def parse_command(self, text):
        """
        Parse natural language command into structured format
        """
        text_lower = text.lower().strip()

        # Navigation commands
        if any(word in text_lower for word in ["move forward", "go forward", "forward"]):
            distance = self.extract_number(text_lower) or 1.0
            return {
                "action": "navigate",
                "command": "move_forward",
                "parameters": {
                    "distance": distance,
                    "unit": "meters"
                }
            }
        elif any(word in text_lower for word in ["move backward", "go backward", "backward", "back"]):
            distance = self.extract_number(text_lower) or 1.0
            return {
                "action": "navigate",
                "command": "move_backward",
                "parameters": {
                    "distance": distance,
                    "unit": "meters"
                }
            }
        elif "turn left" in text_lower or "rotate left" in text_lower:
            angle = self.extract_number(text_lower) or 90
            return {
                "action": "rotate",
                "command": "turn_left",
                "parameters": {
                    "angle": angle,
                    "unit": "degrees"
                }
            }
        elif "turn right" in text_lower or "rotate right" in text_lower:
            angle = self.extract_number(text_lower) or 90
            return {
                "action": "rotate",
                "command": "turn_right",
                "parameters": {
                    "angle": angle,
                    "unit": "degrees"
                }
            }

        # Simple movement commands
        elif "stop" in text_lower:
            return {
                "action": "stop",
                "command": "emergency_stop",
                "parameters": {}
            }
        elif "hello" in text_lower or "wave" in text_lower:
            return {
                "action": "greet",
                "command": "wave_hand",
                "parameters": {}
            }

        # Default: unrecognized command
        return None

    def extract_number(self, text):
        """
        Extract numeric values from text
        """
        numbers = re.findall(r'\d+\.?\d*', text)
        return float(numbers[0]) if numbers else None

    def validate_command(self, command):
        """
        Validate that a command is safe and appropriate
        """
        action = command.get("action")

        if action == "navigate":
            distance = command.get("parameters", {}).get("distance", 0)
            if distance > 10.0:  # Max 10 meters
                return False, "Navigation distance too large"
            if distance <= 0:
                return False, "Navigation distance must be positive"

        elif action == "rotate":
            angle = command.get("parameters", {}).get("angle", 0)
            if abs(angle) > 180:  # Max 180 degree turns
                return False, "Rotation angle too large"

        return True, "Command is valid"

    def execute_command(self, command):
        """
        Execute the parsed command on the robot
        """
        action = command.get("action")

        if action == "navigate":
            return self.execute_navigation_command(command)
        elif action == "rotate":
            return self.execute_rotation_command(command)
        elif action == "stop":
            return self.execute_stop_command()
        elif action == "greet":
            return self.execute_greeting_command()
        else:
            self.get_logger().warn(f'Unknown action type: {action}')
            return False

    def execute_navigation_command(self, command):
        """
        Execute navigation commands
        """
        try:
            distance = command.get("parameters", {}).get("distance", 1.0)

            # Calculate time needed (assuming 0.5 m/s speed)
            duration = distance / 0.5

            # Create velocity command
            twist = Twist()
            twist.linear.x = 0.5  # m/s forward

            # Execute for calculated duration
            self.execute_timed_command(twist, duration)

            return True
        except Exception as e:
            self.get_logger().error(f'Navigation command error: {e}')
            return False

    def execute_rotation_command(self, command):
        """
        Execute rotation commands
        """
        try:
            angle = command.get("parameters", {}).get("angle", 90)

            # Convert degrees to radians and calculate duration
            # Assuming 30 deg/s rotation speed (0.52 rad/s)
            angular_speed = 0.52
            if angle < 0:
                angular_speed = -angular_speed

            duration = abs(angle * 3.14159 / 180) / abs(angular_speed)

            # Create rotation command
            twist = Twist()
            twist.angular.z = angular_speed

            # Execute for calculated duration
            self.execute_timed_command(twist, duration)

            return True
        except Exception as e:
            self.get_logger().error(f'Rotation command error: {e}')
            return False

    def execute_stop_command(self):
        """
        Execute stop command
        """
        try:
            stop_twist = Twist()
            self.cmd_vel_publisher.publish(stop_twist)
            return True
        except Exception as e:
            self.get_logger().error(f'Stop command error: {e}')
            return False

    def execute_greeting_command(self):
        """
        Execute greeting command (placeholder - would control arm in real robot)
        """
        try:
            self.get_logger().info("Executing greeting command - wave gesture")
            # In a real robot, this would control an arm to wave
            time.sleep(2)  # Simulate wave action
            return True
        except Exception as e:
            self.get_logger().error(f'Greeting command error: {e}')
            return False

    def execute_timed_command(self, twist_cmd, duration):
        """
        Execute a command for a specific duration
        """
        start_time = self.get_clock().now().nanoseconds / 1e9

        while (self.get_clock().now().nanoseconds / 1e9) - start_time < duration:
            self.cmd_vel_publisher.publish(twist_cmd)
            time.sleep(0.1)  # 10 Hz update rate

        # Stop the robot after command completion
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)


def main(args=None):
    rclpy.init(args=args)
    pipeline = VoiceToActionPipeline()

    try:
        rclpy.spin(pipeline)
    except KeyboardInterrupt:
        pipeline.get_logger().info('Pipeline interrupted by user')
    finally:
        pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Example Usage Scenarios

### Scenario 1: Basic Navigation
**User says:** "Move forward 2 meters"

**Pipeline execution:**
1. Speech recognition: "Move forward 2 meters"
2. Command parsing: `{action: "navigate", command: "move_forward", parameters: {distance: 2.0, unit: "meters"}}`
3. Validation: Distance is valid (2.0 ≤ 10.0 meters)
4. Execution: Robot moves forward at 0.5 m/s for 4 seconds

### Scenario 2: Rotation Command
**User says:** "Turn left 45 degrees"

**Pipeline execution:**
1. Speech recognition: "Turn left 45 degrees"
2. Command parsing: `{action: "rotate", command: "turn_left", parameters: {angle: 45, unit: "degrees"}}`
3. Validation: Angle is valid (45 ≤ 180 degrees)
4. Execution: Robot rotates left at 0.52 rad/s for ~1.5 seconds

### Scenario 3: Combined Commands
**User says:** "Move forward 1 meter, then turn right"

**Pipeline execution:**
1. First command: "Move forward 1 meter"
   - Parsed and executed as navigation command
2. After completion, system listens for next command
3. Second command: "turn right" (assuming 90 degrees)
   - Parsed and executed as rotation command

## Error Handling Examples

### Example 1: Unrecognized Command
```python
# User says: "Bippity boppity boo"
# System response:
# - Speech recognition: "Bippity boppity boo"
# - Command parsing: None (returns None)
# - System feedback: "Could not understand: Bippity boppity boo"
```

### Example 2: Invalid Command
```python
# User says: "Move forward 50 meters"
# System response:
# - Speech recognition: "Move forward 50 meters"
# - Command parsing: {action: "navigate", ...}
# - Validation: "Navigation distance too large" (50 > 10 meters)
# - System feedback: "Command invalid: Navigation distance too large"
```

### Example 3: Safe Command Rejection
```python
# System detects obstacle but user says: "Move forward 3 meters"
# System response:
# - Command validation checks for obstacles
# - Validation: "Obstacles detected in navigation path"
# - System feedback: "Command invalid: Obstacles detected in navigation path"
```

## Performance Optimization Examples

### Optimized Speech Recognition
```python
class OptimizedVoiceRecognition:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.command_keywords = [
            "move", "forward", "backward", "turn", "left", "right",
            "stop", "go", "navigate", "hello", "wave", "pick up", "place"
        ]

    def optimized_listen(self):
        """
        Optimized listening with keyword spotting
        """
        with self.microphone as source:
            # Use phrase time limit to reduce processing time
            audio = self.recognizer.listen(source, timeout=5.0, phrase_time_limit=3.0)

        try:
            # Use Google's specialized models for short phrases
            text = self.recognizer.recognize_google(
                audio,
                show_all=False
            )
            return text
        except sr.UnknownValueError:
            return None
```

### Caching Common Commands
```python
class CachedCommandProcessor:
    def __init__(self):
        # Cache for common commands to improve response time
        self.command_cache = {
            "move forward": {
                "action": "navigate",
                "command": "move_forward",
                "parameters": {"distance": 1.0, "unit": "meters"}
            },
            "stop": {
                "action": "stop",
                "command": "emergency_stop",
                "parameters": {}
            }
        }

    def process_with_cache(self, text):
        """
        Process command with caching for common commands
        """
        # Check cache first
        if text.lower() in self.command_cache:
            return self.command_cache[text.lower()]

        # Fall back to full parsing
        return self.parse_command(text)
```

## Integration Testing Example

Here's how to test the complete pipeline:

```python
import unittest
from unittest.mock import Mock, patch

class TestVoiceToActionPipeline(unittest.TestCase):
    def setUp(self):
        # Mock ROS 2 node
        self.pipeline = VoiceToActionPipeline()

    def test_basic_navigation_command(self):
        """Test 'move forward 2 meters' command"""
        command_text = "Move forward 2 meters"
        result = self.pipeline.parse_command(command_text)

        expected = {
            "action": "navigate",
            "command": "move_forward",
            "parameters": {
                "distance": 2.0,
                "unit": "meters"
            }
        }

        self.assertEqual(result, expected)

    def test_rotation_command(self):
        """Test 'turn left 90 degrees' command"""
        command_text = "Turn left 90 degrees"
        result = self.pipeline.parse_command(command_text)

        expected = {
            "action": "rotate",
            "command": "turn_left",
            "parameters": {
                "angle": 90,
                "unit": "degrees"
            }
        }

        self.assertEqual(result, expected)

    def test_validation_distance_limit(self):
        """Test that large distances are rejected"""
        command = {
            "action": "navigate",
            "command": "move_forward",
            "parameters": {"distance": 15.0}  # Too large
        }

        is_valid, reason = self.pipeline.validate_command(command)
        self.assertFalse(is_valid)
        self.assertIn("distance too large", reason.lower())

if __name__ == '__main__':
    unittest.main()
```

## Real-World Deployment Considerations

### 1. Audio Quality Management
```python
class AudioQualityManager:
    def __init__(self):
        self.noise_threshold = 0.1  # Adjust based on environment

    def check_audio_quality(self, audio_data):
        """
        Check if audio quality is sufficient for recognition
        """
        # Calculate audio level (simplified)
        audio_level = self.calculate_audio_level(audio_data)

        if audio_level < self.noise_threshold:
            return False, "Audio too quiet, please speak louder"

        if audio_level > 0.9:  # Too loud, possible distortion
            return False, "Audio too loud, please speak softer"

        return True, "Audio quality is good"
```

### 2. Context-Aware Command Processing
```python
class ContextAwareProcessor:
    def __init__(self):
        self.robot_location = None
        self.robot_state = "idle"
        self.available_objects = []

    def parse_with_context(self, text, context):
        """
        Parse command considering current robot context
        """
        # Example: "Go there" needs context to know where "there" is
        if "there" in text.lower() and context.get("pointed_location"):
            text = text.lower().replace("there", context["pointed_location"])

        # Parse the updated command
        return self.parse_command(text)
```

This completes the practical examples for voice command processing. The examples demonstrate the complete flow from voice input to robot action, including error handling, validation, and real-world considerations.
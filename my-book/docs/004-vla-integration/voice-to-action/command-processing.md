---
sidebar_position: 300
title: Command Processing
---

# Command Processing

This section covers converting recognized speech into structured robot commands that can be executed by your robotic system. You'll learn how to parse natural language voice commands and convert them into structured inputs that your robot can understand and execute.

## Overview

Command processing bridges the gap between natural language understanding and robotic action. It involves:

1. **Parsing**: Extracting meaningful information from recognized speech
2. **Structuring**: Converting natural language into structured command formats
3. **Validation**: Ensuring commands are valid and safe to execute
4. **Execution**: Sending commands to appropriate robot systems

## Command Parsing Approaches

### 1. Rule-Based Parsing

Simple rule-based parsing works well for constrained command sets:

```python
def parse_command_simple(text):
    """
    Simple rule-based command parser for basic commands
    """
    text_lower = text.lower().strip()

    # Navigation commands
    if "move forward" in text_lower:
        distance = extract_number(text_lower) or 1.0
        return {
            "action": "navigate",
            "command": "forward",
            "distance": distance,
            "unit": "meters"
        }
    elif "move backward" in text_lower:
        distance = extract_number(text_lower) or 1.0
        return {
            "action": "navigate",
            "command": "backward",
            "distance": distance,
            "unit": "meters"
        }
    elif "turn left" in text_lower or "rotate left" in text_lower:
        angle = extract_number(text_lower) or 90
        return {
            "action": "rotate",
            "direction": "left",
            "angle": angle,
            "unit": "degrees"
        }
    elif "turn right" in text_lower or "rotate right" in text_lower:
        angle = extract_number(text_lower) or 90
        return {
            "action": "rotate",
            "direction": "right",
            "angle": angle,
            "unit": "degrees"
        }

    # Manipulation commands
    elif "pick up" in text_lower or "grasp" in text_lower:
        object_name = extract_object_name(text_lower)
        return {
            "action": "manipulate",
            "command": "pick_up",
            "object": object_name
        }
    elif "place" in text_lower or "put" in text_lower:
        object_name = extract_object_name(text_lower)
        location = extract_location(text_lower)
        return {
            "action": "manipulate",
            "command": "place",
            "object": object_name,
            "location": location
        }

    # Default: unrecognized command
    return {
        "action": "unknown",
        "raw_text": text,
        "confidence": 0.0
    }

def extract_number(text):
    """
    Extract numeric values from text
    """
    import re
    numbers = re.findall(r'\d+\.?\d*', text)
    return float(numbers[0]) if numbers else None

def extract_object_name(text):
    """
    Extract object names from text (simplified approach)
    """
    # Common object keywords
    objects = ["cup", "bottle", "book", "ball", "box", "red cup", "blue bottle"]
    for obj in objects:
        if obj in text:
            return obj
    return "unknown_object"

def extract_location(text):
    """
    Extract location from text (simplified approach)
    """
    locations = ["table", "kitchen", "living room", "bedroom", "shelf"]
    for loc in locations:
        if loc in text:
            return loc
    return "unknown_location"
```

### 2. Template-Based Parsing

More sophisticated parsing using predefined templates:

```python
import re
from typing import Dict, List, Optional

class CommandTemplate:
    def __init__(self, pattern: str, action: str, params: List[str]):
        self.pattern = pattern
        self.action = action
        self.params = params

class TemplateParser:
    def __init__(self):
        self.templates = [
            # Navigation templates
            CommandTemplate(
                r"go to the (\w+)",
                "navigate",
                ["location"]
            ),
            CommandTemplate(
                r"move (\w+) (\d+\.?\d*) meters?",
                "navigate",
                ["direction", "distance"]
            ),
            CommandTemplate(
                r"turn (\w+)",
                "rotate",
                ["direction"]
            ),

            # Manipulation templates
            CommandTemplate(
                r"pick up the (\w+)",
                "manipulate",
                ["object"]
            ),
            CommandTemplate(
                r"grasp the (\w+)",
                "manipulate",
                ["object"]
            ),
            CommandTemplate(
                r"place the (\w+) on the (\w+)",
                "manipulate",
                ["object", "location"]
            ),
        ]

    def parse(self, text: str) -> Optional[Dict]:
        """
        Parse text using predefined templates
        """
        text_lower = text.lower().strip()

        for template in self.templates:
            match = re.search(template.pattern, text_lower)
            if match:
                # Extract parameters based on template
                params = match.groups()
                command = {
                    "action": template.action,
                    "raw_text": text,
                    "confidence": 0.9  # High confidence for template match
                }

                # Add extracted parameters
                for i, param_name in enumerate(template.params):
                    if i < len(params):
                        command[param_name] = params[i]

                return command

        return None  # No template matched
```

### 3. LLM-Based Parsing (Advanced)

For complex natural language understanding:

```python
import json
import openai

def parse_command_llm(text: str, robot_capabilities: List[str] = None):
    """
    Use LLM to parse complex natural language commands
    """
    capabilities = robot_capabilities or [
        "navigation", "manipulation", "perception", "communication"
    ]

    prompt = f"""
    Parse this natural language command into a structured robot command.

    Command: "{text}"

    Available robot capabilities: {capabilities}

    Return a JSON object with the following structure:
    {{
        "action": "navigate|manipulate|perceive|communicate|unknown",
        "command": "specific_command_name",
        "parameters": {{"param_name": "param_value", ...}},
        "confidence": float (0.0-1.0)
    }}

    Example responses:
    For "Move forward 2 meters":
    {{"action": "navigate", "command": "move_forward", "parameters": {{"distance": 2.0, "unit": "meters"}}, "confidence": 0.95}}

    For "Pick up the red cup":
    {{"action": "manipulate", "command": "grasp", "parameters": {{"object": "red cup", "object_color": "red", "object_type": "cup"}}, "confidence": 0.92}}

    Response:
    """

    try:
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1
        )

        # Extract and parse JSON response
        response_text = response.choices[0].message.content.strip()

        # Handle potential response formatting
        if response_text.startswith("```json"):
            response_text = response_text[7:]  # Remove ```json
        if response_text.endswith("```"):
            response_text = response_text[:-3]  # Remove ```

        return json.loads(response_text)

    except Exception as e:
        print(f"LLM parsing failed: {e}")
        return {
            "action": "unknown",
            "raw_text": text,
            "confidence": 0.0,
            "error": str(e)
        }
```

## Integration with ROS 2

Here's how to integrate command processing with your ROS 2 system:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration

class CommandProcessorNode(Node):
    def __init__(self):
        super().__init__('command_processor_node')

        # Subscribers
        self.voice_subscriber = self.create_subscription(
            String,
            'voice_command',
            self.voice_command_callback,
            10
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Command parser
        self.parser = TemplateParser()

        self.get_logger().info('Command processor node started')

    def voice_command_callback(self, msg):
        """
        Process incoming voice commands
        """
        command_text = msg.data
        self.get_logger().info(f'Processing command: {command_text}')

        # Parse the command
        parsed_command = self.parser.parse(command_text)

        if parsed_command and parsed_command.get("confidence", 0) > 0.5:
            # Execute the command
            self.execute_command(parsed_command)
        else:
            self.get_logger().warn(f'Could not parse command: {command_text}')
            # Optionally send to LLM-based parser for complex commands
            llm_result = parse_command_llm(command_text)
            if llm_result and llm_result.get("confidence", 0) > 0.7:
                self.execute_command(llm_result)

    def execute_command(self, command):
        """
        Execute parsed command on the robot
        """
        action = command.get("action")

        if action == "navigate":
            self.execute_navigation_command(command)
        elif action == "manipulate":
            self.execute_manipulation_command(command)
        elif action == "rotate":
            self.execute_rotation_command(command)
        else:
            self.get_logger().warn(f'Unknown action type: {action}')

    def execute_navigation_command(self, command):
        """
        Execute navigation commands
        """
        direction = command.get("direction", "forward")
        distance = float(command.get("distance", 1.0))

        # Create velocity command
        twist = Twist()

        if direction in ["forward", "front"]:
            twist.linear.x = 0.5  # m/s
        elif direction in ["backward", "back"]:
            twist.linear.x = -0.5
        elif direction == "left":
            twist.linear.y = 0.5
        elif direction == "right":
            twist.linear.y = -0.5

        # Publish command for a duration based on distance
        duration = distance / 0.5  # Assuming 0.5 m/s speed
        self.execute_timed_command(twist, duration)

    def execute_rotation_command(self, command):
        """
        Execute rotation commands
        """
        direction = command.get("direction", "left")
        angle = float(command.get("angle", 90))  # degrees

        # Convert to angular velocity (assuming 30 deg/s rotation speed)
        twist = Twist()
        angular_speed = 0.52  # rad/s (30 deg/s)

        if direction == "right":
            angular_speed = -angular_speed

        twist.angular.z = angular_speed

        # Calculate duration: angle (rad) / angular speed (rad/s)
        duration = (angle * 3.14159 / 180) / abs(angular_speed)
        self.execute_timed_command(twist, duration)

    def execute_timed_command(self, twist_cmd, duration):
        """
        Execute a command for a specific duration
        """
        import time

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(twist_cmd)
            time.sleep(0.1)  # 10 Hz update rate

        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    command_processor = CommandProcessorNode()

    rclpy.spin(command_processor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Validation and Safety

Implement validation to ensure commands are safe and appropriate:

```python
def validate_command(parsed_command, robot_state):
    """
    Validate that a command is safe and appropriate
    """
    # Check if robot is in safe state
    if robot_state.get("emergency_stop", False):
        return False, "Robot is in emergency stop state"

    # Validate navigation commands
    if parsed_command.get("action") == "navigate":
        distance = parsed_command.get("distance", 0)
        if distance > 10.0:  # Max 10 meters
            return False, "Navigation distance too large"

        # Check for obstacles in planned path
        if has_obstacles_ahead(robot_state, distance):
            return False, "Obstacles detected in navigation path"

    # Validate manipulation commands
    elif parsed_command.get("action") == "manipulate":
        object_name = parsed_command.get("object", "")
        if not object_name:
            return False, "No object specified for manipulation"

        # Check if object is reachable
        if not is_object_reachable(robot_state, object_name):
            return False, f"Object '{object_name}' is not reachable"

    return True, "Command is valid"

def has_obstacles_ahead(robot_state, distance):
    """
    Check if there are obstacles in the robot's path
    """
    # Implementation depends on your robot's sensor system
    # This is a placeholder
    return False

def is_object_reachable(robot_state, object_name):
    """
    Check if an object is reachable by the robot
    """
    # Implementation depends on your robot's manipulation system
    # This is a placeholder
    return True
```

## Error Handling and Feedback

Provide clear feedback to users about command processing:

```python
class CommandProcessorWithFeedback:
    def __init__(self):
        self.feedback_publisher = None  # Initialize with your feedback publisher

    def process_with_feedback(self, command_text):
        """
        Process command with user feedback
        """
        try:
            # Parse command
            parsed_command = self.parser.parse(command_text)

            if not parsed_command:
                self.send_feedback("I didn't understand that command. Could you repeat it?", "error")
                return False

            # Validate command
            is_valid, reason = validate_command(parsed_command, self.get_robot_state())
            if not is_valid:
                self.send_feedback(f"Cannot execute: {reason}", "error")
                return False

            # Execute command
            self.execute_command(parsed_command)
            self.send_feedback(f"Executing: {command_text}", "success")

            return True

        except Exception as e:
            self.send_feedback(f"Error processing command: {str(e)}", "error")
            return False

    def send_feedback(self, message, level):
        """
        Send feedback to user (visual, audio, or text)
        """
        # Implementation depends on your feedback system
        print(f"[{level.upper()}] {message}")
```

## Performance Considerations

- **Response time**: Aim for command processing within 1-2 seconds
- **Accuracy**: Use multiple parsing strategies to improve recognition accuracy
- **Robustness**: Handle ambiguous or incomplete commands gracefully
- **Safety**: Always validate commands before execution

## Next Steps

After implementing command processing, you'll want to add practical examples showing the complete voice command → text → structured input flow. The next section will cover common voice command processing patterns and best practices.
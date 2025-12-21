---
sidebar_position: 400
title: LLM-Based Planning Practical Examples
---

# LLM-Based Planning Practical Examples

This section provides practical examples showing the complete flow from natural language tasks to LLM-generated action sequences and their execution on robotic systems.

## Complete LLM Planning Pipeline

Here's a complete implementation of the LLM-based planning pipeline:

```python
#!/usr/bin/env python3
"""
Complete LLM-Based Planning Pipeline Example
Combines LLM task planning, ROS 2 mapping, and action execution
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json
import openai
import time


class LLMPlanningPipeline(Node):
    def __init__(self):
        super().__init__('llm_planning_pipeline')

        # Initialize LLM
        self.llm_api_key = self.declare_parameter('llm_api_key', '').value or "your-api-key"
        openai.api_key = self.llm_api_key

        # Subscribers
        self.task_subscriber = self.create_subscription(
            String,
            'natural_language_task',
            self.task_callback,
            10
        )

        # Publishers
        self.plan_publisher = self.create_publisher(String, 'action_plan', 10)
        self.status_publisher = self.create_publisher(String, 'planner_status', 10)
        self.feedback_publisher = self.create_publisher(String, 'execution_feedback', 10)

        # Initialize components
        self.plan_executor = PlanExecutionPipeline(self)

        self.get_logger().info('LLM Planning Pipeline started')

    def task_callback(self, msg):
        """
        Complete pipeline: Natural Language → LLM Plan → ROS 2 Execution
        """
        task_text = msg.data
        self.get_logger().info(f'Received task: {task_text}')

        # Update status
        self.publish_status(f"Processing task: {task_text}")

        try:
            # Step 1: Use LLM to generate action plan
            self.publish_status("Generating plan with LLM...")
            plan_result = self.generate_llm_plan(task_text)

            if not plan_result["success"]:
                self.get_logger().error(f'LLM planning failed: {plan_result["error"]}')
                self.publish_status(f"Planning failed: {plan_result['error']}")
                return

            # Step 2: Validate the generated plan
            is_valid, validation_msg = self.validate_plan(plan_result["plan"])
            if not is_valid:
                self.get_logger().error(f'Plan validation failed: {validation_msg}')
                self.publish_status(f"Plan validation failed: {validation_msg}")
                return

            # Step 3: Publish the plan for execution
            plan_msg = String()
            plan_msg.data = json.dumps({
                "task": task_text,
                "plan": plan_result["plan"],
                "timestamp": self.get_clock().now().nanoseconds
            })
            self.plan_publisher.publish(plan_msg)

            self.get_logger().info(f'Generated plan with {len(plan_result["plan"])} steps')

            # Step 4: Execute the plan
            self.publish_status("Executing plan...")
            execution_success = self.plan_executor.execute_plan(plan_result["plan"])

            if execution_success:
                self.get_logger().info('Plan executed successfully')
                self.publish_status("Plan executed successfully")
                self.publish_feedback({
                    "task": task_text,
                    "success": True,
                    "steps_completed": len(plan_result["plan"])
                })
            else:
                self.get_logger().error('Plan execution failed')
                self.publish_status("Plan execution failed")
                self.publish_feedback({
                    "task": task_text,
                    "success": False,
                    "error": "Execution failed"
                })

        except Exception as e:
            self.get_logger().error(f'Pipeline error: {e}')
            self.publish_status(f"Pipeline error: {e}")

    def generate_llm_plan(self, task_text):
        """
        Generate action plan using LLM
        """
        prompt = f"""
        Convert this natural language task to a structured action plan for a robot.

        Task: "{task_text}"

        Robot capabilities: ["navigation", "manipulation", "perception", "communication"]

        Return a JSON list of actions with these types:
        - navigation: {{"type": "navigate", "x": float, "y": float, "description": "Human-readable description"}}
        - manipulation: {{"type": "manipulate", "action": "grasp|release|move", "object": "object_name", "description": "Human-readable description"}}
        - perception: {{"type": "perceive", "target": "object_or_location", "description": "Human-readable description"}}
        - communication: {{"type": "communicate", "message": "message_text", "description": "Human-readable description"}}

        Each action should have:
        - type: The action category
        - description: Human-readable explanation
        - parameters: Action-specific parameters

        Response (JSON only, no additional text):
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1,
                max_tokens=1000
            )

            response_text = response.choices[0].message.content.strip()

            # Clean up response if it contains markdown
            if response_text.startswith("```"):
                start = response_text.find('{')
                end = response_text.rfind('}') + 1
                response_text = response_text[start:end]

            action_plan = json.loads(response_text)

            return {
                "success": True,
                "plan": action_plan,
                "raw_response": response.choices[0].message.content
            }

        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "raw_response": None
            }

    def validate_plan(self, plan):
        """
        Validate that the plan is executable
        """
        if not plan:
            return False, "Plan is empty"

        if not isinstance(plan, list):
            return False, "Plan must be a list of actions"

        supported_actions = {"navigate", "manipulate", "perceive", "communicate"}

        for i, action in enumerate(plan):
            if not isinstance(action, dict):
                return False, f"Action {i} is not a dictionary"

            action_type = action.get("type")
            if action_type not in supported_actions:
                return False, f"Action {i} has unsupported type: {action_type}"

            if "description" not in action:
                return False, f"Action {i} missing description"

        return True, "Plan is valid"

    def publish_status(self, status_message):
        """
        Publish execution status
        """
        msg = String()
        msg.data = status_message
        self.status_publisher.publish(msg)

    def publish_feedback(self, feedback_data):
        """
        Publish execution feedback
        """
        msg = String()
        msg.data = json.dumps(feedback_data)
        self.feedback_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pipeline = LLMPlanningPipeline()

    try:
        rclpy.spin(pipeline)
    except KeyboardInterrupt:
        pipeline.get_logger().info('Pipeline interrupted by user')
    finally:
        pipeline.destroy_node()
        rclpy.shutdown()
```

## Example Usage Scenarios

### Scenario 1: Simple Navigation Task
**User says:** "Go to the kitchen and tell me what you see there"

**Pipeline execution:**
1. Natural language input: "Go to the kitchen and tell me what you see there"
2. LLM processing generates plan:
   ```json
   [
     {
       "type": "navigate",
       "parameters": {"x": 5.0, "y": 3.0},
       "description": "Navigate to kitchen at coordinates (5.0, 3.0)"
     },
     {
       "type": "perceive",
       "parameters": {"target": "kitchen"},
       "description": "Perceive objects in the kitchen"
     },
     {
       "type": "communicate",
       "parameters": {"message": "I see a table, chair, and refrigerator in the kitchen"},
       "description": "Communicate observations to user"
     }
   ]
   ```
3. ROS 2 mapping converts actions to appropriate services/actions
4. Plan execution with feedback

### Scenario 2: Object Manipulation Task
**User says:** "Find the red cup in the living room and bring it to the table"

**Pipeline execution:**
1. Natural language input: "Find the red cup in the living room and bring it to the table"
2. LLM processing generates plan:
   ```json
   [
     {
       "type": "navigate",
       "parameters": {"x": 2.0, "y": 1.0},
       "description": "Navigate to living room"
     },
     {
       "type": "perceive",
       "parameters": {"target": "red cup"},
       "description": "Look for red cup"
     },
     {
       "type": "manipulate",
       "parameters": {"action": "grasp", "object": "red cup"},
       "description": "Grasp the red cup"
     },
     {
       "type": "navigate",
       "parameters": {"x": 4.0, "y": 2.0},
       "description": "Navigate to table"
     },
     {
       "type": "manipulate",
       "parameters": {"action": "release", "object": "red cup"},
       "description": "Release the red cup on the table"
     }
   ]
   ```
3. Each action is mapped to ROS 2 equivalents and executed

### Scenario 3: Complex Multi-Step Task
**User says:** "Go to John's office, check if he's there, and if he is, tell him I'll be late for the meeting"

**Pipeline execution:**
1. Natural language input: "Go to John's office, check if he's there, and if he is, tell him I'll be late for the meeting"
2. LLM processing with conditional logic generates plan:
   ```json
   [
     {
       "type": "navigate",
       "parameters": {"x": 8.0, "y": 5.0},
       "description": "Navigate to John's office"
     },
     {
       "type": "perceive",
       "parameters": {"target": "person"},
       "description": "Check if John is present"
     },
     {
       "type": "conditional",
       "parameters": {
         "condition": "person_detected",
         "true_actions": [
           {
             "type": "communicate",
             "parameters": {"message": "Hi John, I'll be late for the meeting"},
             "description": "Tell John about being late"
           }
         ],
         "false_actions": [
           {
             "type": "communicate",
             "parameters": {"message": "John is not in his office"},
             "description": "Report John is not there"
           }
         ]
       },
       "description": "Conditional action based on John's presence"
     }
   ]
   ```

## Practical Implementation Examples

### 1. Context-Aware Planning
```python
class ContextAwarePlanner:
    def __init__(self, node):
        self.node = node
        self.environment_context = {
            "locations": {
                "kitchen": {"x": 5.0, "y": 3.0},
                "living_room": {"x": 2.0, "y": 1.0},
                "office": {"x": 8.0, "y": 5.0},
                "bedroom": {"x": 1.0, "y": 4.0}
            },
            "objects": {
                "red_cup": {"location": "kitchen", "color": "red", "type": "cup"},
                "blue_bottle": {"location": "living_room", "color": "blue", "type": "bottle"},
                "book": {"location": "office", "color": "brown", "type": "book"}
            },
            "robot_position": {"x": 0.0, "y": 0.0}
        }

    def generate_contextual_plan(self, task_text):
        """
        Generate plan using environmental context
        """
        context_prompt = f"""
        Task: "{task_text}"

        Environmental Context:
        - Available locations: {self.environment_context['locations']}
        - Available objects: {self.environment_context['objects']}
        - Robot current position: {self.environment_context['robot_position']}

        Generate a step-by-step action plan considering the environmental context.
        Use the provided location coordinates and object information when relevant.

        Return JSON action plan.
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": context_prompt}],
                temperature=0.1
            )

            response_text = response.choices[0].message.content.strip()
            if response_text.startswith("```"):
                start = response_text.find('{')
                end = response_text.rfind('}') + 1
                response_text = response_text[start:end]

            return json.loads(response_text)

        except Exception as e:
            self.node.get_logger().error(f'Contextual planning error: {e}')
            return []
```

### 2. Multi-Modal Planning
```python
class MultiModalPlanner:
    def __init__(self, node):
        self.node = node

    def plan_with_multiple_inputs(self, text_input=None, image_input=None, sensor_input=None):
        """
        Plan using multiple input modalities
        """
        modalities = []
        if text_input:
            modalities.append(f"Text: {text_input}")
        if image_input:
            modalities.append("Image: [visual scene understanding]")
        if sensor_input:
            modalities.append(f"Sensor: {sensor_input}")

        multi_modal_prompt = f"""
        Plan based on multiple input modalities:
        {modalities}

        Generate appropriate action plan considering all available information.

        Return JSON action plan.
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-4-vision-preview",  # Use multimodal model if image input
                messages=[{"role": "user", "content": multi_modal_prompt}],
                temperature=0.1
            )

            response_text = response.choices[0].message.content.strip()
            if response_text.startswith("```"):
                start = response_text.find('{')
                end = response_text.rfind('}') + 1
                response_text = response_text[start:end]

            return json.loads(response_text)

        except Exception as e:
            self.node.get_logger().error(f'Multi-modal planning error: {e}')
            return []
```

### 3. Collaborative Planning
```python
class CollaborativePlanner:
    def __init__(self, node):
        self.node = node
        self.human_feedback_buffer = []

    def plan_with_human_feedback(self, initial_task, human_feedback=None):
        """
        Plan that incorporates human feedback and corrections
        """
        if human_feedback:
            self.human_feedback_buffer.append(human_feedback)

        feedback_context = "\n".join(self.human_feedback_buffer[-5:])  # Last 5 feedbacks

        collaborative_prompt = f"""
        Initial task: "{initial_task}"

        Previous human feedback: {feedback_context if feedback_context else 'None'}

        Generate an improved action plan considering the human feedback.
        If feedback indicates a problem with a previous plan, adjust accordingly.
        If feedback provides additional information, incorporate it.

        Return JSON action plan.
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": collaborative_prompt}],
                temperature=0.1
            )

            response_text = response.choices[0].message.content.strip()
            if response_text.startswith("```"):
                start = response_text.find('{')
                end = response_text.rfind('}') + 1
                response_text = response_text[start:end]

            return json.loads(response_text)

        except Exception as e:
            self.node.get_logger().error(f'Collaborative planning error: {e}')
            return []
```

## Error Handling Examples

### Example 1: LLM Planning Failure
```python
def robust_llm_planning(self, task_text):
    """
    LLM planning with multiple fallback strategies
    """
    strategies = [
        lambda: self.primary_llm_planning(task_text),
        lambda: self.secondary_llm_planning(task_text),  # Different model
        lambda: self.rule_based_fallback(task_text),    # Rule-based planner
    ]

    for i, strategy in enumerate(strategies):
        try:
            result = strategy()
            if result and self.validate_plan(result):
                return result
        except Exception as e:
            self.node.get_logger().warn(f'Strategy {i+1} failed: {e}')
            continue

    # All strategies failed
    return self.emergency_response_plan(task_text)
```

### Example 2: Plan Execution Failure
```python
def handle_execution_failure(self, failed_step, plan, current_state):
    """
    Handle plan execution failure with recovery
    """
    failed_action = plan[failed_step]
    self.node.get_logger().error(f'Action failed: {failed_action}')

    # Determine failure type and recovery strategy
    if failed_action['type'] == 'navigate':
        return self.handle_navigation_failure(failed_step, plan, current_state)
    elif failed_action['type'] == 'manipulate':
        return self.handle_manipulation_failure(failed_step, plan, current_state)
    elif failed_action['type'] == 'perceive':
        return self.handle_perception_failure(failed_step, plan, current_state)
    else:
        return self.handle_generic_failure(failed_step, plan, current_state)

def handle_navigation_failure(self, failed_step, plan, current_state):
    """
    Handle navigation failure with replanning
    """
    # Try alternative route
    original_goal = plan[failed_step].get('parameters', {})
    alternative_route = self.find_alternative_route(
        current_state['position'],
        original_goal
    )

    if alternative_route:
        # Modify plan to use alternative route
        plan[failed_step]['parameters']['alternative_route'] = alternative_route
        return plan, failed_step  # Retry from failed step

    # If no alternative route, skip this step if possible
    if self.can_skip_step(plan, failed_step):
        plan.pop(failed_step)  # Remove failed step
        return plan, failed_step  # Continue from next step

    return None  # Cannot recover
```

## Performance Optimization Examples

### 1. Plan Caching
```python
class CachedPlanner:
    def __init__(self):
        self.plan_cache = {}
        self.cache_size_limit = 50

    def get_cached_plan(self, task_hash):
        """
        Retrieve plan from cache if available
        """
        return self.plan_cache.get(task_hash)

    def cache_plan(self, task_hash, plan):
        """
        Cache a plan for future use
        """
        if len(self.plan_cache) >= self.cache_size_limit:
            # Remove oldest entry (simple FIFO)
            oldest_key = next(iter(self.plan_cache))
            del self.plan_cache[oldest_key]

        self.plan_cache[task_hash] = plan

    def plan_task_with_cache(self, task_text):
        """
        Plan task using cache when possible
        """
        task_hash = hash(task_text.lower().strip())

        # Check cache first
        cached_plan = self.get_cached_plan(task_hash)
        if cached_plan:
            self.node.get_logger().info('Using cached plan')
            return cached_plan

        # Generate new plan
        plan = self.generate_llm_plan(task_text)

        # Cache the new plan
        if plan and plan.get("success"):
            self.cache_plan(task_hash, plan)

        return plan
```

### 2. Parallel Plan Generation
```python
import asyncio
from concurrent.futures import ThreadPoolExecutor

class ParallelPlanner:
    def __init__(self, node):
        self.node = node
        self.executor = ThreadPoolExecutor(max_workers=3)

    async def generate_multiple_plans(self, task_text):
        """
        Generate multiple plans in parallel for comparison
        """
        # Different approaches to the same task
        approaches = [
            lambda: self.semantic_planning(task_text),
            lambda: self.grammar_based_planning(task_text),
            lambda: self.example_based_planning(task_text)
        ]

        # Execute all approaches in parallel
        tasks = [
            asyncio.get_event_loop().run_in_executor(self.executor, approach)
            for approach in approaches
        ]

        results = await asyncio.gather(*tasks, return_exceptions=True)

        # Select best plan based on criteria
        valid_plans = [r for r in results if not isinstance(r, Exception) and r.get("success")]

        if valid_plans:
            # Choose plan with highest confidence or shortest length
            best_plan = max(valid_plans, key=lambda x: x.get("confidence", len(x.get("plan", []))))
            return best_plan

        return {"success": False, "error": "All planning approaches failed"}
```

## Integration Testing Example

```python
import unittest
from unittest.mock import Mock, patch

class TestLLMPlanningPipeline(unittest.TestCase):
    def setUp(self):
        # Mock the node and other dependencies
        self.mock_node = Mock()
        self.planner = LLMPlanningPipeline(self.mock_node)

    def test_simple_navigation_plan(self):
        """Test LLM planning for simple navigation"""
        task = "Go to the kitchen"

        with patch('openai.ChatCompletion.create') as mock_create:
            # Mock LLM response
            mock_response = Mock()
            mock_response.choices = [Mock()]
            mock_response.choices[0].message.content = '''
            [
                {
                    "type": "navigate",
                    "parameters": {"x": 5.0, "y": 3.0},
                    "description": "Navigate to kitchen"
                }
            ]
            '''
            mock_create.return_value = mock_response

            result = self.planner.generate_llm_plan(task)

            self.assertTrue(result["success"])
            self.assertEqual(len(result["plan"]), 1)
            self.assertEqual(result["plan"][0]["type"], "navigate")

    def test_complex_manipulation_plan(self):
        """Test LLM planning for complex manipulation"""
        task = "Pick up the red cup and place it on the table"

        with patch('openai.ChatCompletion.create') as mock_create:
            mock_response = Mock()
            mock_response.choices = [Mock()]
            mock_response.choices[0].message.content = '''
            [
                {
                    "type": "navigate",
                    "parameters": {"x": 2.0, "y": 1.0},
                    "description": "Navigate to red cup location"
                },
                {
                    "type": "manipulate",
                    "parameters": {"action": "grasp", "object": "red cup"},
                    "description": "Grasp the red cup"
                },
                {
                    "type": "navigate",
                    "parameters": {"x": 4.0, "y": 2.0},
                    "description": "Navigate to table"
                },
                {
                    "type": "manipulate",
                    "parameters": {"action": "release", "object": "red cup"},
                    "description": "Release cup on table"
                }
            ]
            '''
            mock_create.return_value = mock_response

            result = self.planner.generate_llm_plan(task)

            self.assertTrue(result["success"])
            self.assertEqual(len(result["plan"]), 4)
            self.assertEqual(result["plan"][1]["type"], "manipulate")
            self.assertEqual(result["plan"][1]["parameters"]["action"], "grasp")

if __name__ == '__main__':
    unittest.main()
```

These practical examples demonstrate the complete flow from natural language tasks to LLM-generated action sequences and their execution on robotic systems, showing how to handle various scenarios and edge cases in real-world applications.
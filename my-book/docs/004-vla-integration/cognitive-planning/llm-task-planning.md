---
sidebar_position: 200
title: LLM Task Planning
---

# LLM Task Planning

This section covers using Large Language Models (LLMs) to translate natural language tasks into executable action plans for robotic systems. You'll learn how to leverage the cognitive capabilities of LLMs to create intelligent robotic systems that understand complex human instructions.

## Overview

Large Language Models provide the cognitive layer needed for complex task decomposition in robotic systems. They excel at:

- **Natural Language Understanding**: Converting human instructions into structured representations
- **Task Decomposition**: Breaking complex tasks into executable steps
- **Reasoning**: Applying common sense and domain knowledge to planning
- **Adaptability**: Handling novel situations through generalization

## LLM Integration Approaches

### 1. API-Based Integration

The most common approach for integrating LLMs with robotic systems is through APIs:

```python
import openai
import json
from typing import List, Dict, Any

class LLMBridge:
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        openai.api_key = api_key
        self.model = model

    def plan_task(self, natural_language_task: str, robot_capabilities: List[str] = None) -> Dict[str, Any]:
        """
        Convert natural language task to structured action plan
        """
        capabilities = robot_capabilities or [
            "navigation", "manipulation", "perception", "communication"
        ]

        prompt = f"""
        Convert this natural language task to a structured action plan for a robot.

        Task: "{natural_language_task}"

        Robot capabilities: {capabilities}

        Return a JSON list of actions with these types:
        - navigation: {{"type": "navigate", "x": float, "y": float, "description": string}}
        - manipulation: {{"type": "manipulate", "action": string, "object": string, "description": string}}
        - perception: {{"type": "perceive", "target": string, "description": string}}
        - communication: {{"type": "communicate", "message": string, "description": string}}

        Each action should have:
        - type: The action category
        - description: Human-readable explanation
        - parameters: Action-specific parameters

        Response (JSON only, no additional text):
        """

        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1,  # Low temperature for consistency
                max_tokens=1000
            )

            # Extract and parse the response
            response_text = response.choices[0].message.content.strip()

            # Remove any markdown formatting if present
            if response_text.startswith("```"):
                # Find the JSON part
                start = response_text.find('{')
                end = response_text.rfind('}') + 1
                if start != -1 and end != 0:
                    response_text = response_text[start:end]

            action_plan = json.loads(response_text)
            return {
                "success": True,
                "plan": action_plan,
                "raw_response": response.choices[0].message.content
            }

        except json.JSONDecodeError as e:
            return {
                "success": False,
                "error": f"JSON parsing error: {str(e)}",
                "raw_response": response.choices[0].message.content if 'response' in locals() else None
            }
        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "raw_response": None
            }

# Example usage
llm_bridge = LLMBridge(api_key="your-api-key")

result = llm_bridge.plan_task(
    "Go to the kitchen, find a red apple, and bring it to me",
    ["navigation", "manipulation", "perception"]
)

if result["success"]:
    print("Generated action plan:")
    for i, action in enumerate(result["plan"]):
        print(f"  {i+1}. {action['description']}")
else:
    print(f"Planning failed: {result['error']}")
```

### 2. Specialized Prompt Engineering

For more reliable task planning, use specialized prompts with clear structure:

```python
class SpecializedLLMPlanner:
    def __init__(self, api_key: str):
        openai.api_key = api_key

    def create_structured_plan(self, task: str, environment: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Create a structured plan with environmental context
        """
        env_context = environment or {
            "locations": ["kitchen", "living room", "bedroom", "office"],
            "objects": ["red apple", "blue cup", "book", "keys"],
            "robot_position": [0, 0]
        }

        prompt = f"""
        Task: {task}

        Environment context:
        - Available locations: {env_context['locations']}
        - Available objects: {env_context['objects']}
        - Robot current position: {env_context['robot_position']}

        Generate a step-by-step action plan. Each step must be:
        1. Feasible given the environment
        2. Logically ordered
        3. Include necessary preconditions

        Return JSON with this exact structure:
        {{
            "task": "{task}",
            "plan": [
                {{
                    "step": 1,
                    "action": "navigate|manipulate|perceive|communicate",
                    "parameters": {{}},
                    "description": "What the robot should do",
                    "preconditions": ["list of conditions that must be true"],
                    "expected_outcome": "What should happen"
                }}
            ],
            "estimated_steps": integer,
            "confidence": 0.0-1.0
        }}

        Ensure the plan is executable and safe.
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that creates structured action plans for robots. Always return valid JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1
            )

            plan_json = response.choices[0].message.content.strip()

            # Extract JSON if wrapped in markdown
            if "```json" in plan_json:
                start = plan_json.find("```json") + 7
                end = plan_json.find("```", start)
                plan_json = plan_json[start:end].strip()
            elif plan_json.startswith("```"):
                start = plan_json.find('{')
                end = plan_json.rfind('}') + 1
                plan_json = plan_json[start:end].strip()

            return json.loads(plan_json)

        except Exception as e:
            return {
                "task": task,
                "plan": [],
                "error": str(e),
                "estimated_steps": 0,
                "confidence": 0.0
            }
```

### 3. Chain-of-Thought Reasoning

For complex tasks, use chain-of-thought reasoning to improve planning quality:

```python
class ChainOfThoughtPlanner:
    def __init__(self, api_key: str):
        openai.api_key = api_key

    def plan_with_reasoning(self, task: str) -> Dict[str, Any]:
        """
        Plan task using chain-of-thought reasoning
        """
        prompt = f"""
        Task: {task}

        Think through this step by step:

        1. **Goal Analysis**: What is the ultimate goal?
        2. **Environment Assessment**: What information do I have about the environment?
        3. **Task Decomposition**: What are the subtasks needed to achieve the goal?
        4. **Resource Requirements**: What objects, locations, or capabilities are needed?
        5. **Action Sequence**: What is the logical sequence of actions?
        6. **Potential Issues**: What could go wrong and how to handle it?

        Now generate a structured action plan based on this reasoning.

        Return JSON:
        {{
            "reasoning": {{
                "goal_analysis": "...",
                "environment_assessment": "...",
                "task_decomposition": ["..."],
                "resource_requirements": ["..."],
                "potential_issues": ["..."]
            }},
            "action_plan": [
                {{
                    "step": 1,
                    "action": "...",
                    "parameters": {{}},
                    "description": "..."
                }}
            ]
        }}
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-4",  # Use more capable model for reasoning
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            content = response.choices[0].message.content.strip()

            # Extract the JSON part
            start = content.find('{')
            end = content.rfind('}') + 1
            json_part = content[start:end]

            return json.loads(json_part)

        except Exception as e:
            return {
                "reasoning": {"error": str(e)},
                "action_plan": [],
                "error": str(e)
            }
```

## Integration with ROS 2

Here's how to integrate LLM-based planning with your ROS 2 system:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json

class LLMTaskPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_task_planner_node')

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

        # Initialize LLM bridge
        self.llm_planner = LLMBridge(api_key=self.get_llm_api_key())

        self.get_logger().info('LLM Task Planner node started')

    def get_llm_api_key(self):
        """
        Get LLM API key from parameters or environment
        """
        # In a real system, you'd get this from secure storage
        return self.declare_parameter('llm_api_key', '').value or "your-api-key"

    def task_callback(self, msg):
        """
        Process incoming natural language tasks
        """
        task_text = msg.data
        self.get_logger().info(f'Received task: {task_text}')

        # Update status
        status_msg = String()
        status_msg.data = f"Planning task: {task_text}"
        self.status_publisher.publish(status_msg)

        # Plan the task using LLM
        result = self.llm_planner.plan_task(task_text)

        if result["success"]:
            # Publish the action plan
            plan_msg = String()
            plan_msg.data = json.dumps({
                "task": task_text,
                "plan": result["plan"],
                "timestamp": self.get_clock().now().nanoseconds
            })
            self.plan_publisher.publish(plan_msg)

            self.get_logger().info(f'Generated plan with {len(result["plan"])} steps')
            status_msg.data = f"Plan generated: {len(result["plan"])} steps"
        else:
            self.get_logger().error(f'Planning failed: {result["error"]}')
            status_msg.data = f"Planning failed: {result['error']}"

        self.status_publisher.publish(status_msg)

    def validate_plan(self, plan):
        """
        Validate that the generated plan is executable
        """
        # Check if all action types are supported
        supported_actions = {"navigate", "manipulate", "perceive", "communicate"}

        for step in plan:
            if step.get("type") not in supported_actions:
                return False, f"Unsupported action type: {step.get('type')}"

        # Check for required parameters
        for step in plan:
            action_type = step.get("type")
            if action_type == "navigate" and "x" not in step.get("parameters", {}):
                return False, "Navigation action missing coordinates"
            elif action_type == "manipulate" and "object" not in step.get("parameters", {}):
                return False, "Manipulation action missing object"

        return True, "Plan is valid"

def main(args=None):
    rclpy.init(args=args)
    planner_node = LLMTaskPlannerNode()

    rclpy.spin(planner_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Handling Complex Tasks

For complex multi-step tasks, implement hierarchical planning:

```python
class HierarchicalPlanner:
    def __init__(self, llm_bridge):
        self.llm_bridge = llm_bridge

    def plan_complex_task(self, high_level_task: str) -> Dict[str, Any]:
        """
        Plan complex tasks using hierarchical decomposition
        """
        # First, decompose the high-level task
        decomposition_prompt = f"""
        Decompose this high-level task into subtasks:
        Task: "{high_level_task}"

        Return JSON:
        {{
            "original_task": "{high_level_task}",
            "subtasks": [
                {{"id": 1, "description": "...", "dependencies": []}},
                {{"id": 2, "description": "...", "dependencies": [1]}}
            ]
        }}
        """

        try:
            # Get subtask decomposition
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": decomposition_prompt}],
                temperature=0.1
            )

            decomposition = json.loads(response.choices[0].message.content.strip())

            # Plan each subtask individually
            subtask_plans = {}
            for subtask in decomposition["subtasks"]:
                plan_result = self.llm_bridge.plan_task(subtask["description"])
                if plan_result["success"]:
                    subtask_plans[subtask["id"]] = plan_result["plan"]
                else:
                    return {
                        "success": False,
                        "error": f"Failed to plan subtask {subtask['id']}: {plan_result['error']}"
                    }

            # Combine subtask plans with dependency information
            final_plan = {
                "original_task": high_level_task,
                "decomposition": decomposition,
                "subtask_plans": subtask_plans,
                "execution_order": self.determine_execution_order(decomposition["subtasks"]),
                "success": True
            }

            return final_plan

        except Exception as e:
            return {
                "success": False,
                "error": str(e)
            }

    def determine_execution_order(self, subtasks):
        """
        Determine the order to execute subtasks based on dependencies
        """
        # Simple topological sort implementation
        execution_order = []
        remaining = set(task["id"] for task in subtasks)

        while remaining:
            ready_tasks = []
            for task in subtasks:
                if task["id"] in remaining:
                    deps = set(task["dependencies"])
                    if deps.issubset(set(t["id"] for t in subtasks if t["id"] not in remaining)):
                        ready_tasks.append(task)

            if not ready_tasks:
                raise Exception("Circular dependency detected")

            for task in ready_tasks:
                execution_order.append(task["id"])
                remaining.remove(task["id"])

        return execution_order
```

## Error Handling and Fallbacks

Implement robust error handling for LLM-based planning:

```python
class RobustLLMPlanner:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.fallback_strategies = [
            self.use_simpler_model,
            self.use_rule_based_fallback,
            self.request_human_intervention
        ]

    def plan_with_fallbacks(self, task: str) -> Dict[str, Any]:
        """
        Plan task with multiple fallback strategies
        """
        # Try primary planning method
        result = self.plan_task(task)

        if result["success"]:
            return result

        # Try fallback strategies
        for i, fallback_strategy in enumerate(self.fallback_strategies):
            try:
                fallback_result = fallback_strategy(task, result.get("error"))
                if fallback_result["success"]:
                    # Log that fallback was used
                    fallback_result["fallback_used"] = i + 1
                    return fallback_result
            except Exception as e:
                continue  # Try next fallback

        # All strategies failed
        return {
            "success": False,
            "error": "All planning strategies failed",
            "original_error": result.get("error")
        }

    def use_simpler_model(self, task: str, original_error: str) -> Dict[str, Any]:
        """
        Use a simpler model as fallback
        """
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",  # Fallback to simpler model
                messages=[{"role": "user", "content": f"Simple action plan for: {task}"}],
                temperature=0.1
            )

            # Parse simple response format
            simple_plan = self.parse_simple_response(response.choices[0].message.content)
            return {"success": True, "plan": simple_plan}
        except Exception as e:
            return {"success": False, "error": f"Fallback failed: {str(e)}"}

    def parse_simple_response(self, response: str) -> List[Dict[str, Any]]:
        """
        Parse simple response format for fallback
        """
        # Simple parsing logic for fallback responses
        lines = response.split('\n')
        plan = []

        for line in lines:
            if line.strip().startswith(('1.', '2.', '3.', '4.', '5.')):
                # Extract action from numbered list
                action_text = line.split('.', 1)[1].strip()
                plan.append({
                    "type": "unknown",
                    "description": action_text,
                    "parameters": {}
                })

        return plan
```

## Performance Considerations

- **Caching**: Cache results for common tasks to reduce API calls
- **Rate limiting**: Implement rate limiting to manage API usage
- **Timeout handling**: Set appropriate timeouts for LLM calls
- **Local models**: Consider local LLMs for privacy or latency requirements

## Next Steps

After implementing LLM task planning, the next section covers mapping these plans to ROS 2 behaviors and execution systems.
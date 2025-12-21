---
sidebar_position: 400
title: Complete End-to-End VLA Example
---

# Complete End-to-End VLA Example

This section provides a complete end-to-end example that integrates all Vision-Language-Action (VLA) components into a unified autonomous humanoid system. This example demonstrates how to connect vision, language understanding, and robotic action in a real-world scenario.

## Complete Example: Fetch and Deliver Task

Let's walk through a complete example where the humanoid robot receives a voice command, processes it using LLMs, and executes a complex task involving navigation, perception, and manipulation.

### Scenario: "Bring me the red cup from the kitchen table"

This is a complex task that requires:
1. **Voice processing**: Understanding the spoken command
2. **LLM planning**: Breaking down the task into steps
3. **Navigation**: Going to the kitchen
4. **Perception**: Finding the red cup
5. **Manipulation**: Grasping the cup
6. **Navigation**: Returning to the user
7. **Communication**: Reporting completion

### Complete System Integration

```python
#!/usr/bin/env python3
"""
Complete End-to-End VLA Example
Demonstrates integration of all VLA components for a fetch and deliver task
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
import json
import openai
import time
from typing import Dict, Any, List


class CompleteVLADemoNode(Node):
    def __init__(self):
        super().__init__('complete_vla_demo')

        # Initialize all components
        self.voice_processor = VoiceProcessor(self)
        self.llm_planner = LLMTaskPlanner(self)
        self.vla_pipeline = VLAPipeline(self)
        self.workflow_manager = WorkflowManager(self)

        # Publishers
        self.status_publisher = self.create_publisher(String, 'vla_demo_status', 10)
        self.task_result_publisher = self.create_publisher(String, 'task_result', 10)

        # Subscriber for demo start
        self.demo_subscriber = self.create_subscription(
            String,
            'start_vla_demo',
            self.start_demo_callback,
            10
        )

        self.get_logger().info('Complete VLA Demo node initialized')

    def start_demo_callback(self, msg):
        """
        Start the complete VLA demo
        """
        try:
            demo_config = json.loads(msg.data)
            task_description = demo_config.get('task', 'Bring me the red cup from the kitchen table')

            self.get_logger().info(f'Starting VLA demo with task: {task_description}')
            self.publish_status(f'Demo starting: {task_description}')

            # Execute complete end-to-end task
            result = self.execute_complete_task(task_description)

            # Publish result
            result_msg = String()
            result_msg.data = json.dumps(result)
            self.task_result_publisher.publish(result_msg)

            if result['success']:
                self.get_logger().info('VLA demo completed successfully!')
                self.publish_status('Demo completed successfully!')
            else:
                self.get_logger().error(f'VLA demo failed: {result.get("error")}')
                self.publish_status(f'Demo failed: {result.get("error")}')

        except Exception as e:
            self.get_logger().error(f'Demo execution error: {e}')
            self.publish_status(f'Demo error: {e}')

    def execute_complete_task(self, task_description):
        """
        Execute complete end-to-end task integrating all VLA components
        """
        start_time = time.time()

        try:
            # Phase 1: Voice Processing
            self.publish_status('Phase 1: Processing voice command...')
            voice_result = self.voice_processor.process_command(task_description)

            if not voice_result['success']:
                return {
                    'success': False,
                    'phase': 'voice_processing',
                    'error': f'Voice processing failed: {voice_result.get("error")}',
                    'total_time': time.time() - start_time
                }

            # Phase 2: LLM Planning
            self.publish_status('Phase 2: Generating action plan with LLM...')
            plan_result = self.llm_planner.generate_plan(voice_result['command'])

            if not plan_result['success']:
                return {
                    'success': False,
                    'phase': 'llm_planning',
                    'error': f'LLM planning failed: {plan_result.get("error")}',
                    'total_time': time.time() - start_time
                }

            # Phase 3: VLA Pipeline Execution
            self.publish_status('Phase 3: Executing VLA pipeline...')
            pipeline_result = self.vla_pipeline.execute_plan(plan_result['plan'])

            if not pipeline_result['success']:
                return {
                    'success': False,
                    'phase': 'pipeline_execution',
                    'error': f'Pipeline execution failed: {pipeline_result.get("error")}',
                    'total_time': time.time() - start_time
                }

            # Phase 4: Workflow Management
            self.publish_status('Phase 4: Managing workflow completion...')
            workflow_result = self.workflow_manager.complete_task(pipeline_result)

            if not workflow_result['success']:
                return {
                    'success': False,
                    'phase': 'workflow_management',
                    'error': f'Workflow completion failed: {workflow_result.get("error")}',
                    'total_time': time.time() - start_time
                }

            # Task completed successfully
            return {
                'success': True,
                'phases': {
                    'voice_processing': voice_result,
                    'llm_planning': plan_result,
                    'pipeline_execution': pipeline_result,
                    'workflow_management': workflow_result
                },
                'total_time': time.time() - start_time
            }

        except Exception as e:
            return {
                'success': False,
                'error': str(e),
                'total_time': time.time() - start_time
            }

    def publish_status(self, status_message):
        """
        Publish status message
        """
        msg = String()
        msg.data = status_message
        self.status_publisher.publish(msg)


class VoiceProcessor:
    def __init__(self, node):
        self.node = node

    def process_command(self, command_text):
        """
        Process voice command (simulated for this example)
        """
        try:
            # In a real system, this would involve:
            # - Speech recognition
            # - Natural language processing
            # - Command validation
            # - Intent extraction

            # For this demo, we'll just validate the input
            if not command_text or len(command_text.strip()) == 0:
                return {
                    'success': False,
                    'error': 'Empty command'
                }

            # Extract intent and entities (simplified)
            intent = self.extract_intent(command_text)
            entities = self.extract_entities(command_text)

            return {
                'success': True,
                'command': command_text,
                'intent': intent,
                'entities': entities,
                'processed_text': command_text.lower().strip()
            }

        except Exception as e:
            return {
                'success': False,
                'error': f'Voice processing error: {str(e)}'
            }

    def extract_intent(self, command_text):
        """
        Extract intent from command text
        """
        text_lower = command_text.lower()

        if any(word in text_lower for word in ['bring', 'fetch', 'get', 'carry']):
            return 'fetch_object'
        elif any(word in text_lower for word in ['go to', 'navigate', 'move to']):
            return 'navigate'
        elif any(word in text_lower for word in ['pick up', 'grasp', 'take']):
            return 'grasp_object'
        else:
            return 'unknown'

    def extract_entities(self, command_text):
        """
        Extract entities from command text
        """
        text_lower = command_text.lower()
        entities = {}

        # Extract object (simplified)
        if 'red cup' in text_lower:
            entities['object'] = 'red cup'
            entities['color'] = 'red'
            entities['type'] = 'cup'
        elif 'blue bottle' in text_lower:
            entities['object'] = 'blue bottle'
            entities['color'] = 'blue'
            entities['type'] = 'bottle'

        # Extract location
        if 'kitchen' in text_lower:
            entities['location'] = 'kitchen'
        elif 'living room' in text_lower:
            entities['location'] = 'living room'
        elif 'office' in text_lower:
            entities['location'] = 'office'

        return entities


class LLMTaskPlanner:
    def __init__(self, node):
        self.node = node
        self.api_key = node.declare_parameter('llm_api_key', '').value or "your-api-key"
        openai.api_key = self.api_key

    def generate_plan(self, natural_language_task):
        """
        Generate detailed action plan from natural language task
        """
        try:
            # Create detailed prompt for the specific task
            prompt = f"""
            Task: "{natural_language_task}"

            Generate a detailed step-by-step action plan for a humanoid robot to execute this task.

            Robot capabilities:
            - Navigation: Can move to specific locations in the environment
            - Perception: Can detect and identify objects
            - Manipulation: Can grasp and manipulate objects
            - Communication: Can provide status updates

            Environment context:
            - Known locations: kitchen, living room, office, bedroom
            - Common objects: cups, bottles, books, tables, chairs
            - Robot starts at: current location (assumed to be near user)

            Generate a JSON list of actions with these types:
            - navigation: {{"type": "navigate", "target_location": string, "description": string}}
            - perception: {{"type": "perceive", "target_object": string, "location": string, "description": string}}
            - manipulation: {{"type": "manipulate", "action": "grasp|place|move", "object": string, "description": string}}
            - communication: {{"type": "communicate", "message": string, "description": string}}

            Each action should include:
            - type: The action category
            - description: What the robot should do
            - parameters: Action-specific parameters

            Response (JSON only, no additional text):
            """

            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            response_text = response.choices[0].message.content.strip()

            # Extract JSON if wrapped in markdown
            if response_text.startswith("```"):
                start = response_text.find('{')
                end = response_text.rfind('}') + 1
                response_text = response_text[start:end].strip()

            action_plan = json.loads(response_text)

            return {
                'success': True,
                'plan': action_plan,
                'raw_response': response.choices[0].message.content
            }

        except json.JSONDecodeError as e:
            return {
                'success': False,
                'error': f'JSON parsing error: {str(e)}',
                'raw_response': response.choices[0].message.content if 'response' in locals() else None
            }
        except Exception as e:
            return {
                'success': False,
                'error': f'LLM planning error: {str(e)}'
            }


class VLAPipeline:
    def __init__(self, node):
        self.node = node
        self.navigation_system = NavigationSystem(node)
        self.perception_system = PerceptionSystem(node)
        self.manipulation_system = ManipulationSystem(node)
        self.communication_system = CommunicationSystem(node)

    def execute_plan(self, plan):
        """
        Execute the complete action plan
        """
        try:
            results = []

            for i, action in enumerate(plan):
                self.node.get_logger().info(f'Executing step {i+1}: {action.get("description", "")}')

                # Execute based on action type
                action_result = self.execute_action(action)

                if not action_result['success']:
                    self.node.get_logger().error(f'Step {i+1} failed: {action_result.get("error")}')

                    # Try recovery if possible
                    recovery_result = self.attempt_recovery(action, action_result)

                    if not recovery_result['success']:
                        return {
                            'success': False,
                            'error': f'Step {i+1} failed and recovery unsuccessful: {action_result.get("error")}',
                            'failed_step': i+1,
                            'failed_action': action,
                            'attempted_recovery': recovery_result
                        }

                results.append({
                    'step': i+1,
                    'action': action,
                    'result': action_result
                })

            return {
                'success': True,
                'steps_executed': len(results),
                'step_results': results
            }

        except Exception as e:
            return {
                'success': False,
                'error': f'Pipeline execution error: {str(e)}'
            }

    def execute_action(self, action):
        """
        Execute individual action based on type
        """
        action_type = action.get('type', 'unknown')

        if action_type == 'navigate':
            return self.navigation_system.navigate_to_location(action.get('parameters', {}))
        elif action_type == 'perceive':
            return self.perception_system.perceive_object(action.get('parameters', {}))
        elif action_type == 'manipulate':
            return self.manipulation_system.manipulate_object(action.get('parameters', {}))
        elif action_type == 'communicate':
            return self.communication_system.communicate_message(action.get('parameters', {}))
        else:
            return {
                'success': False,
                'error': f'Unknown action type: {action_type}'
            }

    def attempt_recovery(self, failed_action, failure_result):
        """
        Attempt to recover from action failure
        """
        # Simple recovery strategies
        action_type = failed_action.get('type', 'unknown')

        if action_type == 'navigate':
            # Try alternative path or return to safe position
            return self.navigation_system.attempt_alternative_navigation(failed_action)
        elif action_type == 'perceive':
            # Try different perception approach
            return self.perception_system.attempt_alternative_perception(failed_action)
        elif action_type == 'manipulate':
            # Try different grasp approach
            return self.manipulation_system.attempt_alternative_manipulation(failed_action)
        else:
            return {
                'success': False,
                'error': 'No recovery strategy for this action type'
            }


class NavigationSystem:
    def __init__(self, node):
        self.node = node

    def navigate_to_location(self, params):
        """
        Navigate to specified location
        """
        try:
            target_location = params.get('target_location', 'unknown')

            # In a real system, this would:
            # - Look up location coordinates
            # - Plan path
            # - Execute navigation
            # - Monitor progress

            # Simulate navigation
            self.node.get_logger().info(f'Navigating to {target_location}...')

            # Simulate time for navigation
            time.sleep(2.0)  # 2 seconds navigation time

            # Simulate success
            return {
                'success': True,
                'location': target_location,
                'navigation_time': 2.0
            }

        except Exception as e:
            return {
                'success': False,
                'error': f'Navigation error: {str(e)}'
            }

    def attempt_alternative_navigation(self, failed_action):
        """
        Attempt alternative navigation approach
        """
        # Try backup strategy
        params = failed_action.get('parameters', {})
        target_location = params.get('target_location', 'unknown')

        self.node.get_logger().info(f'Attempting alternative navigation to {target_location}...')

        # Simulate alternative navigation
        time.sleep(1.5)

        return {
            'success': True,
            'location': target_location,
            'strategy': 'alternative_path'
        }


class PerceptionSystem:
    def __init__(self, node):
        self.node = node

    def perceive_object(self, params):
        """
        Perceive target object in specified location
        """
        try:
            target_object = params.get('target_object', 'unknown')
            location = params.get('location', 'unknown')

            self.node.get_logger().info(f'Perceiving {target_object} in {location}...')

            # Simulate perception process
            time.sleep(1.0)  # 1 second perception time

            # Simulate object detection
            # In a real system, this would use cameras, LIDAR, etc.
            detected = self.simulate_object_detection(target_object)

            if detected:
                return {
                    'success': True,
                    'object': target_object,
                    'location': location,
                    'detected': True,
                    'coordinates': {'x': 1.5, 'y': 2.0, 'z': 0.8}
                }
            else:
                return {
                    'success': False,
                    'error': f'Could not detect {target_object} in {location}',
                    'object': target_object,
                    'location': location,
                    'detected': False
                }

        except Exception as e:
            return {
                'success': False,
                'error': f'Perception error: {str(e)}'
            }

    def simulate_object_detection(self, target_object):
        """
        Simulate object detection (in real system, this would use computer vision)
        """
        # Simulate detection based on object type
        import random
        return random.random() > 0.1  # 90% success rate for demonstration

    def attempt_alternative_perception(self, failed_action):
        """
        Attempt alternative perception approach
        """
        params = failed_action.get('parameters', {})
        target_object = params.get('target_object', 'unknown')

        self.node.get_logger().info(f'Attempting alternative perception for {target_object}...')

        # Try different approach - change lighting, camera angle, etc.
        time.sleep(0.8)

        # Simulate successful detection with alternative approach
        return {
            'success': True,
            'object': target_object,
            'strategy': 'alternative_perception'
        }


class ManipulationSystem:
    def __init__(self, node):
        self.node = node

    def manipulate_object(self, params):
        """
        Manipulate target object
        """
        try:
            action = params.get('action', 'unknown')
            obj = params.get('object', 'unknown')

            self.node.get_logger().info(f'Attempting to {action} {obj}...')

            # Simulate manipulation process
            time.sleep(1.5)  # 1.5 seconds manipulation time

            # Simulate success/failure based on action type
            success = self.simulate_manipulation_success(action, obj)

            if success:
                return {
                    'success': True,
                    'action': action,
                    'object': obj,
                    'manipulation_time': 1.5
                }
            else:
                return {
                    'success': False,
                    'error': f'Could not {action} {obj}',
                    'action': action,
                    'object': obj
                }

        except Exception as e:
            return {
                'success': False,
                'error': f'Manipulation error: {str(e)}'
            }

    def simulate_manipulation_success(self, action, obj):
        """
        Simulate manipulation success (in real system, this would control actual robot)
        """
        import random
        return random.random() > 0.2  # 80% success rate for demonstration

    def attempt_alternative_manipulation(self, failed_action):
        """
        Attempt alternative manipulation approach
        """
        params = failed_action.get('parameters', {})
        action = params.get('action', 'unknown')
        obj = params.get('object', 'unknown')

        self.node.get_logger().info(f'Attempting alternative manipulation for {action} {obj}...')

        # Try different grasp strategy, approach angle, etc.
        time.sleep(1.0)

        return {
            'success': True,
            'action': action,
            'object': obj,
            'strategy': 'alternative_manipulation'
        }


class CommunicationSystem:
    def __init__(self, node):
        self.node = node

    def communicate_message(self, params):
        """
        Communicate message to user
        """
        try:
            message = params.get('message', 'No message')

            self.node.get_logger().info(f'Communicating: {message}')

            # In a real system, this would use text-to-speech, etc.
            # For simulation, just log the message
            self.node.get_logger().info(f'Communicated: {message}')

            return {
                'success': True,
                'message': message,
                'delivered': True
            }

        except Exception as e:
            return {
                'success': False,
                'error': f'Communication error: {str(e)}'
            }


class WorkflowManager:
    def __init__(self, node):
        self.node = node

    def complete_task(self, pipeline_result):
        """
        Complete task workflow and report results
        """
        try:
            # Verify task completion
            if pipeline_result['success']:
                # Task completed successfully
                self.node.get_logger().info('Task completed successfully!')

                # Communicate completion
                completion_msg = {
                    'type': 'communicate',
                    'parameters': {
                        'message': 'Task completed successfully!',
                        'description': 'Report task completion to user'
                    }
                }

                comm_system = CommunicationSystem(self.node)
                comm_result = comm_system.communicate_message(completion_msg['parameters'])

                return {
                    'success': True,
                    'completion_status': 'success',
                    'communication_result': comm_result
                }
            else:
                # Task failed
                error_msg = pipeline_result.get('error', 'Unknown error')
                self.node.get_logger().error(f'Task failed: {error_msg}')

                # Communicate failure
                failure_msg = {
                    'type': 'communicate',
                    'parameters': {
                        'message': f'Task failed: {error_msg}',
                        'description': 'Report task failure to user'
                    }
                }

                comm_system = CommunicationSystem(self.node)
                comm_result = comm_system.communicate_message(failure_msg['parameters'])

                return {
                    'success': False,
                    'completion_status': 'failed',
                    'error': error_msg,
                    'communication_result': comm_result
                }

        except Exception as e:
            return {
                'success': False,
                'error': f'Workflow completion error: {str(e)}'
            }


def main(args=None):
    rclpy.init(args=args)
    demo_node = CompleteVLADemoNode()

    try:
        rclpy.spin(demo_node)
    except KeyboardInterrupt:
        demo_node.get_logger().info('VLA Demo interrupted by user')
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Simulation Example

### 1. Gazebo Simulation Integration

```python
class SimulationIntegration:
    def __init__(self, node):
        self.node = node
        # Initialize Gazebo interfaces
        self.gazebo_interface = GazeboInterface(node)

    def setup_simulation_scenario(self, task_description):
        """
        Set up simulation scenario for the given task
        """
        # Parse task to determine required objects and environment
        scenario_config = self.parse_task_for_simulation(task_description)

        # Spawn required objects in Gazebo
        for obj in scenario_config['objects']:
            self.gazebo_interface.spawn_object(obj)

        # Set up initial poses
        for pose_config in scenario_config['poses']:
            self.gazebo_interface.set_model_pose(pose_config['model'], pose_config['pose'])

        return scenario_config

    def parse_task_for_simulation(self, task_description):
        """
        Parse task description to determine simulation requirements
        """
        config = {
            'objects': [],
            'poses': [],
            'environment': 'default'
        }

        # Extract object information
        if 'red cup' in task_description.lower():
            config['objects'].append({
                'name': 'red_cup',
                'type': 'cup',
                'color': 'red',
                'pose': {'x': 5.0, 'y': 3.0, 'z': 0.8}
            })

        if 'kitchen' in task_description.lower():
            config['environment'] = 'kitchen_scene'

        # Extract other requirements
        if 'table' in task_description.lower():
            config['objects'].append({
                'name': 'table',
                'type': 'table',
                'pose': {'x': 5.0, 'y': 3.0, 'z': 0.0}
            })

        return config

    def run_simulation_test(self, task_description):
        """
        Run complete simulation test
        """
        # Set up simulation environment
        scenario = self.setup_simulation_scenario(task_description)

        # Initialize VLA system in simulation
        vla_system = CompleteVLADemoNode(self.node)

        # Start the task
        start_msg = String()
        start_msg.data = json.dumps({'task': task_description})

        # In simulation, we'd publish to the start topic
        # vla_system.start_demo_callback(start_msg)

        return {
            'success': True,
            'scenario': scenario,
            'task': task_description
        }
```

## Performance Monitoring Example

### 2. Real-Time Performance Monitoring

```python
class PerformanceMonitor:
    def __init__(self, node):
        self.node = node
        self.metrics = {
            'response_times': [],
            'success_rates': [],
            'error_counts': {},
            'throughput': []
        }

    def monitor_task_execution(self, task_description, execution_function):
        """
        Monitor performance during task execution
        """
        start_time = time.time()

        try:
            # Execute the task
            result = execution_function()

            # Calculate metrics
            execution_time = time.time() - start_time

            # Update metrics
            self.metrics['response_times'].append(execution_time)

            if result['success']:
                self.metrics['success_rates'].append(1.0)
            else:
                self.metrics['success_rates'].append(0.0)
                error_type = result.get('error', 'unknown_error')
                self.metrics['error_counts'][error_type] = self.metrics['error_counts'].get(error_type, 0) + 1

            # Calculate rolling averages
            avg_response_time = sum(self.metrics['response_times'][-10:]) / min(len(self.metrics['response_times']), 10)
            avg_success_rate = sum(self.metrics['success_rates'][-10:]) / min(len(self.metrics['success_rates']), 10)

            # Log performance
            self.node.get_logger().info(f'Performance - Time: {execution_time:.2f}s, Avg: {avg_response_time:.2f}s, Success Rate: {avg_success_rate:.2%}')

            return {
                'success': True,
                'result': result,
                'metrics': {
                    'execution_time': execution_time,
                    'average_response_time': avg_response_time,
                    'average_success_rate': avg_success_rate
                }
            }

        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }

    def get_performance_report(self):
        """
        Generate performance report
        """
        if not self.metrics['response_times']:
            return {'error': 'No performance data collected'}

        return {
            'total_executions': len(self.metrics['response_times']),
            'average_response_time': sum(self.metrics['response_times']) / len(self.metrics['response_times']),
            'min_response_time': min(self.metrics['response_times']),
            'max_response_time': max(self.metrics['response_times']),
            'success_rate': sum(self.metrics['success_rates']) / len(self.metrics['success_rates']) if self.metrics['success_rates'] else 0,
            'error_summary': self.metrics['error_counts']
        }
```

## Error Handling Example

### 3. Comprehensive Error Handling

```python
class RobustVLASystem:
    def __init__(self, node):
        self.node = node
        self.error_handlers = {
            'navigation_error': self.handle_navigation_error,
            'perception_error': self.handle_perception_error,
            'manipulation_error': self.handle_manipulation_error,
            'communication_error': self.handle_communication_error
        }

    def execute_with_error_handling(self, task_description):
        """
        Execute task with comprehensive error handling
        """
        try:
            # Phase 1: Voice Processing
            voice_result = self.process_voice_with_error_handling(task_description)
            if not voice_result['success']:
                return self.handle_phase_error('voice', voice_result)

            # Phase 2: Planning
            plan_result = self.plan_with_error_handling(voice_result['command'])
            if not plan_result['success']:
                return self.handle_phase_error('planning', plan_result)

            # Phase 3: Execution
            execution_result = self.execute_with_error_handling(plan_result['plan'])
            if not execution_result['success']:
                return self.handle_phase_error('execution', execution_result)

            return {
                'success': True,
                'final_result': execution_result
            }

        except Exception as e:
            return self.handle_unexpected_error(e)

    def handle_phase_error(self, phase, result):
        """
        Handle error in specific phase
        """
        error_type = result.get('error_type', 'unknown')
        error_message = result.get('error', str(result))

        self.node.get_logger().error(f'{phase.capitalize()} phase failed: {error_message}')

        # Try recovery based on error type
        if error_type in self.error_handlers:
            return self.error_handlers[error_type](result)
        else:
            # Generic recovery
            return self.generic_error_recovery(phase, result)

    def handle_navigation_error(self, error_result):
        """
        Handle navigation-specific errors
        """
        # Log error
        self.node.get_logger().error(f'Navigation error: {error_result.get("error")}')

        # Try alternative navigation
        alternative_result = self.try_alternative_navigation(error_result)

        if alternative_result['success']:
            return alternative_result

        # If alternatives fail, return to safe state
        safe_return = self.return_to_safe_state()
        return {
            'success': False,
            'error': f'Navigation failed and recovery unsuccessful: {error_result.get("error")}',
            'recovery_attempt': safe_return
        }

    def handle_perception_error(self, error_result):
        """
        Handle perception-specific errors
        """
        self.node.get_logger().error(f'Perception error: {error_result.get("error")}')

        # Try different perception approach
        alternative_result = self.try_alternative_perception(error_result)

        if alternative_result['success']:
            return alternative_result

        return {
            'success': False,
            'error': f'Perception failed and alternatives unsuccessful: {error_result.get("error")}'
        }

    def handle_manipulation_error(self, error_result):
        """
        Handle manipulation-specific errors
        """
        self.node.get_logger().error(f'Manipulation error: {error_result.get("error")}')

        # Try different manipulation approach
        alternative_result = self.try_alternative_manipulation(error_result)

        if alternative_result['success']:
            return alternative_result

        return {
            'success': False,
            'error': f'Manipulation failed and alternatives unsuccessful: {error_result.get("error")}'
        }

    def handle_communication_error(self, error_result):
        """
        Handle communication-specific errors
        """
        self.node.get_logger().error(f'Communication error: {error_result.get("error")}')

        # Try alternative communication method
        alternative_result = self.try_alternative_communication(error_result)

        if alternative_result['success']:
            return alternative_result

        return {
            'success': False,
            'error': f'Communication failed and alternatives unsuccessful: {error_result.get("error")}'
        }

    def generic_error_recovery(self, phase, error_result):
        """
        Generic error recovery for unknown error types
        """
        self.node.get_logger().error(f'Generic recovery for {phase} error: {error_result.get("error")}')

        # Emergency stop and report
        self.emergency_stop()

        return {
            'success': False,
            'error': f'{phase} phase failed: {error_result.get("error")}',
            'recovery': 'emergency_stop_executed'
        }

    def handle_unexpected_error(self, exception):
        """
        Handle unexpected errors
        """
        self.node.get_logger().error(f'Unexpected error: {str(exception)}')

        # Emergency stop
        self.emergency_stop()

        return {
            'success': False,
            'error': f'Unexpected error: {str(exception)}',
            'recovery': 'emergency_stop_executed'
        }

    def emergency_stop(self):
        """
        Execute emergency stop procedure
        """
        self.node.get_logger().warn('Emergency stop procedure initiated!')
        # In a real system, this would stop all robot motion and return to safe state
        pass

    def return_to_safe_state(self):
        """
        Return robot to safe state
        """
        self.node.get_logger().info('Returning to safe state...')
        # Implementation would return robot to known safe location/configuration
        return {'success': True, 'state': 'safe'}
```

This complete end-to-end example demonstrates how all VLA components work together in a unified autonomous humanoid system, providing a comprehensive reference implementation for students to understand and build upon.
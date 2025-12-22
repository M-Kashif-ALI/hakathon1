---
sidebar_position: 300
title: ROS 2 Mapping
---

# ROS 2 Mapping

This section covers mapping LLM-generated action plans to ROS 2 behaviors and execution systems. You'll learn how to connect high-level cognitive plans to low-level robotic execution in ROS 2 environments.

## Overview

The mapping layer is critical for connecting LLM-generated plans to actual robot behaviors. It involves:

- **Action type mapping**: Converting LLM action types to ROS 2 action servers
- **Parameter translation**: Converting LLM parameters to ROS 2 message formats
- **Behavior execution**: Executing plans using ROS 2 behaviors and action servers
- **Feedback integration**: Providing execution feedback to the LLM system

## ROS 2 Action Server Integration

### 1. Navigation Action Mapping

Map LLM navigation actions to ROS 2 navigation systems:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json

class NavigationMapperNode(Node):
    def __init__(self):
        super().__init__('navigation_mapper_node')

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriber for LLM-generated navigation commands
        self.nav_subscriber = self.create_subscription(
            String,
            'llm_navigation_commands',
            self.nav_command_callback,
            10
        )

        self.get_logger().info('Navigation mapper node started')

    def nav_command_callback(self, msg):
        """
        Process LLM-generated navigation commands
        """
        try:
            # Parse the navigation command from LLM
            command_data = json.loads(msg.data)

            if command_data.get('type') != 'navigate':
                return  # Not a navigation command

            # Extract navigation parameters
            x = command_data.get('parameters', {}).get('x', 0.0)
            y = command_data.get('parameters', {}).get('y', 0.0)
            theta = command_data.get('parameters', {}).get('theta', 0.0)

            # Create navigation goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y
            goal_msg.pose.pose.position.z = 0.0

            # Convert theta to quaternion
            from math import sin, cos
            goal_msg.pose.pose.orientation.z = sin(theta / 2.0)
            goal_msg.pose.pose.orientation.w = cos(theta / 2.0)

            # Wait for action server
            self.nav_client.wait_for_server()

            # Send navigation goal
            future = self.nav_client.send_goal_async(goal_msg)
            future.add_done_callback(self.nav_goal_response_callback)

        except Exception as e:
            self.get_logger().error(f'Error processing navigation command: {e}')

    def nav_goal_response_callback(self, future):
        """
        Handle navigation goal response
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """
        Handle navigation result
        """
        result = future.result().result
        self.get_logger().info(f'Navigation completed with result: {result}')
```

### 2. Manipulation Action Mapping

Map LLM manipulation actions to robot manipulation systems:

```python
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class ManipulationMapperNode(Node):
    def __init__(self):
        super().__init__('manipulation_mapper_node')

        # Action client for manipulation
        self.manip_client = ActionClient(
            self,
            FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory'
        )

        # Subscriber for LLM-generated manipulation commands
        self.manip_subscriber = self.create_subscription(
            String,
            'llm_manipulation_commands',
            self.manip_command_callback,
            10
        )

        # Joint state subscriber to know current state
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.current_joint_positions = {}
        self.joint_names = ['joint1', 'joint2', 'joint3', 'gripper_joint']  # Example joint names

        self.get_logger().info('Manipulation mapper node started')

    def joint_state_callback(self, msg):
        """
        Update current joint positions
        """
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def manip_command_callback(self, msg):
        """
        Process LLM-generated manipulation commands
        """
        try:
            command_data = json.loads(msg.data)

            if command_data.get('type') != 'manipulate':
                return  # Not a manipulation command

            action = command_data.get('parameters', {}).get('action', '')
            target_object = command_data.get('parameters', {}).get('object', '')

            if action == 'grasp':
                self.execute_grasp(target_object)
            elif action == 'release':
                self.execute_release()
            elif action == 'move_to':
                self.execute_move_to(command_data.get('parameters', {}))
            else:
                self.get_logger().warn(f'Unknown manipulation action: {action}')

        except Exception as e:
            self.get_logger().error(f'Error processing manipulation command: {e}')

    def execute_grasp(self, object_name):
        """
        Execute grasping action
        """
        # In a real system, this would:
        # 1. Navigate to object location
        # 2. Plan grasp trajectory
        # 3. Execute grasp with gripper
        # 4. Verify grasp success

        # Example: Close gripper
        trajectory_msg = self.create_gripper_trajectory(gripper_position=0.0)  # Closed
        self.execute_trajectory(trajectory_msg)

    def execute_release(self):
        """
        Execute release action
        """
        # Open gripper
        trajectory_msg = self.create_gripper_trajectory(gripper_position=0.8)  # Open
        self.execute_trajectory(trajectory_msg)

    def create_gripper_trajectory(self, gripper_position):
        """
        Create trajectory for gripper movement
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = ['gripper_joint']  # or appropriate joint names

        point = JointTrajectoryPoint()
        point.positions = [gripper_position]
        point.time_from_start.sec = 2  # 2 seconds to complete
        trajectory.points = [point]

        return trajectory

    def execute_trajectory(self, trajectory_msg):
        """
        Execute joint trajectory
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg

        self.manip_client.wait_for_server()
        future = self.manip_client.send_goal_async(goal_msg)
        future.add_done_callback(self.manip_goal_response_callback)

    def manip_goal_response_callback(self, future):
        """
        Handle manipulation goal response
        """
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.manip_result_callback)

    def manip_result_callback(self, future):
        """
        Handle manipulation result
        """
        result = future.result().result
        self.get_logger().info(f'Manipulation completed: {result}')
```

## Behavior Tree Integration

Integrate with ROS 2 behavior trees for complex task execution:

```python
import py_trees
from py_trees.behaviours import SuccessAlways
from py_trees.composites import Sequence, Selector
from py_trees.decorators import Retry, Timeout

class LLMPlanExecutor:
    def __init__(self, node):
        self.node = node
        self.blackboard = py_trees.blackboard.Blackboard()

    def convert_llm_plan_to_behavior_tree(self, llm_plan):
        """
        Convert LLM-generated plan to ROS 2 behavior tree
        """
        # Create root sequence for the entire plan
        root = Sequence(name="LLM_Plan_Root")

        for step in llm_plan:
            behavior = self.create_behavior_for_step(step)
            root.add_child(behavior)

        return root

    def create_behavior_for_step(self, step):
        """
        Create behavior tree node for individual LLM plan step
        """
        action_type = step.get('type', 'unknown')

        if action_type == 'navigate':
            return self.create_navigation_behavior(step)
        elif action_type == 'manipulate':
            return self.create_manipulation_behavior(step)
        elif action_type == 'perceive':
            return self.create_perception_behavior(step)
        elif action_type == 'communicate':
            return self.create_communication_behavior(step)
        else:
            # Default to success for unknown actions
            return SuccessAlways(name=f"Unknown_Action_{step.get('description', 'N/A')}")

    def create_navigation_behavior(self, step):
        """
        Create navigation behavior with retry and timeout
        """
        # Extract navigation parameters
        params = step.get('parameters', {})
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)

        # Create navigation task
        nav_task = NavigateToPositionTask(self.node, x, y, step.get('description', ''))

        # Add timeout and retry decorators
        nav_with_timeout = Timeout(
            name=f"Navigate_Timeout_{x}_{y}",
            duration=30.0,  # 30 second timeout
            child=nav_task
        )

        nav_with_retry = Retry(
            name=f"Navigate_Retry_{x}_{y}",
            num_attempts=3,  # Try up to 3 times
            child=nav_with_timeout
        )

        return nav_with_retry

    def create_manipulation_behavior(self, step):
        """
        Create manipulation behavior
        """
        params = step.get('parameters', {})
        action = params.get('action', '')
        obj = params.get('object', '')

        manip_task = ManipulationTask(self.node, action, obj, step.get('description', ''))

        # Add safety checks and error handling
        return manip_task

class NavigateToPositionTask(py_trees.behaviour.Behaviour):
    def __init__(self, node, x, y, description=""):
        super().__init__(name=f"NavigateTo_{x}_{y}")
        self.node = node
        self.x = x
        self.y = y
        self.description = description
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None

    def initialise(self):
        """
        Initialize navigation goal
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = self.x
        goal_msg.pose.pose.position.y = self.y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # No rotation

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle goal response
        """
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.feedback_message = "Goal rejected"
            return

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """
        Handle result
        """
        result = future.result().result
        self.feedback_message = f"Navigation result: {result}"

    def update(self):
        """
        Update behavior status
        """
        if self.goal_handle is None:
            return py_trees.common.Status.RUNNING

        # Check if goal is still active
        if self.goal_handle.status == 2:  # STATUS_ACTIVE
            return py_trees.common.Status.RUNNING
        elif self.goal_handle.status == 3:  # STATUS_SUCCEEDED
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
```

## Plan Execution Pipeline

Create a complete pipeline for executing LLM-generated plans:

```python
class PlanExecutionPipeline:
    def __init__(self, node):
        self.node = node
        self.navigation_mapper = NavigationMapperNode()
        self.manipulation_mapper = ManipulationMapperNode()
        self.plan_executor = LLMPlanExecutor(node)

        # Publisher for execution status
        self.status_publisher = node.create_publisher(String, 'plan_execution_status', 10)

    def execute_plan(self, llm_plan, plan_id=None):
        """
        Execute an LLM-generated plan
        """
        plan_id = plan_id or self.generate_plan_id()

        self.publish_status(f"Starting execution of plan {plan_id}")

        try:
            # Convert LLM plan to behavior tree
            behavior_tree = self.plan_executor.convert_llm_plan_to_behavior_tree(llm_plan)

            # Setup tree manager
            tree_manager = py_trees.trees.BehaviourTree(behavior_tree)

            # Add post-tick handler to monitor progress
            tree_manager.add_post_tick_handler(self.post_tick_handler)

            # Run the tree
            tree_manager.setup(timeout=15.0)

            # Execute until completion
            while not self.is_tree_complete(tree_manager):
                tree_manager.tick()

                # Check for preemption or cancellation
                if self.should_cancel_execution(plan_id):
                    self.publish_status(f"Plan {plan_id} cancelled")
                    return False

                # Brief sleep to prevent busy waiting
                import time
                time.sleep(0.1)

            # Check final status
            if tree_manager.root.status == py_trees.common.Status.SUCCESS:
                self.publish_status(f"Plan {plan_id} completed successfully")
                return True
            else:
                self.publish_status(f"Plan {plan_id} failed")
                return False

        except Exception as e:
            self.publish_status(f"Plan {plan_id} execution error: {str(e)}")
            return False

    def post_tick_handler(self, snapshot):
        """
        Handle post-tick events for monitoring
        """
        # Log tree state for debugging
        pass

    def is_tree_complete(self, tree_manager):
        """
        Check if tree execution is complete
        """
        return (tree_manager.root.status == py_trees.common.Status.SUCCESS or
                tree_manager.root.status == py_trees.common.Status.FAILURE)

    def should_cancel_execution(self, plan_id):
        """
        Check if current execution should be cancelled
        """
        # Implementation depends on your cancellation mechanism
        return False

    def publish_status(self, status_message):
        """
        Publish execution status
        """
        msg = String()
        msg.data = status_message
        self.status_publisher.publish(msg)

    def generate_plan_id(self):
        """
        Generate unique plan ID
        """
        import uuid
        return str(uuid.uuid4())
```

## Error Handling and Recovery

Implement robust error handling for plan execution:

```python
class RobustPlanExecutor:
    def __init__(self, node):
        self.node = node
        self.max_retries_per_step = 3
        self.global_timeout = 300  # 5 minutes for entire plan

    def execute_plan_with_recovery(self, llm_plan):
        """
        Execute plan with built-in recovery mechanisms
        """
        start_time = self.node.get_clock().now().nanoseconds / 1e9

        for i, step in enumerate(llm_plan):
            retry_count = 0

            while retry_count < self.max_retries_per_step:
                try:
                    # Check global timeout
                    current_time = self.node.get_clock().now().nanoseconds / 1e9
                    if current_time - start_time > self.global_timeout:
                        raise TimeoutError("Global plan timeout exceeded")

                    # Execute the step
                    success = self.execute_single_step(step)

                    if success:
                        self.node.get_logger().info(f'Step {i+1} completed successfully')
                        break  # Move to next step
                    else:
                        retry_count += 1
                        if retry_count < self.max_retries_per_step:
                            self.node.get_logger().warn(f'Step {i+1} failed, retrying ({retry_count}/{self.max_retries_per_step})')
                            # Add delay before retry
                            time.sleep(1.0)
                        else:
                            self.node.get_logger().error(f'Step {i+1} failed after {self.max_retries_per_step} attempts')
                            return False

                except Exception as e:
                    retry_count += 1
                    if retry_count < self.max_retries_per_step:
                        self.node.get_logger().warn(f'Step {i+1} exception: {e}, retrying')
                    else:
                        self.node.get_logger().error(f'Step {i+1} failed with exception after retries: {e}')
                        return False

        return True  # All steps completed successfully

    def execute_single_step(self, step):
        """
        Execute a single plan step with error handling
        """
        try:
            action_type = step.get('type', 'unknown')

            if action_type == 'navigate':
                return self.execute_navigation_step(step)
            elif action_type == 'manipulate':
                return self.execute_manipulation_step(step)
            elif action_type == 'perceive':
                return self.execute_perception_step(step)
            else:
                self.node.get_logger().warn(f'Unknown action type: {action_type}')
                return True  # Skip unknown actions

        except Exception as e:
            self.node.get_logger().error(f'Error executing step: {e}')
            return False

    def execute_navigation_step(self, step):
        """
        Execute navigation step with obstacle detection
        """
        # Implementation would include obstacle detection and replanning
        pass

    def execute_manipulation_step(self, step):
        """
        Execute manipulation step with grasp verification
        """
        # Implementation would include grasp verification and retry logic
        pass

    def execute_perception_step(self, step):
        """
        Execute perception step with result validation
        """
        # Implementation would include result validation
        pass
```

## Integration with LLM Feedback Loop

Provide execution feedback to the LLM system for improved planning:

```python
class FeedbackAwarePlanner:
    def __init__(self, node):
        self.node = node

        # Publisher for execution feedback to LLM
        self.feedback_publisher = node.create_publisher(String, 'execution_feedback', 10)

        # Subscriber for new tasks from LLM
        self.task_subscriber = node.create_subscription(
            String,
            'llm_tasks',
            self.task_callback,
            10
        )

    def task_callback(self, msg):
        """
        Handle new LLM task with execution context
        """
        try:
            task_data = json.loads(msg.data)

            # Include execution history in context
            execution_context = self.get_execution_context()
            task_data['context'] = execution_context

            # Execute the plan
            success = self.execute_plan_with_context(task_data)

            # Send feedback to LLM
            feedback = {
                'task_id': task_data.get('id'),
                'success': success,
                'execution_log': self.get_execution_log(task_data.get('id')),
                'environment_state': self.get_current_environment_state()
            }

            feedback_msg = String()
            feedback_msg.data = json.dumps(feedback)
            self.feedback_publisher.publish(feedback_msg)

        except Exception as e:
            self.node.get_logger().error(f'Error in task callback: {e}')

    def get_execution_context(self):
        """
        Get execution context for LLM planning
        """
        return {
            'recent_failures': self.get_recent_failures(),
            'successful_patterns': self.get_successful_patterns(),
            'environment_changes': self.get_environment_changes(),
            'robot_state': self.get_robot_state()
        }

    def get_recent_failures(self):
        """
        Get information about recent plan failures
        """
        # Implementation would track recent failures
        return []

    def get_successful_patterns(self):
        """
        Get patterns of successful plan executions
        """
        # Implementation would track successful patterns
        return []
```

## Performance Optimization

### 1. Asynchronous Execution
```python
import asyncio
from concurrent.futures import ThreadPoolExecutor

class AsyncPlanExecutor:
    def __init__(self, node):
        self.node = node
        self.executor = ThreadPoolExecutor(max_workers=4)

    async def execute_plan_async(self, llm_plan):
        """
        Execute plan asynchronously
        """
        tasks = []
        for step in llm_plan:
            task = asyncio.get_event_loop().run_in_executor(
                self.executor,
                self.execute_single_step,
                step
            )
            tasks.append(task)

        results = await asyncio.gather(*tasks, return_exceptions=True)
        return all(result is True for result in results if not isinstance(result, Exception))
```

### 2. Caching and Pre-computation
```python
from functools import lru_cache
import pickle

class CachedPlanMapper:
    def __init__(self):
        self.cache = {}
        self.max_cache_size = 100

    @lru_cache(maxsize=100)
    def map_common_plans(self, plan_description_hash):
        """
        Cache mappings for common plan types
        """
        # Implementation would map common plans
        pass
```

This ROS 2 mapping system provides the critical connection between high-level LLM-generated plans and low-level robotic execution, enabling sophisticated autonomous behavior in robotic systems.
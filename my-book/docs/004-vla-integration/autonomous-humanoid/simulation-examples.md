---
sidebar_position: 500
title: Simulation-Based Examples for Students
---

# Simulation-Based Examples for Students

This section provides practical simulation-based examples that students can reproduce to understand and experiment with Vision-Language-Action (VLA) systems. These examples demonstrate how to set up and run VLA systems in simulation environments like Gazebo, Isaac Sim, or other robotics simulators.

## Setting Up the Simulation Environment

### 1. Installing and Configuring Gazebo

```bash
# Install Gazebo Garden (or preferred version)
sudo apt-get update
sudo apt-get install gazebo

# Install ROS 2 Gazebo plugins
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-gazebo-ros2-control
```

### 2. Creating a VLA Simulation World

Create a simulation world file (`vla_world.sdf`) for your experiments:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="vla_experiment_world">
    <!-- Include common models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Kitchen environment -->
    <model name="kitchen_table">
      <pose>5 3 0 0 0 0</pose>
      <link name="table_link">
        <visual name="table_visual">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
          <material>
            <diffuse>0.8 0.6 0.4</diffuse>
          </material>
        </visual>
        <collision name="table_collision">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Red cup on the table -->
    <model name="red_cup">
      <pose>5.1 3.1 1.2 0 0 0</pose>
      <link name="cup_link">
        <visual name="cup_visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
          <material>
            <diffuse>1 0 0</diffuse> <!-- Red -->
          </material>
        </visual>
        <collision name="cup_collision">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Robot spawn location -->
    <state world_name="vla_experiment_world">
      <model name="humanoid_robot">
        <pose>0 0 0.5 0 0 0</pose>
      </model>
    </state>
  </world>
</sdf>
```

### 3. Configuring Robot for VLA Tasks

Create a robot configuration file (`humanoid_vla.config`):

```yaml
# VLA-enabled Humanoid Robot Configuration
robot_description: &robot_description
  # Robot URDF/XACRO description
  # This should include:
  # - Vision sensors (cameras, depth sensors)
  # - Manipulator arms
  # - Mobile base
  # - Actuators for all degrees of freedom

controllers:
  # Navigation controller
  navigation_controller:
    type: "dwb_core::DWBLocalPlanner"
    cmd_vel_topic: "cmd_vel"
    velocity_filters: ["speed_limit_filter"]

  # Arm controllers
  left_arm_controller:
    type: "position_controllers/JointTrajectoryController"
    joints: ["left_shoulder_pan", "left_shoulder_lift", "left_elbow", "left_wrist"]

  right_arm_controller:
    type: "position_controllers/JointTrajectoryController"
    joints: ["right_shoulder_pan", "right_shoulder_lift", "right_elbow", "right_wrist"]

  # Gripper controllers
  left_gripper_controller:
    type: "position_controllers/JointGroupPositionController"
    joints: ["left_gripper_finger1", "left_gripper_finger2"]

  right_gripper_controller:
    type: "position_controllers/JointGroupPositionController"
    joints: ["right_gripper_finger1", "right_gripper_finger2"]

# Sensor configurations
sensors:
  # RGB-D camera for perception
  rgb_camera:
    topic: "/camera/rgb/image_raw"
    frame_id: "camera_link"
    update_rate: 30.0

  # Depth sensor
  depth_camera:
    topic: "/camera/depth/image_raw"
    frame_id: "camera_link"
    update_rate: 30.0

  # Laser scanner for navigation
  laser_scan:
    topic: "/scan"
    frame_id: "laser_link"
    update_rate: 10.0
```

## Complete Simulation Example: Fetch and Carry Task

### 1. Launch File Setup

Create a launch file (`vla_fetch_demo.launch.py`):

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='vla_experiment_world',
        description='Choose one of the world files from `gazebo_worlds`'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('your_robot_gazebo'),
                'worlds',
                LaunchConfiguration('world') + '.sdf'
            ])
        }.items()
    )

    # Robot spawn node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # VLA demo node
    vla_demo = Node(
        package='vla_demo_package',
        executable='complete_vla_demo',
        name='complete_vla_demo',
        output='screen'
    )

    # Navigation stack
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ])
    )

    # Perception pipeline
    perception = Node(
        package='perception_package',
        executable='perception_pipeline',
        name='perception_pipeline',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        spawn_entity,
        vla_demo,
        navigation,
        perception
    ])
```

### 2. Simulation Test Script

Create a test script (`test_vla_simulation.py`) that students can run:

```python
#!/usr/bin/env python3
"""
VLA Simulation Test Script
This script demonstrates how to interact with the VLA simulation system
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json
import time


class VLASimulationTester(Node):
    def __init__(self):
        super().__init__('vla_simulation_tester')

        # Publishers
        self.task_publisher = self.create_publisher(
            String,
            'start_vla_demo',
            10
        )

        self.status_subscriber = self.create_subscription(
            String,
            'vla_demo_status',
            self.status_callback,
            10
        )

        self.task_result_subscriber = self.create_subscription(
            String,
            'task_result',
            self.result_callback,
            10
        )

        self.test_results = []
        self.current_test = None

        self.get_logger().info('VLA Simulation Tester initialized')

    def status_callback(self, msg):
        """
        Handle status updates from the VLA demo
        """
        self.get_logger().info(f'Status: {msg.data}')

    def result_callback(self, msg):
        """
        Handle task results
        """
        try:
            result = json.loads(msg.data)

            if self.current_test:
                self.test_results.append({
                    'test': self.current_test,
                    'result': result,
                    'timestamp': self.get_clock().now().nanoseconds
                })

            self.get_logger().info(f'Task completed: {result}')

        except Exception as e:
            self.get_logger().error(f'Error processing result: {e}')

    def run_basic_fetch_test(self):
        """
        Run basic fetch and carry test
        """
        self.get_logger().info('Starting basic fetch and carry test...')

        self.current_test = 'basic_fetch_test'

        # Create test configuration
        test_config = {
            'task': 'Bring me the red cup from the kitchen table',
            'expected_objects': ['red cup'],
            'expected_location': 'kitchen',
            'timeout': 120  # 2 minutes timeout
        }

        # Publish test configuration
        config_msg = String()
        config_msg.data = json.dumps(test_config)
        self.task_publisher.publish(config_msg)

        # Wait for completion or timeout
        start_time = time.time()
        while time.time() - start_time < test_config['timeout']:
            rclpy.spin_once(self, timeout_sec=1.0)

            # Check if test is complete
            if self.test_results and self.test_results[-1]['test'] == self.current_test:
                break

        # Analyze results
        if self.test_results:
            last_result = self.test_results[-1]['result']
            if last_result.get('success', False):
                self.get_logger().info('✅ Basic fetch test PASSED!')
                return True
            else:
                self.get_logger().error(f'❌ Basic fetch test FAILED: {last_result.get("error")}')
                return False
        else:
            self.get_logger().error('❌ Basic fetch test TIMEOUT')
            return False

    def run_navigation_test(self):
        """
        Run navigation-specific test
        """
        self.get_logger().info('Starting navigation test...')

        self.current_test = 'navigation_test'

        test_config = {
            'task': 'Go to the kitchen table and wait there',
            'expected_action': 'navigation',
            'expected_location': 'kitchen',
            'timeout': 60
        }

        config_msg = String()
        config_msg.data = json.dumps(test_config)
        self.task_publisher.publish(config_msg)

        # Wait for completion
        start_time = time.time()
        while time.time() - start_time < test_config['timeout']:
            rclpy.spin_once(self, timeout_sec=1.0)

            if self.test_results and self.test_results[-1]['test'] == self.current_test:
                break

        if self.test_results:
            last_result = self.test_results[-1]['result']
            success = last_result.get('success', False)
            self.get_logger().info(f'Navigation test: {"PASSED" if success else "FAILED"}')
            return success

        return False

    def run_perception_test(self):
        """
        Run perception-specific test
        """
        self.get_logger().info('Starting perception test...')

        self.current_test = 'perception_test'

        test_config = {
            'task': 'Look around and identify all objects in the kitchen',
            'expected_action': 'perception',
            'expected_location': 'kitchen',
            'timeout': 60
        }

        config_msg = String()
        config_msg.data = json.dumps(test_config)
        self.task_publisher.publish(config_msg)

        # Wait for completion
        start_time = time.time()
        while time.time() - start_time < test_config['timeout']:
            rclpy.spin_once(self, timeout_sec=1.0)

            if self.test_results and self.test_results[-1]['test'] == self.current_test:
                break

        if self.test_results:
            last_result = self.test_results[-1]['result']
            success = last_result.get('success', False)
            self.get_logger().info(f'Perception test: {"PASSED" if success else "FAILED"}')
            return success

        return False

    def run_comprehensive_test_suite(self):
        """
        Run comprehensive test suite
        """
        self.get_logger().info('Starting comprehensive VLA test suite...')

        tests = [
            ('Basic Fetch Test', self.run_basic_fetch_test),
            ('Navigation Test', self.run_navigation_test),
            ('Perception Test', self.run_perception_test)
        ]

        results = {}

        for test_name, test_func in tests:
            self.get_logger().info(f'Running {test_name}...')
            result = test_func()
            results[test_name] = result

            # Pause between tests
            time.sleep(2.0)

        # Print summary
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('TEST SUITE RESULTS:')
        self.get_logger().info('='*50)

        passed = 0
        total = len(results)

        for test_name, result in results.items():
            status = "✅ PASSED" if result else "❌ FAILED"
            self.get_logger().info(f'{test_name}: {status}')
            if result:
                passed += 1

        self.get_logger().info('-'*50)
        self.get_logger().info(f'TOTAL: {passed}/{total} tests passed')
        self.get_logger().info('='*50)

        return passed == total


def main(args=None):
    rclpy.init(args=args)
    tester = VLASimulationTester()

    try:
        success = tester.run_comprehensive_test_suite()

        if success:
            tester.get_logger().info('🎉 All tests passed! VLA system working correctly.')
        else:
            tester.get_logger().info('⚠️ Some tests failed. Check the output above for details.')

    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Student Exercises

### Exercise 1: Simple Navigation Task

**Objective**: Navigate to a specified location in the simulation environment.

**Steps**:
1. Launch the simulation environment:
   ```bash
   ros2 launch your_package vla_fetch_demo.launch.py
   ```

2. Send a navigation command:
   ```bash
   ros2 topic pub /start_vla_demo std_msgs/String "data: '{\"task\": \"Go to the kitchen table\"}'"
   ```

3. Observe the robot's navigation behavior and check for successful arrival.

### Exercise 2: Object Perception Task

**Objective**: Detect and identify objects in the environment.

**Steps**:
1. Launch the simulation with perception system active
2. Send a perception command:
   ```bash
   ros2 topic pub /start_vla_demo std_msgs/String "data: '{\"task\": \"Find the red cup in the kitchen\"}'"
   ```
3. Verify that the object is detected and its properties are correctly identified.

### Exercise 3: Manipulation Task

**Objective**: Grasp and manipulate an object.

**Steps**:
1. Ensure the robot is positioned near the target object
2. Send a manipulation command:
   ```bash
   ros2 topic pub /start_vla_demo std_msgs/String "data: '{\"task\": \"Pick up the red cup from the table\"}'"
   ```
3. Observe the grasp execution and verify success.

### Exercise 4: Complete VLA Task

**Objective**: Execute a complete task involving navigation, perception, and manipulation.

**Steps**:
1. Set up the complete VLA pipeline
2. Send a complex command:
   ```bash
   ros2 topic pub /start_vla_demo std_msgs/String "data: '{\"task\": \"Go to the kitchen, find the red cup on the table, pick it up, and bring it to me\"}'"
   ```
3. Monitor the complete execution flow and analyze the results.

## Performance Testing

### 1. Benchmarking Script

Create a benchmarking script (`benchmark_vla.py`):

```python
#!/usr/bin/env python3
"""
VLA Performance Benchmarking
Tests the performance of VLA components in simulation
"""

import rclpy
from rclpy.node import Node
import time
import statistics
from std_msgs.msg import String
import json


class VLAPerformanceBenchmark(Node):
    def __init__(self):
        super().__init__('vla_performance_benchmark')

        self.task_publisher = self.create_publisher(String, 'start_vla_demo', 10)
        self.results_subscriber = self.create_subscription(
            String, 'task_result', self.results_callback, 10
        )

        self.benchmark_results = []
        self.current_task_start_time = None

    def results_callback(self, msg):
        """
        Record task completion time
        """
        if self.current_task_start_time:
            completion_time = time.time() - self.current_task_start_time
            self.benchmark_results.append(completion_time)
            self.current_task_start_time = None

    def benchmark_task_execution(self, task, num_iterations=10):
        """
        Benchmark task execution time
        """
        self.get_logger().info(f'Benchmarking task: {task} ({num_iterations} iterations)')

        for i in range(num_iterations):
            # Prepare task
            task_config = {
                'task': task,
                'iteration': i + 1
            }

            # Publish task
            task_msg = String()
            task_msg.data = json.dumps(task_config)

            self.current_task_start_time = time.time()
            self.task_publisher.publish(task_msg)

            # Wait for completion (with timeout)
            timeout = 30.0  # 30 second timeout per task
            start_wait = time.time()

            while (time.time() - start_wait) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.current_task_start_time is None:  # Task completed
                    break

            if self.current_task_start_time is not None:
                # Task timed out
                self.get_logger().warn(f'Task {i+1} timed out')
                self.current_task_start_time = None
                self.benchmark_results.append(float('inf'))

            # Small delay between iterations
            time.sleep(1.0)

        # Calculate statistics
        valid_times = [t for t in self.benchmark_results if t != float('inf')]

        if valid_times:
            stats = {
                'task': task,
                'iterations': num_iterations,
                'successful_runs': len(valid_times),
                'mean_time': statistics.mean(valid_times),
                'median_time': statistics.median(valid_times),
                'stdev_time': statistics.stdev(valid_times) if len(valid_times) > 1 else 0,
                'min_time': min(valid_times),
                'max_time': max(valid_times),
                'timeout_rate': (num_iterations - len(valid_times)) / num_iterations
            }

            return stats
        else:
            return {
                'task': task,
                'iterations': num_iterations,
                'successful_runs': 0,
                'error': 'All tasks timed out'
            }

    def run_complete_benchmark_suite(self):
        """
        Run complete benchmark suite
        """
        tasks_to_benchmark = [
            'Go to the kitchen table',
            'Find the red cup in the kitchen',
            'Pick up the red cup',
            'Go to the kitchen, find the red cup, and pick it up'
        ]

        all_results = []

        for task in tasks_to_benchmark:
            # Clear previous results
            self.benchmark_results = []

            stats = self.benchmark_task_execution(task, num_iterations=5)
            all_results.append(stats)

            # Print results
            self.print_benchmark_results(stats)

            # Small delay between tasks
            time.sleep(2.0)

        return all_results

    def print_benchmark_results(self, stats):
        """
        Print benchmark results in a formatted way
        """
        self.get_logger().info('')
        self.get_logger().info(f'BENCHMARK RESULTS FOR: {stats["task"]}')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Iterations: {stats["iterations"]}')
        self.get_logger().info(f'Successful runs: {stats["successful_runs"]}')
        self.get_logger().info(f'Timeout rate: {stats["timeout_rate"]:.2%}')

        if stats["successful_runs"] > 0:
            self.get_logger().info(f'Mean execution time: {stats["mean_time"]:.2f}s')
            self.get_logger().info(f'Median execution time: {stats["median_time"]:.2f}s')
            self.get_logger().info(f'Std deviation: {stats["stdev_time"]:.2f}s')
            self.get_logger().info(f'Min time: {stats["min_time"]:.2f}s')
            self.get_logger().info(f'Max time: {stats["max_time"]:.2f}s')

        self.get_logger().info('='*60)
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    benchmark = VLAPerformanceBenchmark()

    try:
        results = benchmark.run_complete_benchmark_suite()

        # Print summary
        benchmark.get_logger().info('\n' + '='*60)
        benchmark.get_logger().info('BENCHMARK SUMMARY')
        benchmark.get_logger().info('='*60)

        for result in results:
            task_name = result['task'][:50] + '...' if len(result['task']) > 50 else result['task']
            if result['successful_runs'] > 0:
                avg_time = result['mean_time']
                success_rate = result['successful_runs'] / result['iterations']
                benchmark.get_logger().info(f'{task_name:<50} | {avg_time:>6.2f}s | {success_rate:>5.1%} success')
            else:
                benchmark.get_logger().info(f'{task_name:<50} | Timeout | 0.0% success')

        benchmark.get_logger().info('='*60)

    except KeyboardInterrupt:
        benchmark.get_logger().info('Benchmark interrupted by user')
    finally:
        benchmark.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Troubleshooting Guide for Students

### Common Issues and Solutions

#### 1. Robot Not Responding
**Problem**: Robot doesn't execute commands
**Solutions**:
- Check if all required nodes are running: `ros2 node list`
- Verify topic connections: `ros2 topic info /start_vla_demo`
- Check robot state: `ros2 service call /robot_state std_srvs/Trigger`

#### 2. Object Detection Failures
**Problem**: Robot can't find specified objects
**Solutions**:
- Verify object exists in simulation environment
- Check camera calibration and positioning
- Adjust perception parameters in config file

#### 3. Navigation Failures
**Problem**: Robot fails to navigate to target location
**Solutions**:
- Check map server and localization nodes
- Verify costmap parameters
- Inspect obstacle detection

#### 4. Manipulation Failures
**Problem**: Robot fails to grasp or manipulate objects
**Solutions**:
- Check gripper calibration
- Verify approach angles and distances
- Adjust grasp planning parameters

## Advanced Exercises

### Exercise 5: Custom Task Creation
Students can create their own tasks by modifying the LLM prompt and testing different scenarios.

### Exercise 6: Multi-Object Tasks
Design tasks that require the robot to interact with multiple objects sequentially.

### Exercise 7: Error Recovery Testing
Introduce artificial errors and test the system's recovery capabilities.

These simulation-based examples provide hands-on experience with VLA systems in a controlled, safe environment where students can experiment, learn, and develop their understanding of autonomous humanoid robotics.
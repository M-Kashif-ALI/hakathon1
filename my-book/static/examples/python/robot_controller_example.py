"""
Example of a complete robot controller node using rclpy.

This demonstrates how to create a node that subscribes to sensor data,
processes it, and publishes control commands - a typical pattern in
humanoid robot control systems.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math


class RobotController(Node):
    """
    A robot controller that demonstrates the integration of
    sensor subscription and command publishing patterns.
    """

    def __init__(self):
        super().__init__('robot_controller')

        # Publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        # Subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50Hz

        # Store current state
        self.current_positions = []
        self.current_velocities = []
        self.command_counter = 0

        self.get_logger().info('Robot controller initialized')

    def joint_state_callback(self, msg):
        """Callback function to handle incoming joint state messages."""
        self.current_positions = list(msg.position)
        self.current_velocities = list(msg.velocity)

        if len(self.current_positions) > 0:
            self.get_logger().debug(
                f'Received joint states: {len(self.current_positions)} joints'
            )

    def control_loop(self):
        """Main control loop executed at regular intervals."""
        # Example: Generate a simple oscillating pattern for the joints
        target_positions = []
        for i in range(max(1, len(self.current_positions))):
            # Create a different oscillation for each joint
            angle = self.command_counter * 0.1 + i * 0.5
            pos = math.sin(angle) * 0.5  # Limit to ±0.5 radians
            target_positions.append(pos)

        # Create and publish command message
        cmd_msg = Float64MultiArray()
        cmd_msg.data = target_positions
        self.joint_cmd_publisher.publish(cmd_msg)

        self.command_counter += 1

        if self.command_counter % 100 == 0:  # Log every 100 iterations
            self.get_logger().info(f'Published command with {len(target_positions)} joint positions')


def main(args=None):
    """Main function to run the robot controller node."""
    rclpy.init(args=args)

    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
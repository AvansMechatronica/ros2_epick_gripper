#!/usr/bin/env python3
"""Console script for controlling the Epick gripper via ROS2 action interface."""

import argparse
import sys
from threading import Event

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand


class GripperController(Node):
    """ROS2 node for controlling the Epick gripper."""

    def __init__(self, action_name='/epick_gripper_action_controller/gripper_cmd'):
        """
        Initialize the gripper controller.

        Args:
            action_name: The name of the gripper action server
        """
        super().__init__('epick_gripper_control_node')
        self._action_client = ActionClient(self, GripperCommand, action_name)
        self._goal_done_event = Event()
        self._goal_result = None

    def send_goal(self, position, max_effort=0.0, wait=True, timeout=10.0):
        """
        Send a gripper command goal.

        Args:
            position: Target position (1.0 = grip, 0.0 = release)
            max_effort: Maximum effort (not used by Epick, kept for compatibility)
            wait: Whether to wait for the goal to complete
            timeout: Timeout in seconds when waiting

        Returns:
            True if goal succeeded, False otherwise (or None if not waiting)
        """
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info(f'Sending goal: position={position}')

        self._goal_done_event.clear()
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

        if not wait:
            return None

        # Wait for goal to complete
        if self._goal_done_event.wait(timeout=timeout):
            return self._goal_result.result.reached_goal if self._goal_result else False
        else:
            self.get_logger().error('Goal timed out!')
            return False

    def grip(self, wait=True, timeout=10.0):
        """
        Command the gripper to grip an object.

        Args:
            wait: Whether to wait for the goal to complete
            timeout: Timeout in seconds when waiting

        Returns:
            True if grip succeeded, False otherwise
        """
        return self.send_goal(1.0, wait=wait, timeout=timeout)

    def release(self, wait=True, timeout=10.0):
        """
        Command the gripper to release an object.

        Args:
            wait: Whether to wait for the goal to complete
            timeout: Timeout in seconds when waiting

        Returns:
            True if release succeeded, False otherwise
        """
        return self.send_goal(0.0, wait=wait, timeout=timeout)

    def _feedback_callback(self, feedback_msg):
        """Callback for action feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: position={feedback.position:.2f}, '
            f'effort={feedback.effort:.2f}, '
            f'stalled={feedback.stalled}, '
            f'reached_goal={feedback.reached_goal}'
        )

    def _goal_response_callback(self, future):
        """Callback when goal is accepted/rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self._goal_done_event.set()
            return

        self.get_logger().info('Goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        """Callback when goal completes."""
        self._goal_result = future.result()
        result = self._goal_result.result

        status_map = {
            1: 'SUCCEEDED',
            2: 'CANCELED',
            3: 'ABORTED'
        }
        status = status_map.get(self._goal_result.status, 'UNKNOWN')

        self.get_logger().info(
            f'Goal completed with status: {status}\n'
            f'  Position: {result.position:.2f}\n'
            f'  Effort: {result.effort:.2f}\n'
            f'  Reached goal: {result.reached_goal}\n'
            f'  Stalled: {result.stalled}'
        )
        self._goal_done_event.set()


def main(args=None):
    """Main entry point for the gripper control script."""
    parser = argparse.ArgumentParser(
        description='Control the Epick gripper via ROS2 action interface',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s grip                    # Grip an object
  %(prog)s release                 # Release an object
  %(prog)s position 1.0            # Send custom position (1.0=grip, 0.0=release)
  %(prog)s grip --no-wait          # Send grip command without waiting
  %(prog)s grip --timeout 5.0      # Wait max 5 seconds for completion
        """
    )

    parser.add_argument(
        'command',
        choices=['grip', 'release', 'position'],
        help='Command to send to the gripper'
    )
    parser.add_argument(
        'value',
        type=float,
        nargs='?',
        help='Position value (required for "position" command, 0.0-1.0)'
    )
    parser.add_argument(
        '--action-name',
        default='/epick_gripper_action_controller/gripper_cmd',
        help='Action server name (default: %(default)s)'
    )
    parser.add_argument(
        '--no-wait',
        action='store_true',
        help='Do not wait for goal to complete'
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=10.0,
        help='Timeout in seconds when waiting for goal (default: %(default)s)'
    )

    parsed_args = parser.parse_args()

    # Validate position command
    if parsed_args.command == 'position':
        if parsed_args.value is None:
            parser.error('position command requires a value argument')
        if not 0.0 <= parsed_args.value <= 1.0:
            parser.error('position value must be between 0.0 and 1.0')

    # Initialize ROS2
    rclpy.init(args=args)

    controller = GripperController(action_name=parsed_args.action_name)

    try:
        # Execute command
        wait = not parsed_args.no_wait
        result = None

        if parsed_args.command == 'grip':
            result = controller.grip(wait=wait, timeout=parsed_args.timeout)
        elif parsed_args.command == 'release':
            result = controller.release(wait=wait, timeout=parsed_args.timeout)
        elif parsed_args.command == 'position':
            result = controller.send_goal(
                parsed_args.value,
                wait=wait,
                timeout=parsed_args.timeout
            )

        # Spin to process callbacks
        if wait:
            rclpy.spin_once(controller, timeout_sec=0.1)

        # Exit with appropriate code
        if result is None:
            sys.exit(0)  # No wait mode
        elif result:
            print('✓ Command completed successfully')
            sys.exit(0)
        else:
            print('✗ Command failed')
            sys.exit(1)

    except KeyboardInterrupt:
        print('\nInterrupted by user')
        sys.exit(130)
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

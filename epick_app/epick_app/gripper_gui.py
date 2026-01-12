#!/usr/bin/env python3
"""GUI application for controlling the Epick gripper via ROS2 action interface."""

import argparse
import sys
import threading
from tkinter import Tk, Frame, Button, Label, Scale, StringVar, HORIZONTAL, DISABLED, NORMAL, W, E

import rclpy
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node


class GripperGUIController(Node):
    """ROS2 node for controlling the Epick gripper with GUI callbacks."""

    def __init__(self, action_name='/epick_gripper_action_controller/gripper_cmd'):
        """
        Initialize the gripper controller.

        Args:
            action_name: The name of the gripper action server
        """
        super().__init__('epick_gripper_gui_node')
        self._action_client = ActionClient(self, GripperCommand, action_name)
        self._current_goal_handle = None
        self._status_callback = None
        self._feedback_callback = None
        
        # Check server availability
        self.get_logger().info(f'Waiting for action server: {action_name}')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Action server not available yet!')

    def set_status_callback(self, callback):
        """Set callback for status updates."""
        self._status_callback = callback

    def set_feedback_callback(self, callback):
        """Set callback for action feedback."""
        self._feedback_callback = callback

    def send_goal(self, position, max_effort=0.0):
        """
        Send a gripper command goal.

        Args:
            position: Target position (1.0 = grip, 0.0 = release)
            max_effort: Maximum effort (not used by Epick, kept for compatibility)
        """
        if not self._action_client.server_is_ready():
            self._update_status('Error: Action server not available!', 'error')
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info(f'Sending goal: position={position}')
        self._update_status(f'Sending goal: position={position:.2f}...', 'working')

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._handle_feedback
        )
        send_goal_future.add_done_callback(self._handle_goal_response)

    def cancel_goal(self):
        """Cancel the current goal if one is active."""
        if self._current_goal_handle is not None:
            self.get_logger().info('Canceling current goal')
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._handle_cancel_response)

    def _handle_feedback(self, feedback_msg):
        """Callback for action feedback."""
        feedback = feedback_msg.feedback
        status_text = (
            f'Position: {feedback.position:.2f} | '
            f'Effort: {feedback.effort:.2f} | '
            f'Stalled: {feedback.stalled} | '
            f'Goal: {feedback.reached_goal}'
        )
        self._update_status(status_text, 'working')
        
        if self._feedback_callback:
            self._feedback_callback(feedback)

    def _handle_goal_response(self, future):
        """Callback when goal is accepted/rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self._update_status('Goal rejected by server!', 'error')
            self._current_goal_handle = None
            return

        self.get_logger().info('Goal accepted!')
        self._current_goal_handle = goal_handle
        self._update_status('Goal accepted, executing...', 'working')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_result)

    def _handle_result(self, future):
        """Callback when goal completes."""
        result = future.result()
        self._current_goal_handle = None

        status_map = {
            1: 'SUCCEEDED',
            2: 'CANCELED',
            3: 'ABORTED'
        }
        status = status_map.get(result.status, 'UNKNOWN')

        result_msg = (
            f'Status: {status} | '
            f'Position: {result.result.position:.2f} | '
            f'Reached: {result.result.reached_goal} | '
            f'Stalled: {result.result.stalled}'
        )
        
        state = 'success' if result.status == 1 else 'error'
        self._update_status(result_msg, state)
        self.get_logger().info(result_msg)

    def _handle_cancel_response(self, future):
        """Callback when cancel request completes."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self._update_status('Goal canceled', 'idle')
            self.get_logger().info('Goal successfully canceled')
        else:
            self._update_status('Failed to cancel goal', 'error')
            self.get_logger().warn('Failed to cancel goal')

    def _update_status(self, message, state='idle'):
        """Update status via callback."""
        if self._status_callback:
            self._status_callback(message, state)


class GripperGUI:
    """Main GUI class for the gripper controller."""

    def __init__(self, controller):
        """
        Initialize the GUI.

        Args:
            controller: GripperGUIController instance
        """
        self.controller = controller
        self.controller.set_status_callback(self.update_status)
        
        # Create main window
        self.root = Tk()
        self.root.title('Epick Gripper Control')
        self.root.geometry('500x400')
        self.root.resizable(True, True)
        
        # Status colors
        self.colors = {
            'idle': '#333333',
            'working': '#FF8C00',
            'success': '#008000',
            'error': '#FF0000'
        }
        
        self._create_widgets()
        
    def _create_widgets(self):
        """Create all GUI widgets."""
        # Title
        title_frame = Frame(self.root)
        title_frame.pack(pady=10, padx=10, fill='x')
        
        title_label = Label(
            title_frame,
            text='Epick Gripper Controller',
            font=('Arial', 16, 'bold')
        )
        title_label.pack()
        
        # Quick action buttons
        action_frame = Frame(self.root)
        action_frame.pack(pady=20, padx=10, fill='x')
        
        self.grip_button = Button(
            action_frame,
            text='GRIP',
            command=self.grip,
            bg='#4CAF50',
            fg='white',
            font=('Arial', 14, 'bold'),
            height=2,
            cursor='hand2'
        )
        self.grip_button.pack(side='left', expand=True, fill='both', padx=5)
        
        self.release_button = Button(
            action_frame,
            text='RELEASE',
            command=self.release,
            bg='#f44336',
            fg='white',
            font=('Arial', 14, 'bold'),
            height=2,
            cursor='hand2'
        )
        self.release_button.pack(side='left', expand=True, fill='both', padx=5)
        
        # Position slider
        slider_frame = Frame(self.root)
        slider_frame.pack(pady=20, padx=10, fill='x')
        
        slider_label = Label(
            slider_frame,
            text='Custom Position',
            font=('Arial', 12)
        )
        slider_label.pack()
        
        self.position_var = Scale(
            slider_frame,
            from_=0.0,
            to=1.0,
            resolution=0.01,
            orient=HORIZONTAL,
            length=400,
            label='Position (0.0 = Release, 1.0 = Grip)'
        )
        self.position_var.set(0.5)
        self.position_var.pack(pady=5)
        
        send_button = Button(
            slider_frame,
            text='Send Custom Position',
            command=self.send_custom_position,
            bg='#2196F3',
            fg='white',
            font=('Arial', 11, 'bold'),
            cursor='hand2'
        )
        send_button.pack(pady=5)
        
        # Cancel button
        cancel_frame = Frame(self.root)
        cancel_frame.pack(pady=10, padx=10, fill='x')
        
        self.cancel_button = Button(
            cancel_frame,
            text='Cancel Current Goal',
            command=self.cancel,
            bg='#FF9800',
            fg='white',
            font=('Arial', 10),
            cursor='hand2',
            state=DISABLED
        )
        self.cancel_button.pack()
        
        # Status display
        status_frame = Frame(self.root, relief='sunken', borderwidth=2)
        status_frame.pack(pady=10, padx=10, fill='both', expand=True)
        
        status_title = Label(
            status_frame,
            text='Status',
            font=('Arial', 11, 'bold')
        )
        status_title.pack(anchor=W, padx=5, pady=5)
        
        self.status_text = StringVar()
        self.status_text.set('Ready')
        
        self.status_label = Label(
            status_frame,
            textvariable=self.status_text,
            font=('Arial', 10),
            fg=self.colors['idle'],
            anchor=W,
            justify='left',
            wraplength=450
        )
        self.status_label.pack(anchor=W, padx=5, pady=5, fill='both', expand=True)
        
    def grip(self):
        """Send grip command."""
        self.controller.send_goal(1.0)
        self.cancel_button.config(state=NORMAL)
        
    def release(self):
        """Send release command."""
        self.controller.send_goal(0.0)
        self.cancel_button.config(state=NORMAL)
        
    def send_custom_position(self):
        """Send custom position from slider."""
        position = self.position_var.get()
        self.controller.send_goal(position)
        self.cancel_button.config(state=NORMAL)
        
    def cancel(self):
        """Cancel current goal."""
        self.controller.cancel_goal()
        
    def update_status(self, message, state='idle'):
        """
        Update status display.
        
        Args:
            message: Status message to display
            state: State type ('idle', 'working', 'success', 'error')
        """
        self.status_text.set(message)
        self.status_label.config(fg=self.colors.get(state, self.colors['idle']))
        
        # Disable cancel button if not working
        if state in ['idle', 'success', 'error']:
            self.cancel_button.config(state=DISABLED)
        
    def run(self):
        """Start the GUI main loop."""
        self.root.mainloop()
        
    def destroy(self):
        """Destroy the GUI window."""
        self.root.quit()
        self.root.destroy()


def spin_ros(executor):
    """Spin ROS executor in separate thread."""
    try:
        executor.spin()
    except Exception as e:
        print(f'ROS executor error: {e}')


def main(args=None):
    """Main entry point for the gripper GUI."""
    parser = argparse.ArgumentParser(
        description='GUI application for controlling the Epick gripper'
    )
    parser.add_argument(
        '--action-name',
        default='/epick_gripper_action_controller/gripper_cmd',
        help='Action server name (default: %(default)s)'
    )
    parsed_args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create controller and GUI
    controller = GripperGUIController(action_name=parsed_args.action_name)
    gui = GripperGUI(controller)
    
    # Start ROS spinning in separate thread
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    
    ros_thread = threading.Thread(target=spin_ros, args=(executor,), daemon=True)
    ros_thread.start()
    
    try:
        # Run GUI (blocking)
        gui.run()
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        # Cleanup
        gui.destroy()
        controller.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()

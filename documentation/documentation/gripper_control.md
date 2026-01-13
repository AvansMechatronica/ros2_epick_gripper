# Epick Gripper Control Applications

Python applications for controlling the Epick gripper via ROS2 action interface, including both command-line and GUI tools.

## Installation

Build the package in your ROS2 workspace:

```bash
cd ~/my_ur_ws
colcon build --packages-select epick_app
source install/setup.bash
```

## Usage

The `epick_gripper_control` command provides a simple interface to control the gripper:

### Basic Commands

**Grip an object:**
```bash
epick_gripper_control grip
```

**Release an object:**
```bash
epick_gripper_control release
```

**Send custom position (0.0 = release, 1.0 = grip):**
```bash
epick_gripper_control position 1.0
epick_gripper_control position 0.5
epick_gripper_control position 0.0
```

### Advanced Options

**Send command without waiting for completion:**
```bash
epick_gripper_control grip --no-wait
```

**Set custom timeout (default: 10 seconds):**
```bash
epick_gripper_control grip --timeout 5.0
```

**Use custom action server name:**
```bash
epick_gripper_control grip --action-name /my_gripper/gripper_cmd
```

### Help

```bash
epick_gripper_control --help
```

## Command Line Arguments

- `command`: Required. One of: `grip`, `release`, `position`
- `value`: Required for `position` command. Position value between 0.0 and 1.0
- `--action-name`: Optional. Action server name (default: `/epick_gripper_action_controller/gripper_cmd`)
- `--no-wait`: Optional. Don't wait for goal completion
- `--timeout`: Optional. Timeout in seconds (default: 10.0)

## Exit Codes

- `0`: Success
- `1`: Command failed
- `130`: Interrupted by user (Ctrl+C)

## Examples

```bash
# Simple grip and release
epick_gripper_control grip
epick_gripper_control release

# Quick commands without waiting
epick_gripper_control grip --no-wait
epick_gripper_control release --no-wait

# Custom position with 5 second timeout
epick_gripper_control position 0.7 --timeout 5.0

# Use different action server
epick_gripper_control grip --action-name /custom_gripper/gripper_cmd
```

## Python API

You can also use the `GripperController` class directly in your Python code:

```python
import rclpy
from epick_app.gripper_control import GripperController

rclpy.init()
controller = GripperController()

# Grip
success = controller.grip(wait=True, timeout=10.0)
print(f"Grip successful: {success}")

# Release
success = controller.release(wait=True, timeout=10.0)
print(f"Release successful: {success}")

# Custom position
success = controller.send_goal(0.5, wait=True, timeout=10.0)

controller.destroy_node()
rclpy.shutdown()
```

## GUI Application

A graphical interface for controlling the gripper with buttons and visual feedback.

### Launch GUI

```bash
epick_gripper_gui
```

**With custom action server:**
```bash
epick_gripper_gui --action-name /my_gripper/gripper_cmd
```

### GUI Features

- **Quick Action Buttons**: Large GRIP and RELEASE buttons for immediate control
- **Custom Position Slider**: Fine-grained control with slider (0.0 to 1.0)
- **Real-time Status**: Live feedback showing position, effort, and goal status
- **Cancel Goal**: Stop current action at any time
- **Color-coded Status**: Visual feedback (orange=working, green=success, red=error)

### GUI Screenshot

```
┌─────────────────────────────────────┐
│   Epick Gripper Controller          │
├─────────────────────────────────────┤
│  ┌───────┐        ┌────────┐        │
│  │ GRIP  │        │RELEASE │        │
│  └───────┘        └────────┘        │
│                                     │
│  Custom Position                    │
│  ├──────────────────────┤           │
│  0.0 ═══●══════════ 1.0            │
│     [Send Custom Position]          │
│                                     │
│     [Cancel Current Goal]           │
│                                     │
│  Status                             │
│  ┌─────────────────────────────┐   │
│  │ Position: 1.00 | Reached: ✓ │   │
│  └─────────────────────────────┘   │
└─────────────────────────────────────┘
```

## Requirements

- ROS2 (Humble or later)
- `rclpy`
- `control_msgs`
- `tkinter` (usually included with Python)
- Running `epick_gripper_action_controller` node

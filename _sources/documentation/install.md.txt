# Installatie van de Robotiq package

## Clone repository

```bash
cd ~/my_ur_ws/src
git clone https://github.com/AvansMechatronica/ros2_epick_gripper.git
git clone https://github.com/RoverRobotics-forks/serial-ros2.git serial

```

## Build workspace

```
# Build the workspace
cd ~/my_ur_ws
colcon build --symlink-install
source install/setup.bash
```

# URDF(virtuele gripper)

Voeg een virtuele gripper toe aan je URDF bestand van je project:

```xml
<?xml version="1.0"?>
<robot name="example_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!--
    Inhoud bestaande URDF file
    -->

    <xacro:include filename="$(find epick_description)/urdf/epick_single_suction_cup.xacro"/>

    <xacro:arg name="use_fake_hardware" default="true" />
    <xacro:arg name="com_port" default="/dev/ttyUSB0" />

    <xacro:epick_single_suction_cup parent="<link naar de end-effector>" use_fake_hardware="$(arg use_fake_hardware"  com_port="$(arg com_port)"/>

</robot>
```


# Starten Controller

De grippercontroller kan als volgt gestart worden:
```bash
ros2 launch epick_description epick_gripper.launch.py
```

De controller biedt een ROS 2 service `/grip_cmd` die een boolean request accepteert om de gripper aan te sturen. Een true request start het grijpen, terwijl een false request het loslaten activeert.

Voor monitoring doeleinden biedt de controller ook een `/object_detection_status` topic die een van de volgende object detectie statussen teruggeeft:
- UNKNOWN;
- OBJECT_DETECTED_AT_MIN_PRESSURE;
- OBJECT_DETECTED_AT_MAX_PRESSURE;
- NO_OBJECT_DETECTED







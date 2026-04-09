# ur_rtde_publisher

A ROS 2 package that provides a RTDE (Real Time Data Exchange) publisher for Universal Robots manipulators. The node connects to a robot's IP via UR’s [RTDE protocol](https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html), retrieves selected robot state variables (such as robot mode, safety status and timestamps), and exposes them as ROS 2 topics.

## Architecture (overview)

The package follows the following structure:

- rtde_publisher_node: Implements the ROS2 node. It handles parameters loading, RTDE connections setup, and the main publish loop.
- publisher: Responsible for creating ROS publishers based on the RTDE recipe and the YAML mapping file.
- converter: Contains utility functions to convert RTDE raw data types into ROS messages types.

## Usage

The node is started using the provided launch file:

```bash
ros2 launch ur_rtde_publisher rtde_publisher.launch.xml \
  robot_ip:=192.168.56.101 \
  output_recipe:='["payload", "robot_mode"]' \
  rtde_frequency:=125
```

## Parameters

- robot_ip (string): IP address of the robot.
- output_recipe (string[]): List of RTDE output variables to request and publish.
- rtde_frequency (int): RTDE communication frequency in Hz.

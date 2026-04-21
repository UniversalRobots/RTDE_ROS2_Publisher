# ur_rtde_publisher

A ROS 2 package that provides a RTDE (Real Time Data Exchange) publisher for Universal Robots manipulators. The node connects to a robot's IP via UR’s [RTDE protocol](https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html), retrieves selected robot state variables (such as robot mode, safety status and timestamps), and exposes them as ROS 2 topics.

> [!NOTE]
> This package is is currently in an early development stage. At this point, we don't make any
> guarantees about ROS API stability, so the interface types for published data might change in the
> future. Hence, this package is currently not available as a pre-built binary in any ROS
> distribution, and users need to build it from source.

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

## Build status
<table width="100%">
  <tr>
    <th></th>
    <th>Humble</th>
    <th>Jazzy</th>
    <th>Kilted</th>
    <th>Rolling</th>
  </tr>
  <tr>
    <th>Branch</th>
    <td><a href="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/tree/jazzy">jazzy</a></td>
    <td><a href="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/tree/jazzy">jazzy</a></td>
    <td><a href="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/tree/main">main</a></td>
    <td><a href="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/tree/main">main</a></td>
  </tr>
  <tr>
    <th>Build status main</th>
    <td>
      <a href="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/humble_binary_main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/humble_binary_main.yml/badge.svg?event=schedule"
              alt="Humble Binary Main"/>
      </a> <br />
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/jazzy_binary_main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/jazzy_binary_main.yml/badge.svg?event=schedule"
              alt="Jazzy Binary Main"/>
      </a> <br />
    </td>
    <td> <!-- Kilted -->
      <a href="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/kilted_binary_main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/kilted_binary_main.yml/badge.svg?event=schedule"
              alt="Kilted Binary Main"/>
      </a> <br />
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/rolling_binary_main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/rolling_binary_main.yml/badge.svg?event=schedule"
              alt="Rolling Binary Main"/>
      </a> <br />
    </td>
  </tr>
  <tr>
    <th>Build status testing</th>
    <td>
      <a href="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/humble_binary_testing.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/humble_binary_testing.yml/badge.svg?event=schedule"
              alt="Humble Binary Testing"/>
      </a> <br />
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/jazzy_binary_testing.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/jazzy_binary_testing.yml/badge.svg?event=schedule"
              alt="Jazzy Binary Testing"/>
      </a> <br />
    </td>
    <td> <!-- Kilted -->
      <a href="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/kilted_binary_testing.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/kilted_binary_testing.yml/badge.svg?event=schedule"
              alt="Kilted Binary Testing"/>
      </a> <br />
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/rolling_binary_testing.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/rolling_binary_testing.yml/badge.svg?event=schedule"
              alt="Rolling Binary Testing"/>
      </a> <br />
    </td>
  </tr>
</table>

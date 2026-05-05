ur_rtde_publisher
=================

A ROS 2 package that provides a RTDE (Real Time Data Exchange) publisher for Universal Robots manipulators. The node connects to a robot's IP via UR’s `RTDE protocol <https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html>`_, retrieves selected robot state variables (such as robot mode, safety status and timestamps), and exposes them as ROS 2 topics.

.. note::

   This package is currently in an early development stage. At this point, we don't make any
   guarantees about ROS API stability, so the interface types for published data might change in the
   future. Hence, this package is currently not available as a pre-built binary in any ROS
   distribution, and users need to :ref:`build it from source <ur_rtde_pub/installation>`.

Usage
-----

The node is started using the provided launch file:

.. code-block:: bash

   ros2 launch ur_rtde_publisher rtde_publisher.launch.xml \
     robot_ip:=192.168.56.101 \
     output_recipe:='["payload", "robot_mode"]' \
     rtde_frequency:=125

See the documentation's :ref:`ur_rtde_pub/usage` section for more details on how to run the node and verify its operation.

Parameters
^^^^^^^^^^

* ``robot_ip`` (string): IP address of the robot.
* ``output_recipe`` (string[]): List of RTDE output variables to request and publish.
* ``rtde_frequency`` (int, optional, default: ``500``): RTDE communication frequency in Hz.
* ``tf_prefix`` (string, optional, default: ``""``): Optional prefix applied to the ``frame_id`` of stamped ROS 2 messages.

Architecture
------------

The package follows the following structure:

* **rtde_publisher_node:** Implements the ROS2 node. It handles parameters loading, RTDE connections setup, and the main publish loop.
* **rtde_publisher:** Responsible for creating ROS publishers based on the RTDE recipe and the YAML mapping file.
* **rtde_converter:** Contains utility functions to convert RTDE raw data types into ROS messages types.

Build status
------------

.. list-table::
   :widths: 20 20 20 20 20
   :header-rows: 1

   * - 
     - Humble
     - Jazzy
     - Kilted
     - Rolling
   * - Branch
     - `jazzy <https://github.com/UniversalRobots/RTDE_ROS2_Publisher/tree/jazzy>`_
     - `jazzy <https://github.com/UniversalRobots/RTDE_ROS2_Publisher/tree/jazzy>`_
     - `main <https://github.com/UniversalRobots/RTDE_ROS2_Publisher/tree/main>`_
     - `main <https://github.com/UniversalRobots/RTDE_ROS2_Publisher/tree/main>`_
   * - Build status main
     - .. image:: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/humble_binary_main.yml/badge.svg?event=schedule
          :target: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/humble_binary_main.yml?query=event%3Aschedule++
          :alt: Humble Binary Main
     - .. image:: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/jazzy_binary_main.yml/badge.svg?event=schedule
          :target: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/jazzy_binary_main.yml?query=event%3Aschedule++
          :alt: Jazzy Binary Main
     - .. image:: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/kilted_binary_main.yml/badge.svg?event=schedule
          :target: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/kilted_binary_main.yml?query=event%3Aschedule++
          :alt: Kilted Binary Main
     - .. image:: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/rolling_binary_main.yml/badge.svg?event=schedule
          :target: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/rolling_binary_main.yml?query=event%3Aschedule++
          :alt: Rolling Binary Main
   * - Build status testing
     - .. image:: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/humble_binary_testing.yml/badge.svg?event=schedule
          :target: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/humble_binary_testing.yml?query=event%3Aschedule++
          :alt: Humble Binary Testing
     - .. image:: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/jazzy_binary_testing.yml/badge.svg?event=schedule
          :target: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/jazzy_binary_testing.yml?query=event%3Aschedule++
          :alt: Jazzy Binary Testing
     - .. image:: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/kilted_binary_testing.yml/badge.svg?event=schedule
          :target: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/kilted_binary_testing.yml?query=event%3Aschedule++
          :alt: Kilted Binary Testing
     - .. image:: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/rolling_binary_testing.yml/badge.svg?event=schedule
          :target: https://github.com/UniversalRobots/RTDE_ROS2_Publisher/actions/workflows/rolling_binary_testing.yml?query=event%3Aschedule++
          :alt: Rolling Binary Testing

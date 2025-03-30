SLAM: Mapping
*************

Overview
========
While wandering around, HomeR stitches the laser scans according to its *odometry* to fabricate a map.
Brian Douglas made a remarkable video under the topic of 
`Pose Graph Optimization <https://www.mathworks.com/videos/autonomous-navigation-part-3-understanding-slam-using-pose-graph-optimization-1594984678407.html>`_
, which explained the mapping process quite well.

Requirements
============
HomeR creates a map with the help of two ROS packages: 
`rplidar_ros <https://index.ros.org/p/rplidar_ros/>`_
, which allows ROS to work with the RPLIDAR A1 from 
`SLAMTEC <https://www.slamtec.com/en/lidar/a1>`_
and 
`slam_toolbox <https://github.com/SteveMacenski/slam_toolbox>`_
to simultaneous localization and mapping (SLAM)
.

It would be nice to have a gamepad or a keyboard, so that the mapping can be achieved by "*teleoperation*".

The keyboard teleop requires 
`teleop_twist_keyboard <https://index.ros.org/r/teleop_twist_keyboard/>`_
package, while 
the gamepad teleop needs
`teleop_twist_joy <https://index.ros.org/p/teleop_twist_joy/>`_ 
package.

Run following command in terminal to install these dependencies.

.. code-block:: console

   $ sudo apt install ros-jazzy-rplidar-ros ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard

Usage
=====

Bring Up HomeR Interface
------------------------

* Start HomeR driver node. 
* Set up static transforms from ``base_link`` to ``base_footprint`` and from ``base_link`` to ``lidar_link``.
* Launch customized ``rplidar_composition`` node.

**On Raspberry Pi**, launch the HomeR interface in terminal.

.. code-block:: console

   $ ros2 launch homer_control bringup_homer.launch.py


Teleop Mapping
--------------

* Launch ``teleop_launch.py`` with customed gamepad configurations.
* Launch ``online_async_launch.py`` with customed mapping configurations
* Start up ``rviz2`` for visualizing map, frames, lidar scans, etc..

**On host machine**, launch the gamepad controlled mapping in terminal.

.. code-block:: console

   $ ros2 launch homer_control joy_chart.launch.py

See an example below

.. figure:: ../images/joy_chart.gif
    :align: center

Gamepad is the default choice of teleoperation, feel free to start a ``teleop_twist_keyboard`` node to drive HomeR with a keyboard.

Save the Map
------------
*(To be continued)*



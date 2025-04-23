.. _slam:

SLAM: Simultaneous Localization and Mapping
*******************************************

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

Bring Up HomeR Interface (on **RPi**)
=====================================

.. code-block:: console

   $ ros2 launch homer_control bringup_homer.launch.py

This launch file includes following operations:

* Start HomeR driver node. 
* Set up static transforms from ``base_link`` to ``base_footprint`` and from ``base_link`` to ``lidar_link``.
* Launch customized ``rplidar_composition`` node.
* Launch `teleop-launch.py` from ``teleop_twist_joy`` package using custom gamepad configurations.

Gamepad is the default choice of teleoperation, feel free to start a ``teleop_twist_keyboard`` node to drive HomeR with a keyboard.

Teleop SLAM (on **Host**)
=========================

On host machine**, launch the gamepad controlled mapping in terminal.

.. code-block:: console

   $ ros2 launch homer_control joy_chart.launch.py

This launch file includes following operations:

* Launch ``online_async_launch.py`` with custom mapping configurations
* Start up ``rviz2`` for visualizing map, frames, lidar scans, etc..

See an example below

.. figure:: ../images/joy_chart.gif
    :align: center


Save the Map
============

Add ``SlamToolboxPlugin`` panel in Rviz.
----------------------------------------

Find it by going through ``Panels`` -> ``Add New Panel`` -> ``SlamToolboxPlugin`` (under ``slam_toolbox``).
Select it and click ``OK`` button.

Specify map path
----------------

In the text box next to the ``Serialize Map`` button, type desired path where you want to save the map, for instance: ``/home/username/Desktop/my_map``

Refer to the example below:

.. figure:: ../images/save_map.gif
    :align: center


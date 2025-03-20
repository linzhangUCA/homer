SLAM: Mapping
*************

Overview
========
HomeR can create a map with the help of two external packages: `rplidar_ros <https://index.ros.org/p/rplidar_ros/>`_ and `slam_toolbox <https://github.com/SteveMacenski/slam_toolbox>`_.
While wandering around, HomeR stitches the laser scans according to its *odometry* to fabricate a map eventually.
Brian Douglas made a remarkable video under the topic of `Pose Graph Optimization <https://www.mathworks.com/videos/autonomous-navigation-part-3-understanding-slam-using-pose-graph-optimization-1594984678407.html>`_, which explained the mapping process quite well.

Requirements
============
It would be nice to have a gamepad or a keyboard, so that the mapping process can be achieved by *teleoperation*.
To install all the dependencies,

.. code-block:: console

   $ sudo apt install ros-jazzy-rplidar-ros ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard

Usage
=====

Bring Up HomeR Driver Node
--------------------------

.. code-block:: console

   $ ros2 launch homer_control bringup_homer.launch.py


Joystick Guided Mapping
--------------------------

.. code-block:: console

   $ ros2 launch homer_control joy_chart.launch.py



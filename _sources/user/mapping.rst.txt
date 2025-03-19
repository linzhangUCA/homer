============
Create A Map
============

The map creation will be achieved by the previously set _odom_ frame system and LiDAR scans.
Two external packages are involved: `rplidar_ros <https://index.ros.org/p/rplidar_ros/>`_ and `slam_toolbox <https://github.com/SteveMacenski/slam_toolbox>`_

1. Bring Up HomeR Driver Node
-----------------------------

.. code-block:: console

   $ ros2 launch homer_control bringup_homer.launch.py


2. Joystick Guided Mapping
--------------------------

.. code-block:: console

   $ ros2 launch homer_control joy_chart.launch.py



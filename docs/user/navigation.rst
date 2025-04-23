Autonomous Navigation with SLAM
*******************************

Overview
========
Now, the navigation stack will kick in and guide HomeR to its goal.
It plans a trajectory from HomeR's current pose to its *goal pose*.
It monitors HomeR's status (pose and velocity) and gives out advice of velocity command (``cmd_vel``) to keep the robot on track. 
ROS2 uses a collection of pakcages from `navigation2 <https://github.com/ros-navigation/navigation2>`__ framework to navigate all kinds of robots.
Refer to the `First-Time Robot Setup Guide <https://docs.nav2.org/setup_guides/index.html>`__ for a more systematic introduction.

Requirements
============
`slam_toolbox <https://github.com/SteveMacenski/slam_toolbox>`__ is still required, so that the map will be served and HomeR gets localized in it.

`navigation2 <https://index.ros.org/p/navigation2/>`__ and `nav2_bringup <https://index.ros.org/p/nav2_bringup/>`__ will make the HomeR's dream of autonomous navigation come true.

Install dependencies in terminal.

.. code-block:: console

   $ sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

Configure Map Path (on **Host**)
================================
Use whatever text editor (VSCode below) to open the configuration file for ``slam_toolbox`` 

.. code-block:: console

   $ code $HOME/homer_ws/src/homer_control/configs/localization_params.yaml

Change line 18 to set ``map_file_name`` parameter to the path of the map which you've saved in the :ref:`slam` guide. 

.. code-block:: yaml

   map_file_name: /your/map/file/path/

Build ``homer_control`` package afterwards.

.. code-block:: console

   $ cd ~/homer_ws/
   $ colcon build --packages-select homer_control


Bring Up the HomeR Driver Interface (on **RPi**)
================================================

.. code-block:: console

   $ ros2 launch homer_control bringup_homer.launch.py

Start Navigation Stack (on **Host**)
====================================

.. code-block:: console

   $ ros2 launch homer_control navigate.launch.py

Set Goal Pose (on **Host**)
===========================

(Optional) Correct initial pose
-------------------------------

It is possible that HomeR not start at the same pose as the mapping stage.
In this case, you'll need to manually set HomeR's pose using the ``2D Pose Estimate`` tool from Rviz.
Find it in the ``Tools`` panel (usually on top of the window). 
Observe the true pose of the robot.
Click ``2D Pose Estimate`` button.
Click and hold left mouse button at the location in the map you think the robot is at.
Rotate the green arrow, until you think it is pointing to the robots heading direction.

Set goal pose
-------------

Find ``2D Goal Pose`` button in the ``Tools`` panel.
Click the button.
Click and hold left mouse button at the location in the map you want to send the robot to.
Rotate the green arrow, until you think it is pointing to the robot's end heading direction.

Refer to the example below:

.. figure:: ../images/set_goal_pose.gif
    :align: center




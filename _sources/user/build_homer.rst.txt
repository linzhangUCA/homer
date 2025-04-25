Set Up HomeR
************

Overview
========
Assume you have configured ROS development environment following the :ref:`ros_config` guide.
This guide will walk you through the whole process of setting up the HomeR using ROS.
You can also repeat the guide over and over again to practice core ROS commands.
**The ``HomeR`` needs to be deployed on both RPi and **Host**.
Let's get started from creating a ROS workspace.

Create ROS Workspace
====================
Let's create a ROS workspace under the ``$HOME`` directory with name ``ros2_ws``.

.. code-block:: console

   $ cd ~  # change directory to $HOME
   $ mkdir -p ros2_ws/src

Clone HomeR Project
===================
Download all ROS packages for ``HomeR`` from Github.

.. code-block:: console

   $ cd ~/ros2_ws/src/
   $ git clone https://github.com/linzhanguca/homer.git


Build ``homer_control`` package
===============================
Now, let's build the ``homer_control`` package

.. code-block:: console

  $ cd ~/ros2_ws  # CRITICAL!!! BUILD HAS TO BE DONE IN WORKSPACE
  $ colcon build
  $ source ~/ros2_ws/install/local_setup.bash  # introduce your packages to ROS

Now, the ``HomeR`` is good to go.


Afterwards
==========
If you don't want to repeat introducing ``HomeR`` to ROS every time you start a new terminal, you can add ``source ~/ros2_ws/install/local_setup.bash`` to the ``~/.bashrc`` file.
Or, execute following command **once**.

.. code-block:: console

  $ echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc

**Reminder**: set up ``HomeR`` on both RPi and Host.


Read the :ref:`slam` and :ref:`nav` guide for more examples.



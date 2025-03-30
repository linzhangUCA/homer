Configure ROS Environment (Post-Installation)
**********************************************

Overview
========
We need to configure the on-board Raspberry Pi (RPi) and the host machine (Host) to have a better experience when using the ROS.

Load ROS by Default (on **RPi and Host**)
=========================================
Execute following command in terminal **once** to append line: ``source /opt/ros/jazzy/setup.bash`` at the end of the ``$HOME/.bashrc`` file.
This allows any newly opened terminal to load ROS automatically.
Or, you'll have to manually source ROS setup every time you fire up a new terminal.

.. code-block:: console

   $ echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc

Sanity Check
------------
#. Close current terminal and open a new one.
   Make sure you don't see any file not found message in the new terminal.
#. Run following command in terminal.

.. code-block:: console

  $ ros2 topic list

And the expected result should be as follows.

.. code-block:: console

  /parameter_events
  /rosout

Set ROS Domain ID (on **RPi and Host**)
=======================================
Execute following command in terminal **once** to append line: ``export ROS_DOMAIN_ID=<id number>`` at the end of the ``$HOME/.bashrc`` file.
This allows the on-board Raspberry Pi to communicate with the host machine, **exclusively**, through your (wireless/wifi) network.

.. code-block:: console

   $ echo "export ROS_DOMAIN_ID=99" >> $HOME/.bashrc  # use a number between 0 and 101

Sanity Check
------------
#. Fire up a new terminal on both RPi and Host then type ``echo $ROS_DOMAIN_ID``.
   Compare if the out numbers are identical (or these two computer will lose communication).
#. On RPi, start a ``talker`` executable to send out messages.

.. code-block:: console

  $ ros2 run demo_nodes_cpp talker

#. On Host, start a ``listener`` executable to receive messages sent from RPi.

.. code-block:: console

  $ ros2 run demo_nodes_py listener

Synchronize Clock
=================
(*To be Continued*)


.. _ros_config:

Configure ROS Environment (Post-Installation)
**********************************************

1. Overview
===========
We need to distribute the ROS system on two computers: a Raspberry Pi (**RPi**) attaching to the robot; and a off-the-board host computer (**Host**).
We will let the RPI to handle the hardware interface (Communicate with the micro-controller), and let the Host to do computations required by SLAM and navigation packages.
You can refer to the official tutorial of `Configuring environment <https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html>`_.
And this guide is emphasizing configuring ROS environment on two computers working together.

2. Load ROS by Default (on **RPi** and **Host**)
================================================
Execute following command in terminal **once** to append line: ``source /opt/ros/jazzy/setup.bash`` at the end of the ``$HOME/.bashrc`` file.
This allows any newly opened terminal to load ROS automatically.
Or, you'll have to manually source ROS setup every time you fire up a new terminal.

.. code-block:: console

   $ echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc

Sanity Check
----------------
1. Close current terminal and open a new one.
   Make sure you don't see any file not found message in the new terminal.
2. Run following command in terminal.

.. code-block:: console

  $ ros2 topic list

And the expected result should be as follows.

.. code-block:: console

  /parameter_events
  /rosout

3. Set ROS Domain ID (on **RPi and Host**)
==========================================
Execute following command in terminal **once** to append line: ``export ROS_DOMAIN_ID=<id number>`` at the end of the ``$HOME/.bashrc`` file.
This allows the on-board Raspberry Pi to communicate with the host machine, **exclusively**, through your (wireless/wifi) network.

.. code-block:: console

   $ echo "export ROS_DOMAIN_ID=99" >> $HOME/.bashrc  # use a number between 0 and 101

Sanity Check
------------
1. Fire up a new terminal on both RPi and Host then type ``echo $ROS_DOMAIN_ID``.
   Compare if the out numbers are identical (or these two computer will lose communication).
2. On RPi, start a ``talker`` executable to send out messages.

.. code-block:: console

  $ ros2 run demo_nodes_cpp talker

3. On Host, start a ``listener`` executable to receive messages sent from RPi.

.. code-block:: console

  $ ros2 run demo_nodes_py listener

4. Synchronize Clock
====================
It is critical to have the RPi and the Host using the same clock.
And `Chrony <https://chrony-project.org/>`_  can sync the time on both computers via Network Time Protocol (NTP).
You can also refer to the blog `post <https://robofoundry.medium.com/how-to-sync-time-between-robot-and-host-machine-for-ros2-ecbcff8aadc4>`_.

1. Install Chrony (on **RPi** and **Host**)

Fire up a terminal, run following command:

.. code-block:: console

  $ sudo apt install chrony

2. Configure Chrony on **Host**

Open configuration file with ``sudo`` privilege (You can use any text editor, the example below use ``nano``).

.. code-block:: console

  $ sudo nano /etc/chrony/chrony.conf

Add following line to the end of the file

.. code-block:: yaml

  allow 192.168.0.0/24

Restart ``chronyd`` service on Host

.. code-block:: console

  $ sudo systemctl restart chronyd

3. Configure Chrony on **RPi**

Open configuration file with ``sudo`` privilege.

.. code-block:: console

  $ sudo nano /etc/chrony/chrony.conf

Add following line to the end of the file

.. code-block:: console

  $ server 192.168.0.XX iburst

``XX`` is your Host's IP address.
You can find your Host's IP address with command ``ip route``

Then restart ``chronyd`` service on RPi

.. code-block:: console

  $ sudo systemctl restart chronyd


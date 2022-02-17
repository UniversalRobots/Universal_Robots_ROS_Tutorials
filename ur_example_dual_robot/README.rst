ur_example_dual_robot
---------------------

**NOTE: This package is currently work in progress**

This demo is about integrating two robots into one URDF and starting a driver for both robots.

Requirements & Build
^^^^^^^^^^^^^^^^^^^^

You'll have to have the ``ur_robot_driver`` setup and installed as explained in its documentation.

To build and use this package, copy it to your catkin_workspace containing the driver, install its
dependencies using ``rosdep install --ignore-src --from-paths . -r -y`` and build your workspace as
usual.

For this demo to work, you'll need **docker** and **docker-compose** installed and your user needs
to be able to run docker containers. Those tools will be installed by the ``rosdep`` command above,
but you will have to setup your user to execute docker containers accordingly. See the `Docker
documentation <https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user>`_ for details

Startup
^^^^^^^

You'll need two shells: One for starting two simulated robots using docker + ursim and one for the
ROS components.

In the first shell execute

.. code-block:: bash

   rosrun ur_example_dual_robot docker_alice_bob.sh

Wait, until the robots are started up. You can connect to the robots using their web interface:
 - Alice: `http://10.5.0.5:6080/vnc.html <http://10.5.0.5:6080/vnc.html>`_
 - Bob: `http://10.5.0.6:6080/vnc.html <http://10.5.0.6:6080/vnc.html>`_

When the robots have booted, start the driver instances as follows

.. code-block:: bash

   roslaunch ur_example_dual_robot dual_robot_startup.launch

This should startup the drivers, an RViz instance and an rqt_joint_trajectory_controller window.

To steer the robots, you'll first have to start the external_control program on both using the web
interface (the programs should be loaded by default, simply start the robots and press the play
button). In the shell running the drivers, you should now see ``Robot connected to reverse interface.
Ready to receive control commands.`` twice.

Using the rqt_joint_trajectory_controller window you can select one of the robots, click on the big
red button and then use the sliders to move the robots around.

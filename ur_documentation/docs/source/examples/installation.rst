.. _examples_installation:

Installation
============
The following sections will assume that you've created a catkin_workspace and you've cloned and
built the driver and examples as explained in the following.

First, create a catkin_workspace, clone the repositories and build them:

.. code-block:: bash

   # create the workspace
   source /opt/ros/<your_ros_version>/setup.bash
   mkdir -p catkin_ws/src && cd catkin_ws

   # clone all necessary repositories
   git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Tutorials.git

   # install dependencies
   sudo apt update -qq
   rosdep update
   rosdep install --from-paths src --ignore-src -y

   # build
   catkin_make
   source devel/setup.bash


Remember to always source your workspace (as in the last line of the code snippet above) in every new shell.

Docker setup
------------

For most examples to work, you'll need **docker** and **docker-compose** installed and your user needs
to be able to run docker containers. Those tools will be installed by the ``rosdep`` command above,
but you will have to setup your user to execute docker containers accordingly. See the `Docker
documentation <https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user>`_ for details.

The short version:

.. code-block:: bash

   sudo groupadd docker
   sudo usermod -aG docker $USER
   # log out and log back in

After that you should be able to run docker containers using your user account:

.. code-block:: bash

   docker run --rm -it universalrobots/ursim_e-series

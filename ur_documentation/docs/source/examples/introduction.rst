Examples
========
This section documents the examples created in `<https://github.com/UniversalRobots/Universal_Robots_ROS_Tutorials>`_.

Those examples explain certain use cases in which the `ur_robot_driver <http://wiki.ros.org/ur_robot_driver>`_ could be used.

Installation
------------
The following sections will assume that you've created a catkin_workspace and you've cloned and
built the driver and examples as explained in the following.

First, create a catkin_workspace, clone the repositories and build them:

.. code-block:: bash

   # create the workspace
   source /opt/ros/<your_ros_version>/setup.bash
   mkdir -p catkin_ws/src && cd catkin_ws

   # clone all necessary repositories
   git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
   git clone -b calibration_devel https://github.com/fmauch/universal_robot.git
   git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Tutorials.git

   # install dependencies
   sudo apt update -qq
   rosdep update
   rosdep install --from-paths src --ignore-src -y

   # build
   catkin_make
   source devel/setup.bash


Remember to always source your workspace (as in the last line of the code snippet above) in every new shell.

.. include:: dual_robot.rst

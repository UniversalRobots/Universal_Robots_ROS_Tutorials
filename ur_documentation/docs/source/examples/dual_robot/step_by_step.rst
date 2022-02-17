Step by step explanation of a dual-robot setup
==============================================


This chapter explains all the steps necessary to create a dual-arm setup using the
``ur_robot_driver``.

In order to create a multi-robot workcell we basically have to consider three aspects:

* When integrating multiple robots into one *robot_description* we face one challenge: We can't have
  multiple *tf* frames with the same name, e.g. ``base_link`` in the same *robot_description*.
  Therefore, we'll have to use prefixes.
* Once the robot description uses tf prefixes, the joint names will also use those prefixes.
  Therefore, we'll need to adapt our robot controllers that they are defined on those "modified"
  joint names.
* As we will be starting multiple instances of the same ROS nodes, we'll have to take care that
  their names (nodes, topics, services) do not collide. One easy way to achieve this is using ROS
  namespaces

Assembling the URDF
-------------------

The ur_description_ package provides `macro files <https://github.com/ros-industrial/universal_robot/blob/melodic-devel-staging/ur_description/urdf/inc/ur5e_macro.xacro>`_ to generate an instance of a Universal Robots arm. We'll use this to assemble a description containing a Box with a UR3e and a UR10e ontop:

.. code-block:: xml
   :linenos:

   <?xml version="1.0"?>
   <robot xmlns:xacro="http://wiki.ros.org/xacro" name="my_work_cell">
     <!--Load the macro for creating a robot-->
     <xacro:include filename="$(find ur_description)/urdf/inc/ur10e_macro.xacro"/>
     <xacro:include filename="$(find ur_description)/urdf/inc/ur3e_macro.xacro"/>

     <!--Instanciate the robots-->
     <xacro:ur3e_robot prefix="bob_" kinematics_parameters_file="$(arg bob_kinematics)"/>
     <xacro:ur10e_robot prefix="alice_" kinematics_parameters_file="$(arg alice_kinematics)" />

     <!--common link where the tf tree originates from-->
     <link name="world"/>

     <!--Define the robot poses in the world-->
     <joint name="world_to_bob" type="fixed">
       <parent link="world" />
       <child link = "bob_base_link" />
       <origin xyz="-0.5 0 0" rpy="0 0 0" />
     </joint>
     <joint name="world_to_alice" type="fixed">
       <parent link="world" />
       <child link = "alice_base_link" />
       <origin xyz="0.5 0 0" rpy="0 0 0" />
     </joint>
   </robot>

Let's break it down:

First, we'll have to **include** the macros to generate the two arms:

.. code-block:: xml
   :linenos:
   :lineno-start: 4

     <xacro:include filename="$(find ur_description)/urdf/inc/ur10e_macro.xacro"/>
     <xacro:include filename="$(find ur_description)/urdf/inc/ur3e_macro.xacro"/>

The two include lines only loaded the macro for generating robots. Next, we can call the macros to
actually **create the arms**.

.. code-block:: xml
   :linenos:
   :lineno-start: 8

     <xacro:ur3e_robot prefix="bob_" kinematics_parameters_file="$(arg bob_kinematics)"/>
     <xacro:ur10e_robot prefix="alice_" kinematics_parameters_file="$(arg alice_kinematics)" />

This creates the two robots ``alice`` and ``bob``. We choose their names as a tf-prefix in order to
make the link and joint names unique. Note the trailing underscore in the prefixes, as the prefixes
will be added in front of the link and joint names without adding a further underscore.

The last thing to do is **connecting** the two robots to our common ``world`` link by adding two fixed
joints.

.. code-block:: xml
   :linenos:
   :lineno-start: 15

     <joint name="world_to_bob" type="fixed">
       <parent link="world" />
       <child link = "bob_base_link" />
       <origin xyz="-0.5 0 0" rpy="0 0 0" />
     </joint>
     <joint name="world_to_alice" type="fixed">
       <parent link="world" />
       <child link = "alice_base_link" />
       <origin xyz="0.5 0 0" rpy="0 0 0" />
     </joint>

We can view our custom workcell by running

.. code-block:: bash

   roslaunch ur_example_dual_robot view_dual_robot.launch

Use the sliders of the joint_state_publisher_gui to move the virtual robots around. It should look
something like this:

.. image:: rviz.png
   :alt: RViz window showing the dual arm setup

hello?

Define controller configuration files for the two robots
--------------------------------------------------------

Create a launchfile and start drivers for both robots
-----------------------------------------------------

Bonus: Use correct robot calibration with dual_robot setup
----------------------------------------------------------


.. _ur_description: https://github.com/ros-industrial/universal_robot/tree/melodic-devel-staging/ur_description

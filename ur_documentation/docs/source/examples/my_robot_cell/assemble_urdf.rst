===================
Assembling the URDF
===================

The `ur_description <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description>`_ package provides `macro files <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/rolling/urdf/ur_macro.xacro>`_ to generate an instance of a Universal Robots arm.
We'll use this to create a custom workcell with a ur20 inside: 

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_description/urdf/my_robot_cell.urdf.xacro
    :language: xml
    :linenos:
    :caption: my_robot_cell_description/urdf/my_robot_cell.urdf.xacro

Let's break it down:

First, we'll have to **include** the macro to generate the robot arm:

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_description/urdf/my_robot_cell.urdf.xacro
    :language: xml
    :start-at:   <xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>
    :end-at:   <xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>
    :linenos:
    :caption: my_robot_cell_description/urdf/my_robot_cell.urdf.xacro

This line only loaded the macro for generating the robot.

Later, we will call the macro to create the arm. Therefore, we need to declare certain arguments that must be passed to the macro.

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_description/urdf/my_robot_cell.urdf.xacro
    :language: xml
    :start-at:   <xacro:arg name="ur_type" default="ur20"/>
    :end-at:   <xacro:arg name="ur_input_recipe_filename" default="$(find ur_robot_driver)/resources/rtde_output_recipe.txt"/>
    :linenos:
    :caption: my_robot_cell_description/urdf/my_robot_cell.urdf.xacro
    
The following section contains all items within the workcell that are not part of the robot arm. If you are not experienced in writing URDFs, you may want to refer to this  `tutorial <https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/URDF-Main.html>`_.

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_description/urdf/my_robot_cell.urdf.xacro
    :language: xml
    :start-at:   <link name="world"/>
    :end-before:   <link name="robot_mount"/>
    :linenos: 
    :caption: my_robot_cell_description/urdf/my_robot_cell.urdf.xacro

This section of the URDF provides an example of what a custom workcell could resemble. Your workspace will likely vary from this one. Please feel free to modify this portion of the URDF to match your own setup. In this instance, our workspace comprises a table in front of a wall, featuring a monitor, and the **ur20** robot arm mounted on top.

The final step before generating the robot is to create its parent link.

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_description/urdf/my_robot_cell.urdf.xacro
    :language: xml
    :start-at: <link name="robot_mount"/>
    :end-before:   <xacro:ur_robot
    :linenos: 
    :caption: my_robot_cell_description/urdf/my_robot_cell.urdf.xacro

After that we are finally able to actually **create the robot arm** by calling the macro. 

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_description/urdf/my_robot_cell.urdf.xacro
    :language: xml
    :start-at:   <xacro:ur_robot
    :end-at:   </xacro:ur_robot>
    :linenos: 
    :caption: my_robot_cell_description/urdf/my_robot_cell.urdf.xacro

Note that the **origin** argument is transmitted in a different manner than the other arguments.

Before we can test our code, it's essential to build and source our Colcon workspace:

.. code-block:: bash

    #source and build your workspace
    colcon build
    source install/setup.bash


We can view our custom workspace by running:

.. code-block:: bash

    #launch rviz
    ros2 launch my_robot_cell_description view_workspace.launch.py

Use the sliders of the ``joint_state_puplisher_gui`` to move the virtual robot around.
It should look something like this:

.. image:: view_workspace.png
    :alt: RViz window showing the custom workspace
    
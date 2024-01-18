=========================
Start the ur_robot_driver
=========================

Before starting the ``ur_robot_driver``, it is necessary to add a **ros2_control** tag to our URDF. Luckily, the ``ur_robot_driver`` already provides us with a macro for that. We can simply include the macro in our assembled URDF.

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_control/urdf/my_robot_cell_control.urdf.xacro
    :language: py
    :linenos:
    :caption: my_robot_cell_control/urdf/my_robot_cell_control.urdf.xacro

This URDF is very similar to the one we have already assembled. We simply need to include the ros2_control macro, 

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_control/urdf/my_robot_cell_control.urdf.xacro
    :language: py
    :start-at:   <xacro:include filename="$(find ur_robot_driver)/urdf/ur.ros2_control.xacro"/>
    :end-at:   <xacro:include filename="$(find ur_robot_driver)/urdf/ur.ros2_control.xacro"/>
    :linenos: 
    :caption: my_robot_cell_control/urdf/my_robot_cell_control.urdf.xacro


define the necessary arguments that need to be passed to the macro,

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_control/urdf/my_robot_cell_control.urdf.xacro
    :language: py
    :start-at:     <xacro:arg name="robot_ip" default="0.0.0.0"/>
    :end-at:   <xacro:my_robot_cell/>
    :linenos: 
    :caption: my_robot_cell_control/urdf/my_robot_cell_control.urdf.xacro


and then call the macro by providing all the specified arguments.

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_control/urdf/my_robot_cell_control.urdf.xacro
    :language: py
    :start-at:    <xacro:ur_ros2_control
    :end-at:   </robot>
    :linenos: 
    :caption: my_robot_cell_control/urdf/my_robot_cell_control.urdf.xacro


Now that everything is in place, we can proceed to start up a driver instance. To initialize the ``ur_robot_driver`` using our customized workcell description, we need to create a launch file.

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_control/launch/start_robot.launch.py
    :language: py
    :linenos:
    :caption: my_robot_cell_control/launch/start_robot.launch.py

Let's break it down:

First we have to assemble all relevant files.

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_control/launch/start_robot.launch.py
    :language: py
    :start-at: def generate_launch_description():
    :end-at: ur_output_recipe_filename = PathJoinSubstitution([ur_robot_driver_package, "resources", "rtde_output_recipe.txt"])
    :linenos: 
    :caption: my_robot_cell_control/launch/start_robot.launch.py

If you are not familiar with extracting the calibration information of your UR robot and saving it in a .yaml file, please consider reading the relevant documentation on `extracting the calibration information <https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/installation/robot_setup.html#extract-calibration-information>`_ .

Next, we need to define all the arguments that we intend to pass through the shell.

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_control/launch/start_robot.launch.py
    :language: py
    :start-at: arg_robot_ip = DeclareLaunchArgument(
    :end-before: robot_description_content = Command(
    :linenos: 
    :caption: my_robot_cell_control/launch/start_robot.launch.py

Here for example we have chosen not to hardcode the robot's IP address. Therefore, we won't need to modify the launch file if the robot becomes accessible through a different IP.

In the next step we need to load our `custom urdf <file:///disk/users/cg247/tut/ur_documentation/docs/build/html/examples/custom_workspace/assemble_urdf.html>`_ with all required arguments. 

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_control/launch/start_robot.launch.py
    :language: py
    :start-at: robot_description_content = Command(
    :end-at: robot_description = {"robot_description": robot_description_content}
    :linenos: 
    :caption: my_robot_cell_control/launch/start_robot.launch.py

In the following parts we need to create all **relevant Nodes** to get the driver started.

We should begin by creating the **control node**, which requires our URDF, the update rate, and the ROS2 controllers file as parameters.

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_control/launch/start_robot.launch.py
    :language: py
    :start-at: control_node = Node(
    :end-before: controller_stopper_node = Node(
    :linenos: 
    :caption: my_robot_cell_control/launch/start_robot.launch.py


It's also a good idea to start the **controller stopper** node, which can stop and restart the ROS2 controllers as needed. 
To function properly, we should specify the **consistent controllers** that must run continuously and should not be stopped by this node.



.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_control/launch/start_robot.launch.py
    :language: py
    :start-at: controller_stopper_node = Node(
    :end-before: dashboard_client_node = Node(
    :linenos: 
    :caption: my_robot_cell_control/launch/start_robot.launch.py


The **dashboard node** needs the robot ip to be started. 

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_control/launch/start_robot.launch.py
    :language: py
    :start-at: dashboard_client_node = Node(
    :end-before: robot_state_publisher = Node(
    :linenos: 
    :caption: my_robot_cell_control/launch/start_robot.launch.py

The **robot state publisher** needs the urdf to be started.

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_control/launch/start_robot.launch.py
    :language: py
    :start-at: robot_state_publisher = Node(
    :end-before: def spawn_controller(name, activate=True):
    :linenos: 
    :caption: my_robot_cell_control/launch/start_robot.launch.py

The last nodes we need to start are the **controller spwaners**.

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_control/launch/start_robot.launch.py
    :language: py
    :start-at: def spawn_controller(name, activate=True):
    :end-before: return LaunchDescription(
    :linenos: 
    :caption: my_robot_cell_control/launch/start_robot.launch.py

This is a small usefull function to **spawn a controller** and set an inactive flag if needed.

Once we've determined which nodes we want to start, we can return the launch description with all arguments and nodes.

.. literalinclude:: ../../../../../my_robot_cell/my_robot_cell_control/launch/start_robot.launch.py
    :language: py
    :start-at: return LaunchDescription(
    :linenos: 
    :caption: my_robot_cell_control/launch/start_robot.launch.py

Remember that we haven't actually invoked the controller spawner node yet, so be sure to call the function for the ROS2 controllers you want to use.

Before we can test our code, it's essential to build and source our Colcon workspace:

.. code-block:: bash

    #cd to your colcon workspace root
    cd ~/colcon_ws

    #source and build your workspace
    colcon build
    source install/setup.bash

We can start the driver by running:

.. code-block:: bash

    #start the driver 
    ros2 launch my_robot_cell_control start_robot.launch.py robot_ip:=192.168.56.101
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution)

def generate_launch_description():
    control_package = FindPackageShare("my_robot_cell_control")
    update_rate_config_file = PathJoinSubstitution([control_package, "config", "update_rate.yaml"])
    controllers_file = PathJoinSubstitution([control_package, "config", "ros2_controllers.yaml"])
    robot_calibration_file= PathJoinSubstitution([control_package, "config", "my_robot_calibration.yaml"])
    description_file = PathJoinSubstitution([control_package, "urdf", "my_robot_cell_control.urdf.xacro"])

    ur_description_package = FindPackageShare("ur_description")
    joint_limits_file = PathJoinSubstitution([ur_description_package, "config", "ur20", "joint_limits.yaml"])
    physical_parameters_file = PathJoinSubstitution([ur_description_package, "config", "ur20", "physical_parameters.yaml"])
    visual_parameters_file = PathJoinSubstitution([ur_description_package, "config", "ur20", "visual_parameters.yaml"])

    ur_client_library_package = FindPackageShare("ur_client_library")
    ur_script_filename = PathJoinSubstitution([ur_client_library_package, "resources", "external_control.urscript"])
    
    ur_robot_driver_package = FindPackageShare("ur_robot_driver")
    ur_input_recipe_filename = PathJoinSubstitution([ur_robot_driver_package, "resources", "rtde_input_recipe.txt"])
    ur_output_recipe_filename = PathJoinSubstitution([ur_robot_driver_package, "resources", "rtde_output_recipe.txt"])

    arg_robot_ip = DeclareLaunchArgument(
        "robot_ip",
        default_value="yyy.yyy.yyy.yyy",
        )
    robot_ip = LaunchConfiguration("robot_ip")

    arg_controller_spawner_timeout = DeclareLaunchArgument(
        "controller_spawner_timeout",
        default_value="60",
    )
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")

    declared_arguments = [
        arg_robot_ip,
        arg_controller_spawner_timeout,
    ]


    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            description_file,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "kinematics_params:=",
            robot_calibration_file,
            " ",
            "joint_limit_params:=",
            joint_limits_file,
            " ",
            "physical_params:=",
            physical_parameters_file,
            " ",
            "visual_params:=",
            visual_parameters_file,
            " ",
            "ur_script_filename:=",
            ur_script_filename,
            " ",
            "ur_input_recipe_filename:=",
            ur_input_recipe_filename,
            " ",
            "ur_output_recipe_filename:=",
            ur_output_recipe_filename,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(controllers_file, allow_substs=False),
        ],
     )
    
    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"headless_mode": False},
            {
                "consistent_controllers": [
                    "joint_state_broadcaster",
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "speed_scaling_state_broadcaster",
                ]
            },
        ],
    )

    dashboard_client_node = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        parameters=[{"robot_ip": robot_ip}],
    )


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    
    def spawn_controller(name, activate=True):
        inactive_flag = ["--inactive"] if not activate else []
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name]
            + inactive_flag
            + ["--controller-manager-timeout", controller_spawner_timeout],
        )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            dashboard_client_node,
            controller_stopper_node,
            robot_state_publisher,
            spawn_controller("joint_state_broadcaster"),
            spawn_controller("io_and_status_controller"),
            spawn_controller("force_torque_sensor_broadcaster"),
            spawn_controller("speed_scaling_state_broadcaster"),
            spawn_controller('scaled_joint_trajectory_controller', activate=True),
            spawn_controller('forward_velocity_controller', activate=False),
            spawn_controller('forward_position_controller', activate=False),
            ]
    )


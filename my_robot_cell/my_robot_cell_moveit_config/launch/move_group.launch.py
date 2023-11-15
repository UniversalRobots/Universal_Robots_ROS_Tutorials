import yaml
import os
import math

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from ament_index_python.packages import get_package_share_directory


def construct_angle_radians(loader, node):
    """Utility function to construct radian values from yaml."""
    value = loader.construct_scalar(node)
    try:
        return float(value)
    except SyntaxError:
        raise Exception("invalid expression: %s" % value)


def construct_angle_degrees(loader, node):
    """Utility function for converting degrees into radians from yaml."""
    return math.radians(construct_angle_radians(loader, node))

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        yaml.SafeLoader.add_constructor("!radians", construct_angle_radians)
        yaml.SafeLoader.add_constructor("!degrees", construct_angle_degrees)
    except Exception:
        raise Exception("yaml support not available; install python-yaml")

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


joint_limits_path=get_package_share_directory("my_robot_cell_moveit_config") + "/config/joint_limits.yaml"

joint_limits_ur_description = load_yaml("ur_description", "config/ur20/joint_limits.yaml")


with open(joint_limits_path, "w") as fileStream:
    yaml.safe_dump(joint_limits_ur_description,fileStream)


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("my_robot_cell", package_name="my_robot_cell_moveit_config").to_moveit_configs()
    return generate_move_group_launch(moveit_config)

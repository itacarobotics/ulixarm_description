from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ulixarm", package_name="ulixarm_moveit_config").to_moveit_configs()
    print(moveit_config)
    return generate_rsp_launch(moveit_config)

import os
import typing

import yaml
from omni.isaac.core.scenes import Scene

from grutopia.core.config import TaskUserConfig
from grutopia.core.robot.robot import create_robots
from grutopia.core.robot.robot_model import RobotModels, RobotModel
from grutopia.core.util import log

# ROBOT_TYPES = {}

ROBOT_MODELS_PATH = os.path.join(
    os.path.split(os.path.realpath(__file__))[0], '../../../grutopia_extension/robots', 'robot_models.yaml')
log.info(f"Loading robot models from: {ROBOT_MODELS_PATH}")

with open(ROBOT_MODELS_PATH, 'r') as f:
    models = yaml.load(f.read(), Loader=yaml.FullLoader)
    log.info(f"Loaded models (raw): {models}")
    
    # Convert the loaded models to proper RobotModel instances
    if 'robots' in models:
        robot_list = [RobotModel(**robot) for robot in models['robots']]
        robot_models = RobotModels(robots=robot_list)
    else:
        raise ValueError(f"Invalid robot_models.yaml structure. Expected 'robots' key, got: {models.keys()}")
    
    log.info(f"Created RobotModels object: {robot_models}")


def init_robots(the_config: TaskUserConfig, scene: Scene) -> typing.Dict:
    log.info(f"Initializing robots with config: {the_config}")
    return create_robots(the_config, robot_models, scene)

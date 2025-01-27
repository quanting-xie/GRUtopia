from grutopia.core.config import SimulatorConfig
from grutopia.core.datahub.web_ui_api import clear as webui_clear
from grutopia.core.env import BaseEnv
from grutopia.core.util.container import is_in_container
import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from grutopia.core.util import log

# Load configuration
file_path = './GRUtopia/custom_scenes/configs/multi_npc_city.yaml'
sim_config = SimulatorConfig(file_path)

# Set up environment
headless = False
webrtc = False

if is_in_container():
    headless = True
    webrtc = True

# Create environment
env = BaseEnv(sim_config, headless=headless, webrtc=webrtc)

task_name = env.config.tasks[0].name

# Define different paths for each NPC
paths = {
    'h1': [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0), (5.0, 5.0, 0.0), (0.0, 5.0, 0.0)],  # Square path
    'robot1': [(3.0, 0.0, 0.0), (3.0, 8.0, 0.0), (8.0, 8.0, 0.0)],  # L-shaped path
    'robot2': [(-3.0, -3.0, 0.0), (3.0, -3.0, 0.0), (0.0, 3.0, 0.0)]  # Triangle path
}

# Initialize actions for all robots
actions = {
    'h1': {'move_with_keyboard': []},  # Player-controlled robot
    'robot1': {'move_along_path': [paths['robot1']]},  # First NPC following path
    'robot2': {'move_along_path': [paths['robot2']]}   # Second NPC following path
}

# Track path completion for each NPC
path_finished = {
    'robot1': False,
    'robot2': False
}

i = 0

# Main simulation loop
while env.simulation_app.is_running():
    i += 1
    env_actions = []
    env_actions.append(actions)
    obs = env.step(actions=env_actions)

    # Check path completion and update actions
    for robot_name in ['robot1', 'robot2']:
        if not path_finished[robot_name]:
            path_finished[robot_name] = obs[task_name][robot_name]['move_along_path'].get('finished', False)
            if path_finished[robot_name]:
                # When path is finished, start a new path (loop back to start)
                log.info(f'{robot_name} completed path, restarting')
                actions[robot_name] = {'move_along_path': [paths[robot_name]]}
                path_finished[robot_name] = False

    # Print debug info periodically
    if i % 100 == 0:
        print(f'Iteration: {i}')
        for robot_name in ['h1', 'robot1', 'robot2']:
            pos = obs[task_name][robot_name]['position']
            orient = quat_to_euler_angles(obs[task_name][robot_name]['orientation'])
            print(f'{robot_name} position: {pos}, orientation: {orient}')

# Cleanup
webui_clear()
env.simulation_app.close()

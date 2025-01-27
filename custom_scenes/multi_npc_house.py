import os
from grutopia.core.config import SimulatorConfig
from grutopia.core.datahub.web_ui_api import clear as webui_clear
from grutopia.core.env import BaseEnv
from grutopia.core.util.container import is_in_container

# Load configuration
file_path = './GRUtopia/custom_scenes/configs/multi_npc_house.yaml'
sim_config = SimulatorConfig(file_path)

# Get API key from environment
api_key = os.getenv('OPENAI_API_KEY')
if not api_key:
    raise ValueError("Please set OPENAI_API_KEY environment variable")

# Update API keys in config
if hasattr(sim_config.config, 'npc'):
    for npc in sim_config.config.npc:
        npc.openai_api_key = api_key

# Set up environment
headless = False
webrtc = False

if is_in_container():
    headless = True
    webrtc = True

env = BaseEnv(sim_config, headless=headless, webrtc=webrtc)

task_name = env.config.tasks[0].name
player_robot_name = 'h1'

# Initialize actions for all robots
actions = {
    'h1': {'move_with_keyboard': []},  # Player-controlled robot
    'robot1': {'move_by_speed': [0.0, 0.0]},  # NPC robot 1
    'robot2': {'move_by_speed': [0.0, 0.0]}   # NPC robot 2
}

i = 0

# Main simulation loop
while env.simulation_app.is_running():
    i += 1
    env_actions = []
    env_actions.append(actions)
    obs = env.step(actions=env_actions)

    if i % 100 == 0:
        print(i)
        print('Player position:', obs[task_name][player_robot_name]['position'])

# Cleanup
webui_clear()
env.simulation_app.close()

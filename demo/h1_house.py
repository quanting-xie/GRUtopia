from grutopia.core.config import SimulatorConfig
from grutopia.core.env import BaseEnv
from grutopia.core.util.container import is_in_container
from grutopia.core.util import log  # Add logging

file_path = './GRUtopia/demo/configs/h1_house.yaml'
sim_config = SimulatorConfig(file_path)

headless = False
webrtc = True  # Keep WebRTC enabled

if is_in_container():
    headless = True
    webrtc = True

env = BaseEnv(sim_config, headless=headless, webrtc=webrtc)

task_name = env.config.tasks[0].name
robot_name = env.config.tasks[0].robots[0].name

# Get initial observation
actions = {'h1': {'move_with_keyboard': []}}
env_actions = []
env_actions.append(actions)
obs = env.step(actions=env_actions)

# Print initial debug information
log.info(f"Task name: {task_name}")
log.info(f"Available objects in task: {obs[task_name].keys()}")

i = 0

while env.simulation_app.is_running():
    i += 1
    env_actions = []
    env_actions.append(actions)
    obs = env.step(actions=env_actions)

    if i % 100 == 0:
        print(f"Step {i}")
        # Print all available observations
        print(f"Available observations: {obs[task_name].keys()}")
        
        for obj_name in ['cctv_1', 'cctv_2']:
            if obj_name in obs[task_name]:
                print(f"Found {obj_name} in observations")
                camera_data = obs[task_name][obj_name].get('camera', {})
                print(f"{obj_name} camera data: {camera_data.keys() if camera_data else 'No camera data'}")
            else:
                print(f"{obj_name} not found in observations")

env.simulation_app.close()

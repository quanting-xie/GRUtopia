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

# Initialize actions for all robots
actions = {
    'h1': {'move_with_keyboard': []},
    'cctv_1': {},  # CCTVs don't need actions but need empty dict
    'cctv_2': {}
}

# Get initial observation
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
        for robot_name in ['cctv_1', 'cctv_2']:
            if robot_name in obs[task_name]:
                print(f"\nReadings from {robot_name}:")
                env_data = obs[task_name][robot_name].get('env_sensor', {})
                if env_data:
                    print(f"Temperature: {env_data['temperature']:.1f}°C")
                    print(f"Humidity: {env_data['humidity']:.1f}%")
                    print(f"Sensor position: {env_data['position']}")

env.simulation_app.close()

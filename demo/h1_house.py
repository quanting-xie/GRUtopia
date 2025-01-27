from grutopia.core.config import SimulatorConfig
from grutopia.core.env import BaseEnv
from grutopia.core.util.container import is_in_container

file_path = './GRUtopia/demo/configs/h1_house.yaml'
sim_config = SimulatorConfig(file_path)

headless = False
webrtc = True  # Enable WebRTC streaming

if is_in_container():
    headless = True
    webrtc = True

env = BaseEnv(sim_config, headless=headless, webrtc=webrtc)

task_name = env.config.tasks[0].name
robot_name = env.config.tasks[0].robots[0].name

i = 0

actions = {'h1': {'move_with_keyboard': []}}

while env.simulation_app.is_running():
    i += 1
    env_actions = []
    env_actions.append(actions)
    obs = env.step(actions=env_actions)

    if i % 100 == 0:
        print(i)
        # Monitor camera feeds
        for obj_name in ['cctv_1', 'cctv_2']:
            if obj_name in obs[task_name]:
                camera_data = obs[task_name][obj_name]['camera']
                if camera_data:
                    rgba = camera_data.get('rgba')
                    if rgba is not None:
                        print(f"{obj_name} camera active and streaming")

env.simulation_app.close()

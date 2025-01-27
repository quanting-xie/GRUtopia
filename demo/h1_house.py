from grutopia.core.config import SimulatorConfig
from grutopia.core.env import BaseEnv
from grutopia.core.util.container import is_in_container
import omni.kit.viewport.utility as vp_utils

file_path = './GRUtopia/demo/configs/h1_house.yaml'
sim_config = SimulatorConfig(file_path)

headless = False
webrtc = False

if is_in_container():
    headless = True
    webrtc = True

env = BaseEnv(sim_config, headless=headless, webrtc=webrtc)

# Get the viewport interface
viewport_interface = vp_utils.get_viewport_interface()

# Register CCTV cameras with viewport
for obj_name in ['cctv_1', 'cctv_2']:
    camera_path = f"/World/{obj_name}/camera"
    viewport_interface.add_viewport_camera(camera_path, obj_name)

task_name = env.config.tasks[0].name
robot_name = env.config.tasks[0].robots[0].name

i = 0

actions = {'h1': {'move_with_keyboard': []}}

while env.simulation_app.is_running():
    i += 1
    env_actions = []
    env_actions.append(actions)
    obs = env.step(actions=env_actions)

    # Access CCTV camera feeds every 100 steps
    if i % 100 == 0:
        print(i)
        
        # Get CCTV camera feeds
        for obj_name in ['cctv_1', 'cctv_2']:
            if obj_name in obs[task_name]:
                camera_data = obs[task_name][obj_name]['camera']
                if camera_data:
                    rgba = camera_data.get('rgba')
                    depth = camera_data.get('depth')
                    # Do something with the camera data
                    print(f"{obj_name} camera resolution: {rgba.shape if rgba is not None else 'No data'}")

env.simulation_app.close()

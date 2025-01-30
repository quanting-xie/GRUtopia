from grutopia.core.config import SimulatorConfig
from grutopia.core.env import BaseEnv
from grutopia.core.util.container import is_in_container
from grutopia.core.util import log  # Add logging
from omni.isaac.core.utils.rotations import quat_to_euler_angles
import numpy as np

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
        print(f"\nStep {i}")
        print(f"Available observations: {obs[task_name].keys()}")
        
        # Now look for robots by their base names with _0 suffix
        for robot_base_name in ['cctv_1', 'cctv_2']:
            robot_name = f"{robot_base_name}_0"  # Add _0 suffix
            if robot_name in obs[task_name]:
                print(f"\nReadings from {robot_name}:")
                print(f"Position: {obs[task_name][robot_name]['position']}")
                print(f"Orientation: {obs[task_name][robot_name]['orientation']}")  # Print orientation
                print(f"Available sensors: {obs[task_name][robot_name].keys()}")
                
                orientation = obs[task_name][robot_name]['orientation']
                euler_angles = np.degrees(quat_to_euler_angles(orientation))
                print(f"\n{robot_name} Euler angles (degrees):")
                print(f"Roll (X): {euler_angles[0]:.1f}")
                print(f"Pitch (Y): {euler_angles[1]:.1f}")
                print(f"Yaw (Z): {euler_angles[2]:.1f}")

                
                env_data = obs[task_name][robot_name].get('env_sensor', {})
                if env_data:
                    print(f"Temperature: {env_data['temperature']:.1f}°C")
                    print(f"Humidity: {env_data['humidity']:.1f}%")
                    print(f"Sensor position: {env_data['position']}")
                else:
                    print("No environmental sensor data available")

                camera_data = obs[task_name][robot_name].get('camera', {})
                if camera_data:
                    print(f"Camera data available: {camera_data.keys()}")
                else:
                    print("No camera data available")

env.simulation_app.close()

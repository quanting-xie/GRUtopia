from typing import Dict, Tuple
import numpy as np
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from grutopia.core.util import log  # Add logging for debugging

from grutopia.core.robot.robot import BaseRobot
from grutopia.core.robot.robot_model import RobotModel
from grutopia.core.config import RobotUserConfig
from omni.isaac.core.scenes import Scene

@BaseRobot.register("CCTVCamera")
class CCTVCamera(BaseRobot):
    """CCTV Camera implementation as a robot."""
    
    def __init__(self, config: RobotUserConfig, robot_model: RobotModel, scene: Scene):
        super().__init__(config, robot_model, scene)
        
        # Debug print the entire config
        log.info(f"CCTV {config.name} - Full config: {vars(config)}")
        
        # Convert parameters to numpy arrays if they exist
        color = np.array(config.color) if hasattr(config, 'color') and config.color is not None else np.array([0.2, 0.2, 0.2])
        scale = np.array(config.scale) if hasattr(config, 'scale') and config.scale is not None else np.array([0.1, 0.1, 0.1])
        position = np.array(config.position) if hasattr(config, 'position') and config.position is not None else np.array([0.0, 0.0, 0.0])
        
        # More explicit rotation handling with debug info
        if hasattr(config, 'rotation'):
            log.info(f"CCTV {config.name} - Found rotation attribute")
            if config.rotation is not None:
                log.info(f"CCTV {config.name} - Rotation value: {config.rotation}")
                euler_degrees = np.array(config.rotation)
            else:
                log.info(f"CCTV {config.name} - Rotation is None")
                euler_degrees = np.array([0.0, 0.0, 0.0])
        else:
            log.info(f"CCTV {config.name} - No rotation attribute found")
            euler_degrees = np.array([0.0, 0.0, 0.0])
        
        # Convert degrees to radians for euler_angles_to_quat
        euler_radians = np.radians(euler_degrees)
        log.info(f"CCTV {config.name} - Euler angles in radians: {euler_radians}")
        
        orientation = euler_angles_to_quat(euler_radians)
        log.info(f"CCTV {config.name} - Computed quaternion: {orientation}")
        
        # Create the physical representation
        self.isaac_robot = VisualCuboid(
            prim_path=config.prim_path,
            name=config.name,
            position=position,
            orientation=orientation,
            scale=scale,
            color=color
        )
        
        # Verify the orientation was set correctly
        pos, rot = self.get_world_pose()
        log.info(f"CCTV {config.name} - Final position: {pos}")
        log.info(f"CCTV {config.name} - Final orientation: {rot}")

    def apply_action(self, action: dict):
        """CCTVs don't need actions as they're static."""
        pass

    def get_obs(self) -> dict:
        """Get observation from the CCTV camera."""
        obs = {}
        
        # Get camera sensor data
        for sensor_name, sensor in self.sensors.items():
            sensor_data = sensor.get_data()
            if sensor_data:
                obs[sensor_name] = sensor_data
        
        # Add position and orientation
        pos, rot = self.get_world_pose()
        obs["position"] = pos
        obs["orientation"] = rot
        
        return obs

    def get_robot_base(self) -> RigidPrim:
        """Get the base prim of the CCTV."""
        return self.isaac_robot

    def get_robot_ik_base(self) -> RigidPrim:
        """CCTVs don't use IK, return the base."""
        return self.isaac_robot

from typing import Dict, Optional
import numpy as np

from grutopia.core.robot.robot import BaseRobot, Scene
from grutopia.core.robot.robot_model import SensorModel
from grutopia.core.robot.sensor import BaseSensor
from grutopia.core.util import log

@BaseSensor.register('EnvironmentalSensor')
class EnvironmentalSensor(BaseSensor):
    """Mock environmental sensor for temperature and humidity readings"""

    def __init__(self, config: SensorModel, robot: BaseRobot, name: str = None, scene: Scene = None):
        super().__init__(config, robot, name)
        # Set default ranges if not provided in config
        self.temp_range = getattr(config, 'temp_range', (20.0, 25.0))  # Default 20-25°C
        self.humidity_range = getattr(config, 'humidity_range', (40.0, 60.0))  # Default 40-60%
        
        # Store sensor position (relative to robot)
        self.position = np.array(getattr(config, 'position', [0, 0, 0]))
        
        # Add some noise to make readings more realistic
        self.noise_std = getattr(config, 'noise_std', 0.1)

    def sensor_init(self) -> None:
        """Initialize sensor - nothing special needed for mock sensors"""
        pass

    def get_data(self) -> Dict:
        """Generate mock environmental data readings"""
        if not self.config.enable:
            return {}
            
        # Get world position of the sensor by combining robot position and sensor offset
        robot_pos, _ = self._robot.get_world_pose()
        sensor_world_pos = robot_pos + self.position
        
        # Generate mock readings with some noise
        temperature = np.random.uniform(*self.temp_range) + np.random.normal(0, self.noise_std)
        humidity = np.random.uniform(*self.humidity_range) + np.random.normal(0, self.noise_std)
        
        return {
            'temperature': float(temperature),
            'humidity': float(humidity),
            'position': sensor_world_pos.tolist()
        } 
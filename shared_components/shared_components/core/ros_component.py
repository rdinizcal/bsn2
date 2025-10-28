from abc import ABC, abstractmethod
from central_hub.components.lifecycle_manager import LifecycleManager
from rclpy.lifecycle import LifecycleNode
from sensor.config_manager import ConfigManager
from sensor.publishers import PublisherManager
from shared_components.adaptation_handler import AdaptationHandler
from shared_components.battery_manager import BatteryManager

class RosComponent(ABC):
    def __init__(self, node: LifecycleNode):
        self.node = node
        self.active = False
        self._finalized = False
        self._heartbeat_timer = None
        self.config: ConfigManager = ConfigManager(self)
        self.battery_manager: BatteryManager = BatteryManager(self)
        self.publisher_manager: PublisherManager = PublisherManager(self)
        self.lifecycle_manager: LifecycleManager = LifecycleManager(self)

        self.adaptation_handler: AdaptationHandler = AdaptationHandler(self)
        if self.config.activate_adaptation:
            self.adaptation_handler.register_with_effector()
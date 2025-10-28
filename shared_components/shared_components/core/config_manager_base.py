# shared_components/shared_components/config_manager_base.py
from abc import ABC, abstractmethod
from typing import Any, Dict
from rclpy.lifecycle import LifecycleNode
from shared_components.enums import EventType
class ConfigManagerBase(ABC):
    def __init__(self, node: LifecycleNode):
        self.node = node
        # declare shared params
        node.declare_parameter("component", "")
        node.declare_parameter("frequency", "1.0")
        node.declare_parameter("battery_capacity", 100.0)
        node.declare_parameter("battery_level", 100.0)
        node.declare_parameter("battery_unit", 0.0)
        node.declare_parameter("instant_recharge", False)
        node.declare_parameter("enable_adaptation", False)

        self.component = node.get_parameter("component").value or ""
        self.frequency = float(node.get_parameter("frequency").get_parameter_value().string_value)

        node.declare_parameter("battery_id", f"{self.component}_battery")
        self.battery_id = node.get_parameter("battery_id").value
        self.battery_capacity = node.get_parameter("battery_capacity").value
        self.battery_level = node.get_parameter("battery_level").value
        self.battery_unit = node.get_parameter("battery_unit").value
        self.instant_recharge = node.get_parameter("instant_recharge").value
        
        self.activate_adaptation = self.node.get_parameter("enable_adaptation").value
        self.last_event = EventType.DEACTIVATE

    @abstractmethod
    def load(self,node) -> None:
        pass

    def validate(self) -> bool:
        if self.frequency <= 0:
            self.node.get_logger().error("frequency must be > 0")
            raise ValueError("frequency must be > 0")
        if not self.component:
            self.node.get_logger().error("component name must be set")
            raise ValueError("component name must be set")
        return True
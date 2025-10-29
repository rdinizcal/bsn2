from typing import Optional, Any
from abc import abstractmethod
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.timer import Timer
import rclpy

# shared managers (safe imports from shared_components to avoid circular deps)
from shared_components.lifecycle_manager import LifecycleManager
from shared_components.adaptation_handler import AdaptationHandler
from shared_components.battery_manager import BatteryManager


class RosComponent(LifecycleNode):
    """
    Base LifecycleNode that centralizes common BSN component responsibilities.

    Uses the Strategy Pattern via factory methods to allow subclasses to provide
    their own ConfigManager and PublisherManager implementations.

    Responsibilities included here (shared by sensors and the central hub):
    - lifecycle manager
    - battery manager
    - adaptation handler (registration is explicit)
    - heartbeat timer helpers
    - factory methods for child-specific managers (config, publisher, etc.)

    Subclasses MUST implement:
    - _create_config_manager() -> returns a ConfigManager instance
    - _create_publisher_manager() -> returns a PublisherManager instance
    
    Subclasses MAY implement:
    - _create_additional_managers() -> hook to create domain-specific managers
    """

    def __init__(self, node_name: str, parameters: Optional[list] = None):
        # initialize LifecycleNode
        super().__init__(node_name, parameter_overrides=parameters or [])

        self.config: Any = self._create_config_manager()
        self.battery_manager: BatteryManager = BatteryManager(self)
        self.lifecycle_manager: LifecycleManager = LifecycleManager(self)
        self.adaptation_handler: AdaptationHandler = AdaptationHandler(self)


        self.publisher_manager: Any = self._create_publisher_manager()
        
        # hook for additional child-specific managers (processor, fusion_engine, etc.)
        self._create_additional_managers()

        # internal state
        self.active: bool = False
        self._finalized: bool = False
        self._heartbeat_timer: Optional[Timer] = None
        
        # register with adaptation if config requires it
        self._register_adaptation()

    def _register_adaptation(self):
        if self.config.activate_adaptation:
            self.adaptation_handler.register_with_effector()

    @abstractmethod
    def _create_config_manager(self) -> Any:
        """
        Factory method: subclass must return its own ConfigManager instance.
        
        Example (in Sensor):
            from sensor.components.config_manager import ConfigManager
            return ConfigManager(self)
            
        Example (in CentralHub):
            from central_hub.components.config_manager import ConfigManager
            return ConfigManager(self)
        """
        raise NotImplementedError("Subclass must implement _create_config_manager()")

    @abstractmethod
    def _create_publisher_manager(self) -> Any:
        """
        Factory method: subclass must return its own PublisherManager instance.
        
        Example (in Sensor):
            from sensor.components.publishers import PublisherManager
            return PublisherManager(self)
            
        Example (in CentralHub):
            from central_hub.components.publishers import PublisherManager
            return PublisherManager(self)
        """
        raise NotImplementedError("Subclass must implement _create_publisher_manager()")

    def _create_additional_managers(self) -> None:
        """
        Hook for subclasses to create domain-specific managers.
        
        Override this in Sensor to create:
            self.processor = DataProcessor(self)
            self.risk_manager = RiskManager(self)
            
        Override this in CentralHub to create:
            self.sensor_handler = SensorDataHandler(self)
            self.fusion_engine = DataFusionEngine(self)
            self.risk_analyzer = RiskAnalyzer(self)
            self.visualizer = Visualizer(self)
        """
        pass
    @abstractmethod
    def run(self) -> None:
        raise NotImplementedError("Subclass must implement _run()")

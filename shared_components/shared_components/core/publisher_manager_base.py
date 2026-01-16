import rclpy
from shared_components.core.ros_component import RosComponent
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from abc import ABC, abstractmethod



# ROS2 interfaces
from bsn_interfaces.msg import Event, Status
from shared_components.enums import StatusContent, Task, EventType

class PublisherManagerBase(ABC):
    """Small shared publisher manager.

    Implements common create/publish helpers for `Status` and `Event`.
    Keep it minimal so it's easy to follow and safe in tests.
    """
    def __init__(self, node: RosComponent): # battery_manager: BatteryManager, config_manager: ConfigManagerBase):
        
        self.node = node
        self.status_pub = None
        self.event_pub = None
        self.qos_profile: QoSProfile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        if self.setup_publishers():
            self.node.get_logger().info("PublisherManagerBase initialized successfully")
        else:
            self.node.get_logger().error("PublisherManagerBase failed to initialize publishers")
    
    def _to_str(self, value) -> str:
        if hasattr(value, 'value'):
            return value.value
        if hasattr(value, 'name'):
            return value.name
        return str(value)

    def setup_publishers(self) -> bool:
        """Create the shared publishers (status + event)."""
        try:
            self.status_pub = self.node.create_publisher(Status, 'component_status', 10)
            self.event_pub = self.node.create_publisher(Event, 'collect_event', 10)
            return self.setup_exclusive_publishers()
        
        except Exception as e:
            self.node.get_logger().error(f'Failed to create base publishers: {e}')
            return False
        
    @abstractmethod
    def setup_exclusive_publishers(self) -> bool:
        pass

    def publish_status(self, content, task):
        if self.status_pub is None:
            self.node.get_logger().warning('Status publisher not available, skipping')
            return
        msg = Status()
        msg.source = self.node.get_name()
        msg.target = 'system'
        msg.content = self._to_str(content)
        msg.task = self._to_str(task)
        try:
            self.status_pub.publish(msg)
        except Exception:
            self.node.get_logger().warning('Failed to publish status')

    def publish_event(self, event_type: EventType):
        """
        Publish an event.
        
        Sends system events to notify other components of important
        state changes or operations.
        
        Args:
            event_type (str): Type of event (e.g., "activate", "deactivate").
        """
        if self.event_pub is None:
            self.node.get_logger().debug("Event publisher not available, skipping publish")
            return
            
        try:
            msg = Event()
            msg.source = self.node.get_name()
            msg.target = "system"
            # Event.content is a string in the IDL; convert Enum -> str
            msg.content = self._to_str(event_type)
            msg.freq = float(self.node.config.frequency)
            self.event_pub.publish(msg)
        except Exception as e:
            self.node.get_logger().warning(f"Failed to publish event: {e}")

        #self.node.get_logger().info(f"Event published: {event_type}")
        
    def publish_event_once(self, event_type: EventType):
        """
        Publish an event only when its normalized value differs from the
        last published event. This avoids flooding the system with the
        same event repeatedly while the node remains in the same state.

        """
        # Normalize event to string for stable comparison
        if event_type.name != self.node.config.last_event:
            try:
                self.publish_event(event_type)
                self.node.config.last_event = event_type
            except Exception as e:
                print("event didnt publish")
                self.node.get_logger().warning(f"Failed to publish event {event_type}: {e}")        

    def publish_heartbeat(self):
        """
        Publish periodic heartbeat.
        
        Sends regular heartbeat messages to indicate sensor node is alive
        and communicating. Content varies based on current operational state.
        """
        if self.status_pub is None:
            self.node.get_logger().debug("Status publisher not available, skipping publish")
            return

        msg = Status()
        msg.source = self.node.get_name()
        msg.target = "system"
        # Ensure content/task are strings (Status.msg expects strings)
        msg.content = StatusContent.RUNNING.value

        # Check for recharge mode
        if self.node.battery_manager.is_recharging:
            msg.task = Task.RECHARGING.value
        else:
            msg.task = Task.NORMAL.value

        try:
            self.status_pub.publish(msg)
            self.node.get_logger().debug(f"Heartbeat published: {msg.content}")
        except Exception as e:
            self.node.get_logger().warning(f"Failed to publish heartbeat: {e}")
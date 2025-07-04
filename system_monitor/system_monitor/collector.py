"""
Message collection and forwarding for Body Sensor Network.

This module provides message collection capabilities that gather system
messages from various BSN components and forward them to the Logger for
persistence and analysis.
"""

import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import Status, Event, EnergyStatus
import threading


class Probe(Node):
    """
    Message collector (probe) component for BSN system monitoring.
    
    This class acts as a centralized collection point for system monitoring
    messages. It receives messages from various BSN components and forwards
    them to the Logger for persistence and analysis. The probe serves as
    an intermediary layer between active system components and the logging
    infrastructure.
    
    The collector operates at a configurable frequency and provides debug
    heartbeat logging to indicate operational status.
    
    Attributes:
        frequency (float): Operating frequency for the collector heartbeat.
        log_status_pub: Publisher for forwarding status messages to logger.
        log_event_pub: Publisher for forwarding event messages to logger.
        log_energy_pub: Publisher for forwarding energy messages to logger.
        status_sub: Subscriber for component status messages.
        event_sub: Subscriber for system event messages.
        energy_sub: Subscriber for energy status messages.
        
    Examples:
        Basic usage:
        ```python
        import rclpy
        from system_monitor.collector import Probe
        
        rclpy.init()
        collector = Probe()
        collector.spin_collector()
        ```
        
        With custom frequency:
        ```python
        from rclpy.parameter import Parameter
        
        params = [Parameter('frequency', 5.0)]
        collector = Probe(parameter_overrides=params)
        ```
    """

    def __init__(self):
        """
        Initialize the message collector probe.
        
        Sets up message subscriptions from BSN components and creates
        publishers to forward messages to the Logger. Configures operating
        frequency for heartbeat and status reporting.
        """
        super().__init__('probe')
        self.get_logger().info("Starting BSN Probe")
        
        self.declare_parameter("frequency", 1.0).value
        self.frequency = self.get_parameter("frequency").value
        
        # Create publishers to log_* topics for the Logger
        self.log_status_pub = self.create_publisher(
            Status, 'log_status', 10)
            
        self.log_event_pub = self.create_publisher(
            Event, 'log_event', 10)
            
        self.log_energy_pub = self.create_publisher(
            EnergyStatus, 'log_energy_status', 10)
        
        # Create subscribers for component messages
        self.status_sub = self.create_subscription(
            Status, 'component_status', self.collect_status, 10)
            
        self.event_sub = self.create_subscription(
            Event, 'collect_event', self.collect_event, 10)
            
        self.energy_sub = self.create_subscription(
            EnergyStatus, 'collect_energy_status', self.collect_energy_status, 10)
            
        self.get_logger().info('Probe initialized and ready to collect messages')

    def collect_status(self, msg):
        """
        Collect and forward status messages from components.
        
        Receives component status messages and forwards them to the Logger
        for persistence. Provides debug logging for message flow tracking.
        
        Args:
            msg (Status): Status message from a system component containing
                         source, target, content, and task information.
        """
        self.get_logger().debug(f"Collected status from {msg.source}")
        self.log_status_pub.publish(msg)
    
    def collect_event(self, msg):
        """
        Collect and forward event messages from components.
        
        Receives system event messages and forwards them to the Logger
        for persistence. Events include lifecycle changes and heartbeats.
        
        Args:
            msg (Event): Event message from a system component containing
                        source, target, and content information.
        """
        self.get_logger().debug(f"Collected event from {msg.source}")
        self.log_event_pub.publish(msg)
    
    def collect_energy_status(self, msg):
        """
        Collect and forward energy status messages from components.
        
        Receives energy status messages containing battery levels and
        consumption information and forwards them to the Logger for
        power management analysis.
        
        Args:
            msg (EnergyStatus): Energy status message containing battery
                               level and consumption data from components.
        """
        self.get_logger().debug(f"Collected energy status from {msg.source}")
        self.log_energy_pub.publish(msg)

    def spin_collector(self):
        """
        Main collection loop with rate-controlled operation.
        
        Runs the collector at the configured frequency, providing periodic
        heartbeat logging to indicate operational status. The loop continues
        until ROS is shut down, maintaining consistent timing for system
        monitoring purposes.
        """
        rate = self.create_rate(self.frequency)
        while rclpy.ok():
            # Log heartbeat message to indicate collector is operational
            self.get_logger().debug("Collector heartbeat")
            
            # Sleep at the configured frequency
            rate.sleep()


def main(args=None):
    """
    Main entry point for the message collector node.
    
    Initializes ROS, creates the collector probe, and runs both the ROS
    communication loop and the collection loop in coordinated threads.
    
    Args:
        args: Command line arguments passed to ROS initialization.
    """
    rclpy.init(args=args)

    collector = Probe()

    thread = threading.Thread(target=rclpy.spin, args=(collector,), daemon=True)
    thread.start()

    try:
        collector.spin_collector()
    finally:
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
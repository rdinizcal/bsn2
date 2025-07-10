#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Message logging and persistence for Body Sensor Network.

This module receives various system messages and converts them to persistent
format for storage and analysis. It handles status updates, events, energy
reports, adaptation commands, and uncertainty messages from all BSN components.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from bsn_interfaces.msg import Status, Event, EnergyStatus, AdaptationCommand, Uncertainty, Persist
import threading


class Logger(Node):
    """
    Message logger and persistence handler for the BSN system.
    
    This class receives messages from various system components and converts
    them to a standardized Persist message format for storage and analysis.
    It handles multiple message types including status updates, events,
    energy reports, adaptation commands, and uncertainty notifications.
    
    The logger adds timestamps and formats messages consistently to enable
    system analysis, debugging, and performance monitoring.
    
    Attributes:
        frequency (float): Operating frequency for the logger.
        time_ref (int): Reference timestamp for relative time calculations.
        persist_pub: Publisher for persistent log messages.
        status_sub: Subscriber for component status messages.
        event_sub: Subscriber for system event messages.
        energy_sub: Subscriber for energy status messages.
        adapt_sub: Subscriber for adaptation command messages.
        uncertainty_sub: Subscriber for uncertainty messages.
        
    Examples:
        Basic usage:
        ```python
        import rclpy
        from system_monitor.logger import Logger
        
        rclpy.init()
        logger = Logger()
        rclpy.spin(logger)
        ```
        
        With custom frequency:
        ```python
        from rclpy.parameter import Parameter
        
        params = [Parameter('frequency', 5.0)]
        logger = Logger(parameter_overrides=params)
        ```
    """
    
    def __init__(self):
        """
        Initialize the logger node.
        
        Sets up message subscriptions for all BSN message types and creates
        the persistence publisher. Configures timing reference for consistent
        timestamp generation across all logged messages.
        """
        super().__init__('logger')
        
        # Declare parameters
        self.declare_parameter("frequency", 2.0)
        self.frequency = self.get_parameter("frequency").value
        
        # Store time reference for consistent timestamps (matching C++ behavior)
        self.time_ref = self.get_clock().now().nanoseconds
        
        # Setup QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,  # Matching C++ queue size
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Setup publishers (matching C++ original)
        self._setup_publishers(qos_profile)
        
        # Setup subscribers (matching C++ original)
        self._setup_subscribers(qos_profile)
        
        self.get_logger().info('Logger started - persisting and routing messages')
    
    def _setup_publishers(self, qos_profile):
        """Setup publishers following C++ Logger pattern"""
        
        # Publisher for persistence (to DataAccess)
        self.persist_pub = self.create_publisher(Persist, 'persist', qos_profile)
        
        # Publishers for message routing (matching C++ Logger::setUp())
        
        # Republish adaptation commands as 'reconfigure' for ParamAdapter
        self.reconfigure_pub = self.create_publisher(
            AdaptationCommand, 'reconfigure', qos_profile)
        
        # Republish events for Enactor
        self.event_pub = self.create_publisher(Event, 'event', qos_profile)
        
        # Republish status messages (for other components that might need them)
        self.status_pub = self.create_publisher(Status, 'status', qos_profile)
        
        self.get_logger().info('Publishers initialized: persist, reconfigure, event, status')
    
    def _setup_subscribers(self, qos_profile):
        """Setup subscribers following C++ Logger body() pattern"""
        
        # Subscribe to log_* topics (matching C++ Logger::body())
        self.adapt_sub = self.create_subscription(
            AdaptationCommand, 'log_adapt', self.receive_adaptation_command, qos_profile)
            
        self.status_sub = self.create_subscription(
            Status, 'log_status', self.receive_status, qos_profile)
            
        self.energy_sub = self.create_subscription(
            EnergyStatus, 'log_energy_status', self.receive_energy_status, qos_profile)
            
        self.event_sub = self.create_subscription(
            Event, 'log_event', self.receive_event, qos_profile)
            
        self.uncertainty_sub = self.create_subscription(
            Uncertainty, 'log_uncertainty', self.receive_uncertainty, qos_profile)
        
        self.get_logger().info('Subscribers initialized for log_* topics')
    
    def now(self):
        """
        Get current time in milliseconds since logger start.
        
        Provides consistent timestamp generation for all logged messages
        using millisecond precision relative to the logger's initialization.
        
        Returns:
            int: Current timestamp in milliseconds since logger started.
        """
        return self.get_clock().now().nanoseconds
    
    def receive_status(self, msg):
        """
        Process and persist a component status message.
        
        Converts incoming status messages to persistent format with
        timestamp and standardized fields for long-term storage and analysis.
        
        Args:
            msg (Status): Status message containing source, target, content,
                         and task information from a system component.
        """
        # Create persist message
        persist_msg = Persist()
        persist_msg.source = msg.source
        persist_msg.target = msg.target
        persist_msg.type = "Status"
        persist_msg.timestamp = self.now() - self.time_ref
        persist_msg.content = msg.content
        
        # Publish to persist (for DataAccess)
        self.persist_pub.publish(persist_msg)
        
        # Republish original message
        self.status_pub.publish(msg)
        
        self.get_logger().debug(f"Logged status: {msg.source}: {msg.content}")
    
    def receive_event(self, msg):
        """
        Process and persist a system event message.
        
        Converts incoming event messages to persistent format for tracking
        system state changes, lifecycle events, and heartbeat signals.
        
        Args:
            msg (Event): Event message containing source, target, and content
                        representing system events or state changes.
        """
        # Create persist message
        persist_msg = Persist()
        persist_msg.source = msg.source
        persist_msg.target = msg.target
        persist_msg.type = "Event"
        persist_msg.timestamp = self.now() - self.time_ref
        persist_msg.content = msg.content
        
        # Publish to persist (for DataAccess)
        self.persist_pub.publish(persist_msg)
        
        # CRITICAL: Republish to 'event' topic (for Enactor)
        self.event_pub.publish(msg)
        
        self.get_logger().debug(
            f"Logged and routed event: {msg.source}: {msg.content}")
    
    def receive_energy_status(self, msg):
        """
        Process and persist an energy status message.
        
        Converts incoming energy status messages to persistent format for
        tracking battery levels, energy consumption, and power management
        across all system components.
        
        Args:
            msg (EnergyStatus): Energy status message containing battery
                               level and consumption information.
        """
        # Create persist message
        persist_msg = Persist()
        persist_msg.source = msg.source
        persist_msg.target = msg.target
        persist_msg.type = "EnergyStatus"
        persist_msg.timestamp = self.now() - self.time_ref
        persist_msg.content = msg.content
        
        # Publish to persist (for DataAccess)
        self.persist_pub.publish(persist_msg)
        
        self.get_logger().debug(f"Logged energy: {msg.source}: {msg.content}")
    
    def receive_adaptation_command(self, msg):
        """
        Process and persist an adaptation command message.
        
        Converts incoming adaptation commands to persistent format for
        tracking system adaptation behavior and command execution.
        
        Args:
            msg (AdaptationCommand): Adaptation command containing source,
                                    target, and action information for
                                    system reconfiguration.
        """
        # Create persist message
        persist_msg = Persist()
        persist_msg.source = msg.source
        persist_msg.target = msg.target
        persist_msg.type = "AdaptationCommand"
        persist_msg.timestamp = self.now() - self.time_ref
        persist_msg.content = msg.action
        
        # Publish to persist (for DataAccess)
        self.persist_pub.publish(persist_msg)
        
        # CRITICAL: Republish to 'reconfigure' topic (for ParamAdapter)
        self.reconfigure_pub.publish(msg)
        
        self.get_logger().debug(
            f"Logged and routed adaptation: {msg.source} -> {msg.target}: {msg.action}")
    
    def receive_uncertainty(self, msg):
        """
        Process and persist an uncertainty message.
        
        Converts incoming uncertainty notifications to persistent format
        for tracking system uncertainty levels and confidence metrics.
        
        Args:
            msg (Uncertainty): Uncertainty message containing uncertainty
                              level or confidence information from components.
        """
        # Create persist message
        persist_msg = Persist()
        persist_msg.source = msg.source
        persist_msg.target = msg.target
        persist_msg.type = "Uncertainty"
        persist_msg.timestamp = self.now() - self.time_ref
        persist_msg.content = msg.content
        
        # Publish to persist (for DataAccess)
        self.persist_pub.publish(persist_msg)
        
        self.get_logger().debug(f"Logged uncertainty: {msg.source}: {msg.content}")


def main(args=None):
    """
    Main entry point for the logger node.
    
    Initializes ROS, creates the logger, and runs it with rate-controlled
    execution. The logger runs continuously processing incoming messages
    and converting them to persistent format.
    
    Args:
        args: Command line arguments passed to ROS initialization.
    """
    rclpy.init(args=args)

    logger = Logger()

    # Run spin in a thread, make thread daemon so we don't have to join it to exit
    thread = threading.Thread(
        target=rclpy.spin, args=(logger,), daemon=True
    )
    thread.start()
    rate = logger.create_rate(logger.frequency)  # 2 Hz

    try:
        while rclpy.ok():
            rate.sleep()
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
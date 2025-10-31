#!/usr/bin/env python3
"""
Combined system monitor and message collector for Body Sensor Network.

This module provides comprehensive system monitoring capabilities including
message collection and forwarding, node activity tracking, heartbeat monitoring,
and system status reporting for the BSN system.
"""

from pyparsing import Optional
import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import Status, Event, EnergyStatus
import threading
import time


class SystemMonitor(Node):
    """
    Combined system monitor and message collector for BSN system.
    
    This class provides dual functionality as both a message collector that
    forwards system messages to the Logger and a comprehensive system monitor
    that tracks node activity, lifecycle events, and system health.
    
    The monitor tracks individual node states including activation status,
    current tasks, battery levels, and heartbeat timing to provide real-time
    system oversight and detect potential issues.
    
    Attributes:
        node_list (list): List of node names to monitor.
        heartbeat_timeout (float): Maximum time between heartbeats before warning.
        check_interval (float): Frequency for checking node status.
        debug (bool): Whether debug logging is enabled.
        frequency (float): Operating frequency for status updates.
        monitored_nodes (dict): Dictionary tracking state of each monitored node.
        log_status_pub: Publisher for forwarding status messages to logger.
        log_event_pub: Publisher for forwarding event messages to logger.
        log_energy_pub: Publisher for forwarding energy messages to logger.
        status_pub: Publisher for system monitor status reports.
        
    Examples:
        Basic usage:
        ```python
        import rclpy
        from system_monitor.node_monitor import SystemMonitor
        
        rclpy.init()
        monitor = SystemMonitor()
        rclpy.spin(monitor)
        ```
        
        With custom configuration:
        ```python
        monitor = SystemMonitor()
        monitor.node_list = ["sensor1", "sensor2", "hub"]
        monitor.heartbeat_timeout = 10.0
        ```
    """

    def __init__(self, node_name: str, parameters= None):
        # initialize LifecycleNode
        super().__init__(node_name, parameter_overrides=parameters or [])
        self.get_logger().info("Starting Combined System Monitor")
        
        # Configure parameters
        self.declare_parameter('monitored_nodes', ['thermometer_node', 'oximeter_node', 'central_hub_node'])
        self.declare_parameter('heartbeat_timeout', 3.0)
        self.declare_parameter('check_interval', 2.0) 
        self.declare_parameter('debug_level', False)
        self.declare_parameter('frequency', 1.0)
        
        # Get parameters
        self.node_list = self.get_parameter('monitored_nodes').value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self.check_interval = self.get_parameter('check_interval').value
        self.debug = self.get_parameter('debug_level').value
        self.frequency = self.get_parameter('frequency').value
        
        # Track monitored nodes and their states
        self.monitored_nodes = {}
        
        # Initialize state tracking for each node
        for node_name in self.node_list:
            self.monitored_nodes[node_name] = {
                'state': 'unknown',
                'last_heartbeat': time.time(),
                'active': False,
                'task': 'idle',       
                'content': 'unknown',
                'is_recharging': False
            }
        
        # === COLLECTOR FUNCTIONALITY ===
        # Create publishers to log_* topics for the Logger
        self.log_status_pub = self.create_publisher(
            Status, 'log_status', 10)
            
        self.log_event_pub = self.create_publisher(
            Event, 'log_event', 10)
            
        self.log_energy_pub = self.create_publisher(
            EnergyStatus, 'log_energy_status', 10)
            
        
        # === SHARED SUBSCRIPTIONS ===
        # Subscribe to all component messages
        self.status_sub = self.create_subscription(
            Status, 'component_status', self.status_callback, 10)
            
        self.event_sub = self.create_subscription(
            Event, 'collect_event', self.event_callback, 10)
            
        self.energy_subscribers = []
        for node_name in self.node_list:
            # Remove '_node' suffix to get sensor name
            sensor_name = node_name.replace('_node', '') if node_name.endswith('_node') else node_name
            energy_topic = f'collect_energy_status/{sensor_name}'
            
            try:
                energy_sub = self.create_subscription(
                    EnergyStatus, energy_topic, self.energy_callback, 10)
                self.energy_subscribers.append(energy_sub)
                self.get_logger().info(f"Subscribed to energy topic: {energy_topic}")
            except Exception as e:
                self.get_logger().warn(f"Failed to subscribe to {energy_topic}: {e}")
        
        # === TIMERS ===
        # Start heartbeat monitoring timer  
        self.create_timer(1.0, self.check_heartbeats)
        
        # Add a timer to publish status
        self.create_timer(2.0, self.publish_monitor_status)
        
        self.get_logger().info(f"Monitoring {len(self.node_list)} nodes: {self.node_list}")
        self.get_logger().info("System monitor initialized and ready")

    def status_callback(self, msg):
        """
        Process and forward status messages from system components.
        
        Handles incoming component status messages by forwarding them to the
        Logger and updating internal node state tracking. Also monitors for
        missing heartbeats and publishes deactivation status when nodes go silent.
        
        Args:
            msg (Status): Status message containing source, target, content,
                         and task information from a system component.
        """
        # Process for monitoring (NodeMonitor functionality) FIRST
        node_name = msg.source
        content = msg.content
        task = msg.task
        
        # Update heartbeat timestamp when we receive ANY status message
        current_time = time.time()
        
        # Update node in tracked list if it's one we care about
        for monitored_node in self.monitored_nodes:
            if node_name == monitored_node or node_name.endswith(monitored_node.split('/')[-1]):
                # UPDATE HEARTBEAT - This is the key addition
                self.monitored_nodes[monitored_node]['last_heartbeat'] = current_time
                
                # Update status info
                self.monitored_nodes[monitored_node]['task'] = task
                self.monitored_nodes[monitored_node]['content'] = content
                
                # Auto-update active state based on status content
                if content == "activated":
                    self.monitored_nodes[monitored_node]['active'] = True
                    self.monitored_nodes[monitored_node]['is_recharging'] = False
                elif content == "deactivated":
                    self.monitored_nodes[monitored_node]['active'] = False
                
                # Check for recharging status
                if task == "recharging":
                    self.monitored_nodes[monitored_node]['is_recharging'] = True
                else:
                    self.monitored_nodes[monitored_node]['is_recharging'] = False
                
                # Log significant state changes
                old_status = self.monitored_nodes[monitored_node].get('last_status') 
                if old_status != content:
                    self.get_logger().info(f"Node {node_name} status: {content} ({task})")
                    self.monitored_nodes[monitored_node]['last_status'] = content
                
                break
    
        # Forward to Logger (Collector functionality) - use ORIGINAL message
        # Don't modify the original message content
        self.log_status_pub.publish(msg)
        
    def event_callback(self, msg):
        """
        Process and forward event messages from system components.
        
        Handles incoming event messages by forwarding them to the Logger
        and updating heartbeat timing and activation states. Events serve
        as heartbeats for monitoring node liveliness.
        
        Args:
            msg (Event): Event message containing source, target, and content
                        information representing system events or heartbeats.
        """
        # Forward to Logger (Collector functionality)
        self.log_event_pub.publish(msg)
        
        # Process for monitoring (NodeMonitor functionality)
        node_name = msg.source
        content = msg.content
        
        # Update node in tracked list if it's one we care about
        for monitored_node in self.monitored_nodes:
            if node_name == monitored_node or node_name.endswith(monitored_node.split('/')[-1]):
                self.get_logger().debug(f"Event from {node_name}: {content}")
                self.monitored_nodes[monitored_node]['last_heartbeat'] = time.time()
                
                # Track active state
                if content == "activate":
                    self.monitored_nodes[monitored_node]['active'] = True
                elif content == "deactivate":
                    self.monitored_nodes[monitored_node]['active'] = False
                elif content == "recharging":
                    self.monitored_nodes[monitored_node]['is_recharging'] = True
                
                # Log state changes
                old_content = self.monitored_nodes[monitored_node].get('last_event')
                if old_content != content:
                    self.get_logger().info(f"Node {node_name} event: {content}")
                    self.monitored_nodes[monitored_node]['last_event'] = content
                
                break
    
    def energy_callback(self, msg):
        """
        Process and forward energy status messages from system components.
        
        Handles incoming energy status messages by forwarding them to the
        Logger for persistence. Energy messages provide battery level and
        consumption information for system monitoring.
        
        Args:
            msg (EnergyStatus): Energy status message containing battery
                               level and energy consumption information.
        """
        # Forward to Logger (Collector functionality)
        self.log_energy_pub.publish(msg)
        
        # Process for monitoring if needed
        node_name = msg.source
        self.get_logger().debug(f"Energy update from {node_name}: {msg.content}")

    def check_heartbeats(self):
        """
        Check for missing heartbeats and log warnings for silent nodes.
        
        Monitors the time since last heartbeat for each tracked node and
        logs warnings when active nodes haven't sent heartbeats within the
        configured timeout period. Inactive nodes are only logged in debug mode.
        """
        current_time = time.time()
        
        for node_name, info in self.monitored_nodes.items():
            time_since_last = current_time - info['last_heartbeat']
            
            # Only log warning if node should be active
            if time_since_last > self.heartbeat_timeout:
                if info['active']:
                    self.get_logger().warn(f"Node {node_name} hasn't sent a heartbeat in {time_since_last:.1f} seconds!")
                elif self.debug:
                    self.get_logger().debug(f"Inactive node {node_name} silent for {time_since_last:.1f}s")

    def publish_monitor_status(self):
        """
        Publish current status of all monitored nodes to log_status.
        """
        for node_name, info in self.monitored_nodes.items():
            # Create a Status message for each monitored node
            status_msg = Status()
            status_msg.source = node_name  # ← The node being reported on
            status_msg.target = 'system'
            status_msg.task = info.get('task', 'monitoring')  # ← The node's current task
            
            # Create detailed status content
            recharge_status = "RECHARGING" if info.get('is_recharging', False) else ""
            
            status_msg.content = info.get('content', 'unknown')
            
            # Publish to log_status
            self.log_status_pub.publish(status_msg)


def main(args=None):
    """
    Main entry point for the system monitor node.
    
    Initializes ROS, creates the system monitor, and runs it in a
    multi-threaded executor to handle message processing and monitoring
    tasks concurrently.
    
    Args:
        args: Command line arguments passed to ROS initialization.
    """
    rclpy.init(args=args)
    node = SystemMonitor()
    
    # Spin in a separate thread
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # Keep the main thread alive
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
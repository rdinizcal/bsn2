#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import Status, Event, EnergyStatus
from std_msgs.msg import String
import threading
import time


class SystemMonitor(Node):
    """
    Combined system monitor and message collector for BSN system.
    
    Features:
    - Collects and forwards all system messages to the Logger
    - Tracks node activity through heartbeats
    - Monitors node states and lifecycle events
    - Reports detailed system status
    - Detects and reports missing heartbeats
    """

    def __init__(self):
        super().__init__('node_monitor')
        self.get_logger().info("Starting Combined System Monitor")
        
        # Configure parameters
        self.declare_parameter('monitored_nodes', ['thermometer_node', 'oximeter_node', 'central_hub'])
        self.declare_parameter('heartbeat_timeout', 5.0)
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
            
        # === MONITOR FUNCTIONALITY ===
        # Publisher for monitor status
        self.status_pub = self.create_publisher(
            String,
            'system_monitor/status',
            10
        )
        
        # === SHARED SUBSCRIPTIONS ===
        # Subscribe to all component messages
        self.status_sub = self.create_subscription(
            Status, 'component_status', self.status_callback, 10)
            
        self.event_sub = self.create_subscription(
            Event, 'collect_event', self.event_callback, 10)
            
        self.energy_sub = self.create_subscription(
            EnergyStatus, 'collect_energy_status', self.energy_callback, 10)
        
        # === TIMERS ===
        # Start heartbeat monitoring timer  
        self.create_timer(1.0, self.check_heartbeats)
        
        # Add a timer to publish status
        self.create_timer(2.0, self.publish_monitor_status)
        
        self.get_logger().info(f"Monitoring {len(self.node_list)} nodes: {self.node_list}")
        self.get_logger().info("System monitor initialized and ready")

    def status_callback(self, msg):
        """Process and forward status messages"""
        # Forward to Logger (Collector functionality)
        self.log_status_pub.publish(msg)
        
        # Process for monitoring (NodeMonitor functionality)
        node_name = msg.source
        content = msg.content
        task = msg.task
        
        # Update node in tracked list if it's one we care about
        for monitored_node in self.monitored_nodes:
            if node_name == monitored_node or node_name.endswith(monitored_node.split('/')[-1]):
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
    
    def event_callback(self, msg):
        """Process and forward event messages"""
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
        """Process and forward energy status messages"""
        # Forward to Logger (Collector functionality)
        self.log_energy_pub.publish(msg)
        
        # Process for monitoring if needed
        node_name = msg.source
        self.get_logger().debug(f"Energy update from {node_name}: {msg.content}")

    def check_heartbeats(self):
        """Check for missing heartbeats and log warnings"""
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
        """Publish current status of all monitored nodes"""
        for node_name, info in self.monitored_nodes.items():
            # Create a detailed status message
            status_msg = String()
            
            # Add recharging state to status display
            recharge_status = "RECHARGING" if info.get('is_recharging', False) else ""
            
            status_msg.data = (
                f"{node_name}: "
                f"ACTIVE={info['active']}, "
                f"TASK={info.get('task', 'idle')}, "
                f"STATUS={info.get('content', 'unknown')} "
                f"{recharge_status}"
            )
            self.status_pub.publish(status_msg)
            
            # Log status periodically if debugging is enabled
            if self.debug:
                self.get_logger().debug(status_msg.data)


def main(args=None):
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
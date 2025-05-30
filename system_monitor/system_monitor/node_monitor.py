#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import State, Transition, TransitionEvent
from bsn_interfaces.msg import Event, Status  # Add Status import
from std_msgs.msg import String
import threading
import time


class NodeMonitor(Node):
    """
    Monitors and controls the lifecycle of managed nodes.
    
    Features:
    - Track node states (active, inactive, etc.)
    - Subscribe to heartbeats and events
    - Control lifecycle transitions of managed nodes
    - Detect and handle node failures
    """

    def __init__(self):
        super().__init__('node_monitor')
        self.get_logger().info("Starting Node Monitor")
        
        # Track monitored nodes and their states
        self.monitored_nodes = {}
        
        # Configure monitor parameters
        self.declare_parameter('monitored_nodes', ['thermometer_node'])
        self.declare_parameter('heartbeat_timeout', 5.0)
        self.declare_parameter('task_timeout', 30.0)
        self.declare_parameter('check_interval', 2.0)
        
        # Auto-management parameters
        self.declare_parameter('auto_configure', True)
        self.declare_parameter('auto_activate', True)
        self.declare_parameter('auto_restart', True)
        self.declare_parameter('max_restart_attempts', 3)
        self.declare_parameter('restart_cooldown_period', 60.0)
        self.declare_parameter('recovery_strategy', 'progressive')
        
        # Debugging parameter
        self.declare_parameter('debug_level', False)
        
        # Get parameters
        self.node_list = self.get_parameter('monitored_nodes').value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self.task_timeout = self.get_parameter('task_timeout').value
        self.check_interval = self.get_parameter('check_interval').value
        
        # Auto-management settings
        self.auto_configure = self.get_parameter('auto_configure').value
        self.auto_activate = self.get_parameter('auto_activate').value
        self.auto_restart = self.get_parameter('auto_restart').value
        
        # Debugging flag
        self.debug = self.get_parameter('debug_level').value
        
        # Subscribe to heartbeat events
        self.event_sub = self.create_subscription(
            Event,
            'collect_event',
            self.event_callback,
            10
        )
        
        # Add status subscription
        self.status_sub = self.create_subscription(
            Status,
            'component_status',
            self.status_callback,
            10
        )
        
        # Publisher for monitor status
        self.status_pub = self.create_publisher(
            String,
            'node_monitor/status',
            10
        )
        
        # Initialize state tracking for each node
        for node_name in self.node_list:
            self.monitored_nodes[node_name] = {
                'state': 'unknown',
                'last_heartbeat': time.time(),
                'active': False,
                'task': 'idle',       # Add task tracking
                'content': 'unknown', # Add content tracking
                'clients': {
                    'change_state': self.create_client(ChangeState, f'/{node_name}/change_state'),
                    'get_state': self.create_client(GetState, f'/{node_name}/get_state')
                }
            }
        
        # Start state polling timer
        self.create_timer(2.0, self.check_nodes_state)
        
        # Start heartbeat monitoring timer  
        self.create_timer(1.0, self.check_heartbeats)
        
        # Add a timer to check for tasks running too long
        self.create_timer(5.0, self.check_task_timeouts)
        
        self.get_logger().info(f"Monitoring {len(self.node_list)} nodes: {self.node_list}")

        # Add a initialization trigger for auto-configure/activate
        if self.auto_configure:
            # Wait longer before initializing to ensure services are available
            self.get_logger().info("Auto-configure enabled, will initialize nodes in 10 seconds...")
            self.create_timer(10.0, self.initialize_nodes)

    def event_callback(self, msg):
        """Process heartbeat and event messages"""
        node_name = msg.source
        content = msg.content
        
        # Update node in tracked list if it's one we care about
        for monitored_node in self.monitored_nodes:
            if node_name == monitored_node or node_name.endswith(monitored_node.split('/')[-1]):
                self.get_logger().debug(f"Event from {node_name}: {content}")
                self.monitored_nodes[monitored_node]['last_heartbeat'] = time.time()
                self.monitored_nodes[monitored_node]['active'] = (content == "activate")
                break

    def status_callback(self, msg):
        """Process status messages"""
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
                elif content == "deactivated":
                    self.monitored_nodes[monitored_node]['active'] = False
                
                # Implement automated recovery if needed
                if task == "collect" and self.monitored_nodes[monitored_node].get('task_start_time', None) is None:
                    # Start timing the task
                    self.monitored_nodes[monitored_node]['task_start_time'] = time.time()
                    
                elif task == "idle" and self.monitored_nodes[monitored_node].get('task_start_time', None) is not None:
                    # Task completed, clear timing
                    self.monitored_nodes[monitored_node]['task_start_time'] = None
                    
                break

    def check_heartbeats(self):
        """Check for missing heartbeats and take action"""
        current_time = time.time()
        
        for node_name, info in self.monitored_nodes.items():
            time_since_last = current_time - info['last_heartbeat']
            
            # Only log warning if node should be active
            if time_since_last > self.heartbeat_timeout:
                if info['active']:
                    self.get_logger().warn(f"Node {node_name} hasn't sent a heartbeat in {time_since_last:.1f} seconds!")
                    
                    # Don't restart immediately, give it more time
                    if time_since_last > self.heartbeat_timeout * 3:  # Allow 3x timeout before action
                        self.get_logger().error(f"Node {node_name} heartbeat missing for too long, attempting restart")
                        restart_success = self.restart_node(node_name)
                        if restart_success:
                            # Reset the heartbeat time to avoid immediate restart again
                            self.monitored_nodes[node_name]['last_heartbeat'] = current_time

    def check_nodes_state(self):
        """Poll each node's lifecycle state"""
        for node_name, info in self.monitored_nodes.items():
            # Get the current state
            client = info['clients']['get_state']
            if client.service_is_ready():
                req = GetState.Request()
                try:
                    # Use synchronous call to avoid race conditions
                    response = client.call(req)
                    if response is not None:
                        state_id = response.current_state.id
                        label = response.current_state.label
                        
                        # Update tracked state
                        old_state = self.monitored_nodes[node_name]['state']
                        self.monitored_nodes[node_name]['state'] = label
                        
                        # Log state change
                        if old_state != label:
                            self.get_logger().info(f"Node {node_name} state: {label} (id={state_id})")
                except Exception as e:
                    self.get_logger().error(f"Error getting state for {node_name}: {e}")
            else:
                if self.debug:
                    self.get_logger().warn(f"GetState service not available for {node_name}")
            
            # Publish status regardless of whether state update succeeded
            status_msg = String()
            status_msg.data = (
                f"{node_name}: "
                f"STATE={info['state']}, "
                f"ACTIVE={info['active']}, "
                f"TASK={info.get('task', 'idle')}, "
                f"STATUS={info.get('content', 'unknown')}"
            )
            self.status_pub.publish(status_msg)

    def get_state(self, node_name):
        """Get current state of a monitored node"""
        client = self.monitored_nodes[node_name]['clients']['get_state']
        
        if not client.service_is_ready():
            self.get_logger().warn(f"GetState service not available for {node_name}")
            self.monitored_nodes[node_name]['state'] = 'unknown'
            return
            
        req = GetState.Request()
        future = client.call_async(req)
        
        # Add a callback to process the result when it's ready
        future.add_done_callback(lambda f: self.state_callback(f, node_name))

    def state_callback(self, future, node_name):
        """Process the result of the get_state service call"""
        try:
            response = future.result()
            if response is not None:
                state_id = response.current_state.id
                label = response.current_state.label
                
                # Update tracked state
                old_state = self.monitored_nodes[node_name]['state']
                self.monitored_nodes[node_name]['state'] = label
                
                # Log state change
                if old_state != label:
                    self.get_logger().info(f"Node {node_name} state: {label} (id={state_id})")
                    
                    # Debugging output
                    if self.debug:
                        self.get_logger().info(f"Detailed state for {node_name}: {self.monitored_nodes[node_name]}")
        except Exception as e:
            self.get_logger().error(f"Failed to get state for {node_name}: {e}")

    def change_state(self, node_name, transition_id):
        """Request a state change for a node"""
        client = self.monitored_nodes[node_name]['clients']['change_state']
        
        if not client.service_is_ready():
            self.get_logger().error(f"ChangeState service not available for {node_name}")
            return False
            
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)
        
        # Add a callback to process the result when it's ready
        future.add_done_callback(lambda f: self.change_state_callback(f, node_name, transition_id))
        return True

    def change_state_callback(self, future, node_name, transition_id):
        """Process the result of the change_state service call"""
        try:
            response = future.result()
            if response is not None and response.success:
                # Map transition IDs to human-readable names for logging
                transition_names = {
                    Transition.TRANSITION_CREATE: "create",
                    Transition.TRANSITION_CONFIGURE: "configure",
                    Transition.TRANSITION_ACTIVATE: "activate", 
                    Transition.TRANSITION_DEACTIVATE: "deactivate",
                    Transition.TRANSITION_CLEANUP: "cleanup",
                    Transition.TRANSITION_DESTROY: "destroy",
                    Transition.TRANSITION_UNCONFIGURED_SHUTDOWN: "unconfigured_shutdown",
                    Transition.TRANSITION_INACTIVE_SHUTDOWN: "inactive_shutdown",
                    Transition.TRANSITION_ACTIVE_SHUTDOWN: "active_shutdown"
                }
                transition_name = transition_names.get(transition_id, f"unknown({transition_id})")
                self.get_logger().info(f"Node {node_name} successfully transitioned: {transition_name}")
                return True
            else:
                self.get_logger().error(f"Failed to change state for {node_name}")
                return False
        except Exception as e:
            self.get_logger().error(f"Error changing state for {node_name}: {e}")
            return False

    def configure_node(self, node_name):
        """Configure a node"""
        return self.change_state(node_name, Transition.TRANSITION_CONFIGURE)

    def activate_node(self, node_name):
        """Activate a node"""
        return self.change_state(node_name, Transition.TRANSITION_ACTIVATE)

    def deactivate_node(self, node_name):
        """Deactivate a node"""
        return self.change_state(node_name, Transition.TRANSITION_DEACTIVATE)

    def cleanup_node(self, node_name):
        """Clean up a node"""
        return self.change_state(node_name, Transition.TRANSITION_CLEANUP)

    def shutdown_node(self, node_name):
        """Shutdown a node"""
        return self.change_state(node_name, Transition.TRANSITION_INACTIVE_SHUTDOWN)

    def restart_node(self, node_name):
        """Restart a node by cycling through deactivate → cleanup → configure → activate"""
        self.get_logger().info(f"Attempting to restart {node_name}")
        
        # First deactivate the node
        if self.deactivate_node(node_name):
            # Wait briefly for deactivation
            time.sleep(1.0)
            
            # Then cleanup
            if self.cleanup_node(node_name):
                time.sleep(1.0)
                
                # Then reconfigure
                if self.configure_node(node_name):
                    time.sleep(1.0)
                    
                    # Finally reactivate
                    return self.activate_node(node_name)
        
        self.get_logger().error(f"Failed to restart {node_name}")
        return False

    def check_task_timeouts(self):
        """Check for tasks that have been running too long"""
        current_time = time.time()
        
        for node_name, info in self.monitored_nodes.items():
            # Skip if no task is running or no start time recorded
            if info.get('task', 'idle') == 'idle' or info.get('task_start_time', None) is None:
                continue
                
            task_duration = current_time - info['task_start_time']
            
            # If task has been running for more than 30 seconds, consider it stuck
            if task_duration > 30.0:
                self.get_logger().warn(f"Node {node_name} has been in {info['task']} task for {task_duration:.1f}s, restarting node")
                self.restart_node(node_name)

    def list_nodes(self):
        """List all monitored nodes and their states"""
        print("Monitored Nodes:")
        print("=" * 80)
        print(f"{'Node Name':<30} {'State':<15} {'Active':<8} {'Status':<15} {'Task':<10}")
        print("-" * 80)
        for node_name, info in self.monitor.monitored_nodes.items():
            print(f"{node_name:<30} "
                  f"{info['state']:<15} "
                  f"{str(info['active']):<8} "
                  f"{info.get('content', 'unknown'):<15} "
                  f"{info.get('task', 'idle'):<10}")

    def initialize_nodes(self):
        """Initialize all monitored nodes (configure/activate)"""
        self.get_logger().info("Starting automatic node initialization...")
        
        for node_name in self.monitored_nodes:
            # First check if the node is already configured/activated
            self.get_state(node_name)
            time.sleep(1.0)  # Increased pause to ensure state is received
            
            # Check current state and take appropriate action
            current_state = self.monitored_nodes[node_name]['state']
            self.get_logger().info(f"Node {node_name} current state: {current_state}")
            
            if current_state == 'unknown':
                self.get_logger().warn(f"Could not determine state of {node_name}, skipping initialization")
            elif current_state == 'unconfigured':
                self.get_logger().info(f"Auto-configuring node {node_name}")
                if self.configure_node(node_name):
                    time.sleep(1.0)
                    if self.auto_activate:
                        self.get_logger().info(f"Auto-activating node {node_name}")
                        self.activate_node(node_name)
            elif current_state == 'inactive':
                if self.auto_activate:
                    self.get_logger().info(f"Auto-activating node {node_name}")
                    self.activate_node(node_name)
            elif current_state == 'active':
                self.get_logger().info(f"Node {node_name} is already active, no action needed")
            else:
                self.get_logger().warn(f"Node {node_name} is in state '{current_state}', not sure how to initialize")


def main(args=None):
    rclpy.init(args=args)
    node = NodeMonitor()
    
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
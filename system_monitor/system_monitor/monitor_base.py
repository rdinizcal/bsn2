import rclpy
from rclpy.node import Node

class MonitorBase(Node):
    """Base class for system monitors."""
    
    def __init__(self, node_name, MONITOR_TYPE):
        super().__init__(node_name)
        
        self.MONITOR_TYPE = MONITOR_TYPE
        
        # Common parameters
        self.declare_parameter('monitor_frequency', 1.0)  # Hz
        self.declare_parameter('monitored_nodes', [])
        
        # Get parameters
        freq_param = self.get_parameter('monitor_frequency').value
        self.frequency = float(freq_param) if freq_param is not None else 1.0
        self.monitored_nodes = self.get_parameter('monitored_nodes').value
        
        # Create timer for regular monitoring
        self.timer = self.create_timer(1.0 / self.frequency, self.monitor_callback)
        
        self.get_logger().info(f"{self.MONITOR_TYPE} Monitor initialized")
    
    def monitor_callback(self):
        """Main monitoring callback, to be implemented by derived classes."""
        raise NotImplementedError("Derived classes must implement monitor_callback")
# /ros_ws/src/adaptation/adaptation/components/base_engine.py
import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import Strategy, Exception, EnergyStatus
from bsn_interfaces.srv import DataAccessRequest, EngineRequest
from threading import Lock
import re

class BaseEngine(Node):
    """Base class for adaptation engines with common functionality"""
    
    def __init__(self, node_name, qos_attribute):
        super().__init__(node_name)
        
        # Common parameters
        self.declare_parameter('qos_attribute', qos_attribute)
        self.declare_parameter('info_quant', 5)
        self.declare_parameter('monitor_freq', 1.0)
        self.declare_parameter('actuation_freq', 1.0)
        self.declare_parameter('setpoint', 0.2)
        self.declare_parameter('offset', 0.05)
        self.declare_parameter('gain', 0.2)
        self.declare_parameter('tolerance', 0.02)
        
        # Get parameters
        self.qos_attribute = self.get_parameter('qos_attribute').value
        self.info_quant = self.get_parameter('info_quant').value
        self.monitor_freq = self.get_parameter('monitor_freq').value
        self.actuation_freq = self.get_parameter('actuation_freq').value
        self.setpoint = self.get_parameter('setpoint').value
        self.offset = self.get_parameter('offset').value
        self.gain = self.get_parameter('gain').value
        self.tolerance = self.get_parameter('tolerance').value
        
        # Internal state
        self.cycles = 0
        self.target_system_model = ""
        self.strategy = {}
        self.priority = {}
        self.deactivated_components = {}
        self.lock = Lock()
        
        # Initialize components
        self.communication_manager = None
        self.formula_manager = None
        self.data_manager = None
        self.adaptation_planner = None
        
    def initialize_components(self):
        """Initialize all component managers - to be called by subclasses"""
        from .communication_manager import CommunicationManager
        from .formula_manager import FormulaManager
        from .data_manager import DataManager
        from .adaptation_planner import AdaptationPlanner
        
        self.communication_manager = CommunicationManager(self)
        self.formula_manager = FormulaManager(self)
        self.data_manager = DataManager(self)
        self.adaptation_planner = AdaptationPlanner(self)
        
    def start_monitoring(self):
        """Start the main monitoring cycle"""
        self.create_timer(1.0/self.monitor_freq, self.monitor_cycle)
        
    def monitor_cycle(self):
        """Main monitoring cycle - template method"""
        with self.lock:
            self.cycles += 1
            
            # Periodically refresh formula
            if self.cycles % (self.monitor_freq * 10) == 0:
                self.formula_manager.fetch_and_setup_formula()
            
            # Execute MAPE cycle
            self.monitor()
            
    def monitor(self):
        """Monitor phase - delegates to data_manager"""
        self.data_manager.collect_data()
        self.analyze()
        
    def analyze(self):
        """Analyze phase - to be implemented by subclasses"""
        raise NotImplementedError("Subclasses must implement analyze()")
        
    def plan(self, current_value, error):
        """Plan phase - delegates to adaptation_planner"""
        self.adaptation_planner.plan_adaptation(current_value, error)
        
    def execute(self):
        """Execute phase - delegates to communication_manager"""
        self.communication_manager.publish_strategy()
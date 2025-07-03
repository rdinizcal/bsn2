import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import Strategy, Exception, EnergyStatus
from bsn_interfaces.srv import DataAccessRequest, EngineRequest
import time
import math
import re
from threading import Lock
from collections import OrderedDict

class ReliabilityEngine(Node):
    """
    Reliability adaptation engine for BSN system in ROS2
    
    Monitors component reliability through DataAccess
    Calculates system-wide reliability based on formula
    Plans reliability-based adaptations for components
    Sends strategies to Enactor for execution
    """

    def __init__(self):
        super().__init__('reliability_engine')
        self.get_logger().info("Starting ReliabilityEngine")
        
        # Declare parameters
        self.declare_parameter('qos_attribute', 'reliability')
        self.declare_parameter('info_quant', 5)  # Amount of information to fetch
        self.declare_parameter('monitor_freq', 1.0)  # Monitoring frequency in Hz
        self.declare_parameter('actuation_freq', 1.0)  # Actuation frequency in Hz
        self.declare_parameter('setpoint', 0.95)  # Target reliability
        self.declare_parameter('offset', 0.05)  # Initial step for adjustments
        self.declare_parameter('gain', 0.2)  # Gain for control
        self.declare_parameter('tolerance', 0.02)  # Stability margin
        
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
        self.prefix = "R_"  # Prefix for reliability terms
        self.cycles = 0
        self.target_system_model = ""  # Formula string
        self.strategy = {}  # Component reliability targets
        self.priority = {}  # Component priorities
        self.deactivated_components = {}  # Track deactivated components
        
        # Thread synchronization
        self.lock = Lock()
        
        # Create publishers
        self.strategy_pub = self.create_publisher(
            Strategy, 'strategy', 10)
        
        # Create services
        self.engine_service = self.create_service(
            EngineRequest, 'EngineRequest', self.send_adaptation_parameter)
        
        # Create subscribers
        self.exception_sub = self.create_subscription(
            Exception, 'exception', self.receive_exception, 1000)
        
        # Create clients for services
        self.data_access_client = self.create_client(
            DataAccessRequest, 'DataAccessRequest')
        
        # Wait for service availability
        while not self.data_access_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('DataAccessRequest service not available, waiting...')
        
        # Initialize
        self.fetch_and_setup_formula()
        
        # Main processing timer
        self.create_timer(1.0/self.monitor_freq, self.monitor_cycle)
        
        self.get_logger().info(f"ReliabilityEngine initialized with setpoint {self.setpoint}")
    
    def fetch_and_setup_formula(self):
        """Fetch formula from DataAccess and set up model"""
        formula_str = self.fetch_formula('reliability')
        if not formula_str:
            self.get_logger().warn("Empty formula received, retrying...")
            return False
        
        self.setup_formula(formula_str)
        return True
    
    def fetch_formula(self, name):
        """Fetch formula from DataAccess"""
        request = DataAccessRequest.Request()
        request.name = '/engine'
        request.query = f"{name}_formula"
        
        future = self.data_access_client.call_async(request)
        
        # Wait for response (synchronous in this case)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response is None or response.content == "":
            self.get_logger().error("Empty formula string received")
            return ""
            
        return response.content
    
    def setup_formula(self, formula_str):
        """Set up model, strategy and priority based on formula"""
        self.get_logger().info(f"Setting up formula: {formula_str}")
        
        # Store formula string
        self.target_system_model = formula_str
        
        # Extract terms from formula
        terms = self.extract_terms(formula_str)
        
        # Initialize strategy and priority
        self.strategy = self.initialize_strategy(terms)
        self.priority = self.initialize_priority(terms)
        
        # Initialize deactivated components
        for term in terms:
            if term.startswith(self.prefix):
                self.deactivated_components[term] = 0
    
    def extract_terms(self, formula_str):
        """Extract terms from formula string"""
        # Simple pattern matching to find terms like R_G3_T1_1, CTX_G3_T1_1, etc.
        pattern = r'[A-Z]+_[A-Z0-9_]+'
        terms = set(re.findall(pattern, formula_str))
        return list(terms)
    
    def initialize_strategy(self, terms):
        """Initialize strategy with default values"""
        strategy = {}
        
        for term in terms:
            if term.startswith(self.prefix):
                # Default reliability is 1.0
                strategy[term] = 1.0
            elif term.startswith("CTX_"):
                # Default context is inactive (0)
                strategy[term] = 0.0
            elif term.startswith("F_"):
                # Default factor is 1.0
                strategy[term] = 1.0
        
        return strategy
    
    def initialize_priority(self, terms):
        """Initialize priority for terms"""
        priority = {}
        
        for term in terms:
            if term.startswith(self.prefix):
                # Default priority is 50 (middle)
                priority[term[2:]] = 50  # Remove R_ prefix for priority map
        
        return priority
    
    def receive_exception(self, msg):
        """Process exception messages from components"""
        with self.lock:
            content = msg.content
            parts = content.split('=')
            
            if len(parts) != 2:
                self.get_logger().error(f"Invalid exception format: {content}")
                return
                
            component = parts[0]  # e.g., "/g3t1_1"
            value = int(parts[1])  # Priority adjustment
            
            # Format component name for priority map (G3_T1_1)
            component = component.upper()  # Convert to uppercase
            if component.startswith('/'):
                component = component[1:]  # Remove leading slash
            
            # Insert underscore between G and T (G3T1_1 -> G3_T1_1)
            if 'T' in component and '_T' not in component:
                t_pos = component.find('T')
                component = component[:t_pos] + '_' + component[t_pos:]
            
            # Check if component exists in priority map
            if component in self.priority:
                # Adjust priority
                self.priority[component] += value
                
                # Constrain priority
                if self.priority[component] > 99:
                    self.priority[component] = 100
                if self.priority[component] < 1:
                    self.priority[component] = 0
                    
                self.get_logger().info(f"Updated priority for {component}: {self.priority[component]}")
            else:
                self.get_logger().error(f"Component not found in priority map: {component}")
    
    def monitor_cycle(self):
        """Main monitoring cycle"""
        with self.lock:
            self.cycles += 1
            
            # Periodically refresh formula
            if self.cycles % (self.monitor_freq * 10) == 0:
                self.fetch_and_setup_formula()
            
            # Execute monitor-analyze-plan-execute cycle
            self.monitor()
    
    def monitor(self):
        """Collect reliability and context data"""
        # Reset temporary values in formula
        for key in self.strategy:
            if key.startswith("CTX_"):
                self.strategy[key] = 0.0
            if key.startswith(self.prefix):
                self.strategy[key] = 1.0
            if key.startswith("F_"):
                self.strategy[key] = 1.0
        
        # Get reliability data for all components
        request = DataAccessRequest.Request()
        request.name = '/engine'
        request.query = f"all:reliability:{self.info_quant}"
        
        future = self.data_access_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response is None or response.content == "":
            self.get_logger().error("Empty reliability data received")
            return
            
        # Parse reliability data (format: "/g3t1_1:0.95;/g3t1_2:0.92;...")
        for pair in response.content.split(';'):
            if not pair:
                continue
                
            parts = pair.split(':')
            if len(parts) != 2:
                continue
                
            component = parts[0]  # e.g., "/g3t1_1"
            reliability = float(parts[1])  # e.g., 0.95
            
            # Format component name for strategy map
            component = component.upper()  # Convert to uppercase
            if component.startswith('/'):
                component = component[1:]  # Remove leading slash
            
            # Insert underscore between G and T (G3T1_1 -> G3_T1_1)
            if 'T' in component and '_T' not in component:
                t_pos = component.find('T')
                component = component[:t_pos] + '_' + component[t_pos:]
            
            # Update strategy
            key = f"{self.prefix}{component}"
            if key in self.strategy:
                self.strategy[key] = reliability
                self.get_logger().debug(f"Updated {key} = {reliability}")
        
        # Get context data (activation status)
        request.query = "all:event:1"
        
        future = self.data_access_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response is None or response.content == "":
            self.get_logger().error("Empty context data received")
            return
            
        # Parse context data (format: "/g3t1_1:activate;/g3t1_2:deactivate;...")
        for pair in response.content.split(';'):
            if not pair:
                continue
                
            parts = pair.split(':')
            if len(parts) != 2:
                continue
                
            component = parts[0]  # e.g., "/g3t1_1"
            status = parts[1]  # e.g., "activate"
            
            # Format component name for strategy map
            component = component.upper()  # Convert to uppercase
            if component.startswith('/'):
                component = component[1:]  # Remove leading slash
            
            # Insert underscore between G and T (G3T1_1 -> G3_T1_1)
            if 'T' in component and '_T' not in component:
                t_pos = component.find('T')
                component = component[:t_pos] + '_' + component[t_pos:]
            
            if component != "G4_T1":  # Handle normal components
                self.strategy[f"CTX_{component}"] = 1.0  # Context is active
                
                if status == "deactivate":
                    # Deactivated components get perfect reliability to avoid affecting formula
                    self.strategy[f"{self.prefix}{component}"] = 1.0
                    self.deactivated_components[f"{self.prefix}{component}"] = 1
                    self.get_logger().warn(f"{component} was deactivated, setting reliability to 1.0")
            else:  # Special case for central hub
                if status == "activate":
                    self.strategy[f"CTX_{component}"] = 1.0
                    self.deactivated_components[f"{self.prefix}{component}"] = 0
                else:  # deactivate
                    self.strategy[f"CTX_{component}"] = 0.0
                    self.deactivated_components[f"{self.prefix}{component}"] = 1
        
        # Continue to analysis
        self.analyze()
    
    def analyze(self):
        """Analyze current reliability against target"""
        # Calculate current system reliability
        r_curr = self.calculate_qos()
        self.get_logger().info(f"Current system reliability: {r_curr:.4f}, target: {self.setpoint:.4f}")
        
        # Calculate error
        error = self.setpoint - r_curr
        
        # Check if error is outside stability margin
        if abs(error) > self.setpoint * self.tolerance:
            # Only plan when it's time to actuate
            if self.cycles >= self.monitor_freq / self.actuation_freq:
                self.cycles = 0
                self.plan(r_curr, error)
    
    def plan(self, r_curr, error):
        """Plan adaptation strategy to achieve target reliability"""
        self.get_logger().info(f"Planning adaptation for error: {error:.4f}")
        
        # Collect components that can be adjusted
        r_vec = []
        for key in self.strategy:
            if key.startswith(self.prefix):
                task = key[2:]  # Remove R_ prefix
                
                # Only include active components
                if (self.strategy.get(f"CTX_{task}", 0) != 0 and 
                    self.strategy.get(f"F_{task}", 0) != 0 and
                    self.deactivated_components.get(key, 0) == 0):
                    r_vec.append(key)
        
        # Sort by priority (higher priority first)
        r_vec_with_priority = [(component, self.priority.get(component[2:], 0)) for component in r_vec]
        r_vec_with_priority.sort(key=lambda x: x[1], reverse=True)
        r_vec = [component for component, _ in r_vec_with_priority]
        
        # No components to adjust
        if not r_vec:
            self.get_logger().warn("No components available for adaptation")
            return
        
        # List to store potential solutions
        solutions = []
        
        # Try adapting components one at a time, starting with highest priority
        for component in r_vec:
            # Save original strategy
            original_strategy = self.strategy.copy()
            
            # Apply initial offset to all components
            for c in r_vec:
                if error > 0:  # Need to increase reliability
                    self.strategy[c] = r_curr * (1 - self.offset)
                else:  # Need to decrease reliability
                    self.strategy[c] = min(1.0, r_curr * (1 + self.offset))
            
            # Calculate new reliability with offset
            r_new = self.calculate_qos()
            
            # Save previous state for iteration
            prev_strategy = self.strategy.copy()
            r_prev = 0
            
            # Adjust current component
            if error > 0:  # Need to increase reliability
                # Keep increasing reliability while improving and not at maximum
                while r_new < self.setpoint and r_prev < r_new and self.strategy[component] > 0 and self.strategy[component] < 1:
                    prev_strategy = self.strategy.copy()
                    r_prev = r_new
                    self.strategy[component] += self.gain * error
                    if self.strategy[component] > 1.0:
                        self.strategy[component] = 1.0
                    r_new = self.calculate_qos()
            else:  # Need to decrease reliability
                # Keep decreasing reliability while improving and not at minimum
                while r_new > self.setpoint and r_prev > r_new and self.strategy[component] > 0 and self.strategy[component] < 1:
                    prev_strategy = self.strategy.copy()
                    r_prev = r_new
                    self.strategy[component] += self.gain * error  # Will be negative
                    if self.strategy[component] < 0.0:
                        self.strategy[component] = 0.0
                    r_new = self.calculate_qos()
            
            # Restore best result from loop
            self.strategy = prev_strategy
            r_new = self.calculate_qos()
            
            # Try adjusting other components too
            for other_component in r_vec:
                if other_component == component:
                    continue
                
                if error > 0:  # Need to increase reliability
                    while r_new < self.setpoint and r_prev < r_new and self.strategy[other_component] > 0 and self.strategy[other_component] < 1:
                        prev_strategy = self.strategy.copy()
                        r_prev = r_new
                        self.strategy[other_component] += self.gain * error
                        if self.strategy[other_component] > 1.0:
                            self.strategy[other_component] = 1.0
                        r_new = self.calculate_qos()
                else:  # Need to decrease reliability
                    while r_new > self.setpoint and r_prev > r_new and self.strategy[other_component] > 0 and self.strategy[other_component] < 1:
                        prev_strategy = self.strategy.copy()
                        r_prev = r_new
                        self.strategy[other_component] += self.gain * error
                        if self.strategy[other_component] < 0.0:
                            self.strategy[other_component] = 0.0
                        r_new = self.calculate_qos()
            
            # Restore best result
            self.strategy = prev_strategy
            r_new = self.calculate_qos()
            
            # Add this solution to list
            solutions.append((self.strategy.copy(), r_new))
            
            # Restore original strategy for next iteration
            self.strategy = original_strategy
        
        # Choose the best solution (closest to setpoint)
        best_solution = None
        best_distance = float('inf')
        
        for solution, r_value in solutions:
            distance = abs(r_value - self.setpoint)
            if distance < best_distance and self.setpoint * (1 - self.tolerance) <= r_value <= self.setpoint * (1 + self.tolerance):
                best_distance = distance
                best_solution = solution
        
        if best_solution:
            # Apply best solution
            self.strategy = best_solution
            self.execute()
        else:
            self.get_logger().warn("Could not find a suitable adaptation")
    
    def execute(self):
        """Execute adaptation by publishing strategy"""
        # Format strategy content
        content = ""
        
        # Group by component base name
        components = {}
        
        for key, value in self.strategy.items():
            if key.startswith(self.prefix):  # Only include reliability terms
                # Convert R_G3_T1_1 to g3t1_1:0.95
                component_name = key[2:]  # Remove R_ prefix
                component_name = component_name.lower()
                
                # Remove underscore between g and t (g3_t1_1 -> g3t1_1)
                parts = component_name.split('_')
                if len(parts) >= 2:
                    component_name = parts[0] + parts[1]
                    if len(parts) > 2:
                        component_name += "_" + "_".join(parts[2:])
                
                components[component_name] = value
        
        # Format as /g3t1_1:0.95;/g3t1_2:0.92;...
        for component, value in components.items():
            content += f"/{component}:{value:.6f};"
        
        # Remove trailing semicolon
        if content.endswith(";"):
            content = content[:-1]
        
        # Create and publish strategy message
        msg = Strategy()
        msg.source = "/engine"
        msg.target = "/enactor"
        msg.content = content
        
        self.strategy_pub.publish(msg)
        self.get_logger().info(f"Published strategy: {content}")
    
    def calculate_qos(self):
        """Calculate QoS based on formula and strategy"""
        # In a real implementation, you'd use a formula parser like Lepton
        # For this example, we'll use a simplified approach
        
        # Calculate weighted average of all reliability values
        total_weight = 0
        weighted_sum = 0
        
        for key, value in self.strategy.items():
            if key.startswith(self.prefix):
                component = key[2:]  # Remove R_ prefix
                
                # Only include active components
                if self.strategy.get(f"CTX_{component}", 0) != 0:
                    weight = 1  # Equal weight for simplicity
                    weighted_sum += value * weight
                    total_weight += weight
        
        if total_weight == 0:
            return 1.0  # Default to perfect reliability if no components
            
        return weighted_sum / total_weight
    
    def send_adaptation_parameter(self, request, response):
        """Provide adaptation parameter to enactor"""
        response.content = self.qos_attribute
        return response
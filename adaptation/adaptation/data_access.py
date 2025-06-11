#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import Persist, Status, Event, EnergyStatus, AdaptationCommand, Uncertainty, TargetSystemData
from bsn_interfaces.srv import DataAccessRequest
import time
import os
from datetime import datetime
from collections import deque
import csv
from threading import Lock
import math

class DataAccess(Node):
    """
    DataAccess component for BSN system in ROS2
    
    Stores and provides access to system monitoring data
    Receives messages from Logger via persist topic
    Provides query interface for adaptation engines and enactor
    Calculates component reliability and cost
    """

    def __init__(self):
        super().__init__('data_access')
        self.get_logger().info("Starting DataAccess")
        
        # Declare parameters
        self.declare_parameter('frequency', 1.0)  # Calculation frequency
        self.declare_parameter('buffer_size', 1000)  # Max entries per component
        self.declare_parameter('log_path', '')  # Path for log files
        self.declare_parameter('reliability_formula', 'successes/(successes+failures)')
        self.declare_parameter('cost_formula', 'battery_consumption')
        self.declare_parameter('flush_frequency', 30)  # Records before flushing to disk
        
        # Get parameters
        self.frequency = self.get_parameter('frequency').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.log_path = self.get_parameter('log_path').value
        self.reliability_formula = self.get_parameter('reliability_formula').value
        self.cost_formula = self.get_parameter('cost_formula').value
        self.flush_frequency = self.get_parameter('flush_frequency').value
        
        # Initialize log paths
        if not self.log_path:
            # Use default path
            self.log_path = os.path.join(os.getcwd(), 'logs')
            
        # Create log directory if it doesn't exist
        os.makedirs(self.log_path, exist_ok=True)
            
        # Generate unique log filenames with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.event_filepath = os.path.join(self.log_path, f'event_{timestamp}.log')
        self.status_filepath = os.path.join(self.log_path, f'status_{timestamp}.log')
        self.energy_status_filepath = os.path.join(self.log_path, f'energy_status_{timestamp}.log')
        self.uncertainty_filepath = os.path.join(self.log_path, f'uncertainty_{timestamp}.log')
        self.adaptation_filepath = os.path.join(self.log_path, f'adaptation_{timestamp}.log')
        
        # Create and initialize log files
        for filepath in [self.event_filepath, self.status_filepath, 
                        self.energy_status_filepath, self.uncertainty_filepath, 
                        self.adaptation_filepath]:
            with open(filepath, 'w') as f:
                f.write('\n')
        
        # Internal state tracking
        self.logical_clock = 0
        self.arrived_status = 0
        self.count_to_calc_and_reset = 0
        self.count_to_fetch = 0
        self.time_ref = self.get_clock().now().nanoseconds
        
        # Thread synchronization
        self.lock = Lock()
        
        # Storage structures
        self.status_vec = []  # For periodic flush to disk
        self.energy_status_vec = []
        self.event_vec = []
        self.uncertainty_vec = []
        self.adaptation_vec = []
        
        # In-memory storage for queries
        self.status = {}  # {component: deque([(timestamp, status), ...])}
        self.events = {}  # {component: deque([event, ...])}
        
        # Component state tracking
        self.components_reliabilities = {}
        self.components_batteries = {
            "g3t1_1": 100, "g3t1_2": 100, "g3t1_3": 100,
            "g3t1_4": 100, "g3t1_5": 100, "g3t1_6": 100
        }
        self.components_costs_engine = {
            "g3t1_1": 0, "g3t1_2": 0, "g3t1_3": 0,
            "g3t1_4": 0, "g3t1_5": 0, "g3t1_6": 0
        }
        self.components_costs_enactor = {
            "g3t1_1": 0, "g3t1_2": 0, "g3t1_3": 0,
            "g3t1_4": 0, "g3t1_5": 0, "g3t1_6": 0
        }
        self.contexts = {}
        
        # Create subscribers
        self.persist_sub = self.create_subscription(
            Persist, 'persist', self.receive_persist_message, 1000)
            
        self.target_system_sub = self.create_subscription(
            TargetSystemData, 'TargetSystemData', self.process_target_system_data, 100)
        
        # Create services
        self.query_srv = self.create_service(
            DataAccessRequest, 'DataAccessRequest', self.process_query)
        
        # Create timers for periodic operations
        self.create_timer(1.0/self.frequency, self.body)
        
        self.get_logger().info("DataAccess initialized and ready for queries")
    
    def now(self):
        """Get current time in milliseconds"""
        return self.get_clock().now().nanoseconds // 1000000
    
    def now_in_seconds(self):
        """Get current time as a timestamp object"""
        return self.get_clock().now()
    
    def body(self):
        """Main processing cycle"""
        self.count_to_fetch += 1
        self.count_to_calc_and_reset += 1
        
        # Calculate reliability periodically
        if self.count_to_calc_and_reset >= self.frequency:
            with self.lock:
                self.apply_time_window()
                for component in self.status:
                    self.calculate_component_reliability(component)
            
            self.count_to_calc_and_reset = 0
        
        # Reload formulas periodically
        if self.count_to_fetch >= self.frequency * 10:
            # In a real system, you might fetch formulas from a file or parameter server
            # Here we just use the parameters
            self.reliability_formula = self.get_parameter('reliability_formula').value
            self.cost_formula = self.get_parameter('cost_formula').value
            
            self.count_to_fetch = 0
    
    def receive_persist_message(self, msg):
        """Process standardized persist messages from Logger"""
        with self.lock:
            self.logical_clock += 1
            
            # Process based on message type
            if msg.type == "Status":
                self.arrived_status += 1
                self.persist_status(msg.timestamp, msg.source, msg.target, msg.content)
                
                # Store status in memory
                if msg.source not in self.status:
                    self.status[msg.source] = deque(maxlen=self.buffer_size)
                
                self.status[msg.source].append((self.now_in_seconds(), msg.content))
                
            elif msg.type == "EnergyStatus":
                self.persist_energy_status(msg.timestamp, msg.source, msg.target, msg.content)
                
                # Update battery status
                try:
                    parts = msg.content.split(':')
                    if len(parts) >= 4:  # Format: "energy:X:cost:Y"
                        battery = float(parts[1])
                        cost = float(parts[3])
                        
                        if msg.source in self.components_batteries:
                            self.components_batteries[msg.source] = battery
                            
                        if msg.source in self.components_costs_enactor:
                            self.components_costs_enactor[msg.source] = cost
                except Exception as e:
                    self.get_logger().error(f"Error parsing energy status: {e}")
                
            elif msg.type == "Event":
                self.persist_event(msg.timestamp, msg.source, msg.target, msg.content)
                
                # Store event in memory
                if msg.source not in self.events:
                    self.events[msg.source] = deque(maxlen=self.buffer_size)
                
                self.events[msg.source].append(msg.content)
                
                # Update context for activations/deactivations
                if msg.content == "activate":
                    self.contexts[msg.source] = 1
                elif msg.content == "deactivate":
                    self.contexts[msg.source] = 0
                
            elif msg.type == "Uncertainty":
                self.persist_uncertainty(msg.timestamp, msg.source, msg.target, msg.content)
                
            elif msg.type == "AdaptationCommand":
                self.persist_adaptation(msg.timestamp, msg.source, msg.target, msg.content)
            
            # Flush to disk periodically
            if self.logical_clock % self.flush_frequency == 0:
                self.flush()
    
    def persist_status(self, timestamp, source, target, content):
        """Add status message to buffer for persistence"""
        status_obj = {
            "name": "Status",
            "logical_clock": self.logical_clock,
            "timestamp": timestamp,
            "source": source,
            "target": target,
            "content": content
        }
        self.status_vec.append(status_obj)
    
    def persist_event(self, timestamp, source, target, content):
        """Add event message to buffer for persistence"""
        event_obj = {
            "name": "Event",
            "logical_clock": self.logical_clock,
            "timestamp": timestamp,
            "source": source,
            "target": target,
            "content": content
        }
        self.event_vec.append(event_obj)
    
    def persist_energy_status(self, timestamp, source, target, content):
        """Add energy status message to buffer for persistence"""
        energy_obj = {
            "name": "EnergyStatus",
            "logical_clock": self.logical_clock,
            "timestamp": timestamp,
            "source": source,
            "target": target,
            "content": content
        }
        self.energy_status_vec.append(energy_obj)
    
    def persist_uncertainty(self, timestamp, source, target, content):
        """Add uncertainty message to buffer for persistence"""
        uncertainty_obj = {
            "name": "Uncertainty",
            "logical_clock": self.logical_clock,
            "timestamp": timestamp,
            "source": source,
            "target": target,
            "content": content
        }
        self.uncertainty_vec.append(uncertainty_obj)
    
    def persist_adaptation(self, timestamp, source, target, content):
        """Add adaptation message to buffer for persistence"""
        adaptation_obj = {
            "name": "Adaptation",
            "logical_clock": self.logical_clock,
            "timestamp": timestamp,
            "source": source,
            "target": target,
            "content": content
        }
        self.adaptation_vec.append(adaptation_obj)
    
    def flush(self):
        """Write buffered messages to disk"""
        self.get_logger().debug(f"Flushing data to disk")
        
        try:
            # Write status messages
            if self.status_vec:
                with open(self.status_filepath, 'a') as f:
                    for item in self.status_vec:
                        f.write(f"{item['name']},{item['logical_clock']},{item['timestamp']},"
                                f"{item['source']},{item['target']},{item['content']}\n")
                self.status_vec = []
            
            # Write energy status messages
            if self.energy_status_vec:
                with open(self.energy_status_filepath, 'a') as f:
                    for item in self.energy_status_vec:
                        # Extract cost value
                        cost = "0"
                        try:
                            parts = item['content'].split(':')
                            if len(parts) >= 4:
                                cost = parts[3]
                        except:
                            pass
                            
                        f.write(f"{item['name']},{item['logical_clock']},{item['timestamp']},"
                                f"{item['source']},{item['target']},{cost}\n")
                self.energy_status_vec = []
            
            # Write event messages
            if self.event_vec:
                with open(self.event_filepath, 'a') as f:
                    for item in self.event_vec:
                        f.write(f"{item['name']},{item['logical_clock']},{item['timestamp']},"
                                f"{item['source']},{item['target']},{item['content']}\n")
                self.event_vec = []
            
            # Write uncertainty messages
            if self.uncertainty_vec:
                with open(self.uncertainty_filepath, 'a') as f:
                    for item in self.uncertainty_vec:
                        f.write(f"{item['name']},{item['logical_clock']},{item['timestamp']},"
                                f"{item['source']},{item['target']},{item['content']}\n")
                self.uncertainty_vec = []
            
            # Write adaptation messages
            if self.adaptation_vec:
                with open(self.adaptation_filepath, 'a') as f:
                    for item in self.adaptation_vec:
                        f.write(f"{item['name']},{item['logical_clock']},{item['timestamp']},"
                                f"{item['source']},{item['target']},{item['content']}\n")
                self.adaptation_vec = []
                
        except Exception as e:
            self.get_logger().error(f"Error flushing data: {e}")
    
    def process_target_system_data(self, msg):
        """Process target system data with battery levels"""
        # Update component battery levels
        self.components_batteries["g3t1_1"] = msg.trm_batt
        self.components_batteries["g3t1_2"] = msg.ecg_batt
        self.components_batteries["g3t1_3"] = msg.oxi_batt
        self.components_batteries["g3t1_4"] = msg.abps_batt
        self.components_batteries["g3t1_5"] = msg.abpd_batt
        self.components_batteries["g3t1_6"] = msg.glc_batt
    
    def process_query(self, request, response):
        """Process DataAccessRequest queries from adaptation engines"""
        response.content = ""
        
        try:
            with self.lock:
                if request.name == "/engine" or request.name == "/enactor":
                    # Parse query (format: "all:status:100" or "all:reliability:" or "all:cost:")
                    query_parts = request.query.split(':')
                    
                    # Handle formula queries
                    if len(query_parts) == 1:
                        if query_parts[0] == "reliability_formula":
                            response.content = self.reliability_formula
                        elif query_parts[0] == "cost_formula":
                            response.content = self.cost_formula
                    
                    # Handle status/reliability/cost queries
                    elif len(query_parts) > 1:
                        component = query_parts[0]
                        query_type = query_parts[1]
                        
                        # Components list query (all)
                        if component == "all":
                            if query_type == "reliability":
                                # Apply time window to ensure calculations use recent data
                                self.apply_time_window()
                                for comp in self.status:
                                    response.content += self.calculate_component_reliability(comp)
                                    
                            elif query_type == "cost":
                                # Apply time window for consistent calculations
                                self.apply_time_window()
                                for comp in self.status:
                                    response.content += self.calculate_component_cost(comp, request.name)
                                    
                            elif query_type == "status":
                                # Get status count if specified
                                count = 1
                                if len(query_parts) > 2:
                                    try:
                                        count = int(query_parts[2])
                                    except:
                                        count = 1
                                
                                # Build status response for all components
                                for comp, status_list in self.status.items():
                                    statuses = []
                                    for _, status_content in list(status_list)[-count:]:
                                        statuses.append(status_content)
                                    
                                    if statuses:
                                        response.content += f"{comp}:{','.join(statuses)};"
                                
                                # Remove trailing semicolon
                                if response.content.endswith(';'):
                                    response.content = response.content[:-1]
                                    
                        # Individual component query
                        else:
                            if query_type == "reliability" and component in self.status:
                                response.content = self.calculate_component_reliability(component)
                                
                            elif query_type == "cost" and component in self.status:
                                response.content = self.calculate_component_cost(component, request.name)
                                
                            elif query_type == "status" and component in self.status:
                                # Get status count if specified
                                count = 1
                                if len(query_parts) > 2:
                                    try:
                                        count = int(query_parts[2])
                                    except:
                                        count = 1
                                
                                # Build status response for specific component
                                statuses = []
                                for _, status_content in list(self.status[component])[-count:]:
                                    statuses.append(status_content)
                                
                                if statuses:
                                    response.content = ','.join(statuses)
        
        except Exception as e:
            self.get_logger().error(f"Error processing query: {e}")
        
        return response
    
    def calculate_component_reliability(self, component):
        """Calculate reliability for a component"""
        if component not in self.status:
            return ""
        
        statuses = self.status[component]
        if not statuses:
            return f"{component}:1.0;"
        
        # Count success and failures
        success_count = 0
        failure_count = 0
        
        for _, status in statuses:
            if status == "success":
                success_count += 1
            elif status == "fail":
                failure_count += 1
        
        # Calculate reliability using formula
        reliability = 1.0  # Default perfect reliability
        
        if success_count + failure_count > 0:
            # Simple success rate calculation
            reliability = success_count / (success_count + failure_count)
        
        # Store in reliability map
        self.components_reliabilities[component] = reliability
        
        return f"{component}:{reliability:.6f};"
    
    def calculate_component_cost(self, component, req_name):
        """Calculate cost for a component"""
        if component not in self.components_batteries:
            return ""
        
        # Calculate cost based on battery consumption
        # Higher battery = lower cost, so we invert
        battery_level = self.components_batteries[component]
        cost = 0.0
        
        if battery_level < 100:
            cost = (100 - battery_level) / 100
        
        # Store in appropriate cost map
        if req_name == "/engine":
            self.components_costs_engine[component] = cost
        elif req_name == "/enactor":
            self.components_costs_enactor[component] = cost
        
        return f"{component}:{cost:.6f};"
    
    def apply_time_window(self):
        """Apply time window to status data to limit calculations to recent data"""
        # Get current time
        now = self.now_in_seconds()
        
        # Calculate cutoff time (10 seconds ago)
        cutoff_delta = 10_000_000_000  # 10 seconds in nanoseconds
        
        # Prune old entries from status map
        for component, status_list in self.status.items():
            # Create a new filtered list
            filtered = deque(maxlen=self.buffer_size)
            
            for timestamp, content in status_list:
                # Keep entries that are within the time window
                if (now.nanoseconds - timestamp.nanoseconds) < cutoff_delta:
                    filtered.append((timestamp, content))
            
            # Replace with filtered list
            self.status[component] = filtered


def main(args=None):
    rclpy.init(args=args)
    data_access = DataAccess()
    
    try:
        rclpy.spin(data_access)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure final flush of data before shutdown
        data_access.flush()
        data_access.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
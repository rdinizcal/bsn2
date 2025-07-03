#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
DataAccess component for BSN system - ROS2 Jazzy migration
Knowledge repository for storing and analyzing system data
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import time
import json
import csv
import os
from pathlib import Path
from datetime import datetime, timezone
from typing import Dict, List, Tuple, Optional, Any
from collections import deque, defaultdict
import math

# ROS2 Messages and Services
from bsn_interfaces.msg import Persist, TargetSystemData
from bsn_interfaces.srv import DataAccessRequest
from std_msgs.msg import Header

# BSN Model imports
from adaptation.adaptation.model.formula import Formula, FormulaError
from adaptation.adaptation.goal_model import GoalTree, Goal, Task, LeafTask, Context, Property

class DataMessage:
    """Base class for data messages"""
    
    def __init__(self, msg_type: str, timestamp: int, logical_clock: int, 
                 source: str, target: str, content: str):
        self.msg_type = msg_type
        self.timestamp = timestamp
        self.logical_clock = logical_clock
        self.source = source
        self.target = target
        self.content = content

class StatusMessage(DataMessage):
    """Status message for component operational state"""
    
    def __init__(self, timestamp: int, logical_clock: int, source: str, target: str, content: str):
        super().__init__("Status", timestamp, logical_clock, source, target, content)

class EventMessage(DataMessage):
    """Event message for component lifecycle events"""
    
    def __init__(self, timestamp: int, logical_clock: int, source: str, target: str, content: str):
        super().__init__("Event", timestamp, logical_clock, source, target, content)

class EnergyStatusMessage(DataMessage):
    """Energy status message for component power consumption"""
    
    def __init__(self, timestamp: int, logical_clock: int, source: str, target: str, content: str):
        super().__init__("EnergyStatus", timestamp, logical_clock, source, target, content)

class UncertaintyMessage(DataMessage):
    """Uncertainty message for system uncertainty data"""
    
    def __init__(self, timestamp: int, logical_clock: int, source: str, target: str, content: str):
        super().__init__("Uncertainty", timestamp, logical_clock, source, target, content)

class AdaptationMessage(DataMessage):
    """Adaptation message for adaptation commands"""
    
    def __init__(self, timestamp: int, logical_clock: int, source: str, target: str, content: str):
        super().__init__("Adaptation", timestamp, logical_clock, source, target, content)

class DataAccess(Node):
    """
    DataAccess component - Knowledge repository for BSN system
    
    Responsibilities:
    - Store system messages (Status, Events, Energy, etc.)
    - Calculate component reliability and cost metrics
    - Provide data access services to adaptation engines
    - Manage system goal model and formulas
    """
    
    def __init__(self):
        super().__init__('data_access')
        self.get_logger().info("Starting DataAccess component")
        
        # Initialize parameters
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('buffer_size', 1000)
        self.declare_parameter('time_window', 10.1)
        self.declare_parameter('log_path', '/tmp/bsn_logs')
        self.declare_parameter('models_path', '/tmp/bsn_models')
        self.declare_parameter('flush_interval', 30)
        
        # Get parameters
        self.frequency = self.get_parameter('frequency').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.time_window = self.get_parameter('time_window').value
        self.log_path = self.get_parameter('log_path').value
        self.models_path = self.get_parameter('models_path').value
        self.flush_interval = self.get_parameter('flush_interval').value
        
        # Initialize data structures
        self.logical_clock = 0
        self.count_to_calc_and_reset = 0
        self.count_to_fetch = 0
        self.arrived_status = 0
        
        # Message buffers
        self.status_messages: List[StatusMessage] = []
        self.event_messages: List[EventMessage] = []
        self.energy_messages: List[EnergyStatusMessage] = []
        self.uncertainty_messages: List[UncertaintyMessage] = []
        self.adaptation_messages: List[AdaptationMessage] = []
        
        # Component data storage
        self.status: Dict[str, deque] = defaultdict(deque)
        self.events: Dict[str, deque] = defaultdict(deque)
        self.contexts: Dict[str, int] = {}
        
        # Component metrics
        self.components_reliabilities: Dict[str, float] = {}
        self.components_batteries: Dict[str, float] = {}
        self.components_costs_engine: Dict[str, float] = {}
        self.components_costs_enactor: Dict[str, float] = {}
        
        # Initialize component default values
        self._initialize_component_data()
        
        # Formulas
        self.reliability_formula_text = ""
        self.cost_formula_text = ""
        self.reliability_formula: Optional[Formula] = None
        self.cost_formula: Optional[Formula] = None
        
        # Goal Model
        self.goal_tree: Optional[GoalTree] = None
        
        # File paths
        self.log_files = {}
        
        # Setup ROS2 interfaces
        self._setup_ros_interfaces()
        
        # Setup logging
        self._setup_logging()
        
        # Load formulas and goal model
        self._load_models()
        
        # Create timer for periodic tasks
        self.create_timer(1.0 / self.frequency, self.timer_callback)
        
        self.get_logger().info("DataAccess initialized successfully")
    
    def _initialize_component_data(self):
        """Initialize default component data"""
        components = ["g3t1_1", "g3t1_2", "g3t1_3", "g3t1_4", "g3t1_5", "g3t1_6"]
        
        for component in components:
            self.components_batteries[component] = 100.0
            self.components_costs_engine[component] = 0.0
            self.components_costs_enactor[component] = 0.0
            self.components_reliabilities[component] = 1.0
    
    def _setup_ros_interfaces(self):
        """Setup ROS2 publishers, subscribers, and services"""
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.persist_sub = self.create_subscription(
            Persist,
            'persist',
            self.receive_persist_message,
            qos_profile
        )
        
        self.target_system_sub = self.create_subscription(
            TargetSystemData,
            'TargetSystemData',
            self.process_target_system_data,
            qos_profile
        )
        
        # Service server
        self.data_service = self.create_service(
            DataAccessRequest,
            'DataAccessRequest',
            self.process_query
        )
    
    def _setup_logging(self):
        """Setup log file paths and create directories"""
        # Create log directory
        Path(self.log_path).mkdir(parents=True, exist_ok=True)
        
        # Create timestamp for unique file names
        timestamp = str(int(time.time() * 1000000))
        
        # Define log file paths
        self.log_files = {
            'status': os.path.join(self.log_path, f'status_{timestamp}.log'),
            'event': os.path.join(self.log_path, f'event_{timestamp}.log'),
            'energy': os.path.join(self.log_path, f'energystatus_{timestamp}.log'),
            'uncertainty': os.path.join(self.log_path, f'uncertainty_{timestamp}.log'),
            'adaptation': os.path.join(self.log_path, f'adaptation_{timestamp}.log')
        }
        
        # Initialize log files
        for log_file in self.log_files.values():
            with open(log_file, 'w') as f:
                f.write("")
    
    def _load_models(self):
        """Load formulas and goal model from files"""
        try:
            # Load reliability formula
            reliability_path = os.path.join(self.models_path, 'reliability.formula')
            if os.path.exists(reliability_path):
                with open(reliability_path, 'r') as f:
                    self.reliability_formula_text = f.read().strip()
                    self.reliability_formula = Formula(self.reliability_formula_text)
                    self.get_logger().info("Loaded reliability formula")
            
            # Load cost formula
            cost_path = os.path.join(self.models_path, 'cost.formula')
            if os.path.exists(cost_path):
                with open(cost_path, 'r') as f:
                    self.cost_formula_text = f.read().strip()
                    self.cost_formula = Formula(self.cost_formula_text)
                    self.get_logger().info("Loaded cost formula")
            
            # Load goal model
            goal_model_path = os.path.join(self.models_path, 'goalModel.json')
            if os.path.exists(goal_model_path):
                self._load_goal_model(goal_model_path)
                self.get_logger().info("Loaded goal model")
                
        except Exception as e:
            self.get_logger().error(f"Error loading models: {e}")
    
    def _load_goal_model(self, file_path: str):
        """Load goal model from JSON file"""
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
            
            # Create goal tree
            actor = data.get('actors', [{}])[0].get('text', 'BSN')
            self.goal_tree = GoalTree(actor)
            
            # Parse nodes and create goal hierarchy
            # This is a simplified version - you may need to adapt based on your JSON structure
            nodes_data = data.get('actors', [{}])[0].get('nodes', [])
            
            # Create nodes
            nodes = {}
            for node_data in nodes_data:
                node_id = node_data.get('id', '')
                text = node_data.get('text', '')
                node_type = node_data.get('type', '')
                
                if 'Goal' in node_type:
                    nodes[node_id] = Goal(node_id, text)
                elif 'Task' in node_type:
                    if 'T1.' in text:  # Leaf tasks
                        # Create leaf task with default properties
                        context = Context(f"CTX_{node_id}", f"{text} available", True)
                        cost = Property(f"W_{node_id}", 5.0)
                        reliability = Property(f"R_{node_id}", 0.95)
                        frequency = Property(f"F_{node_id}", 1.0)
                        nodes[node_id] = LeafTask(node_id, text, context, cost, reliability, frequency)
                    else:
                        nodes[node_id] = Task(node_id, text)
            
            # Add root goal (simplified - you may need more complex hierarchy building)
            for node in nodes.values():
                if isinstance(node, Goal) and 'G1' in node.get_id():
                    self.goal_tree.add_root_goal(node)
                    break
                    
        except Exception as e:
            self.get_logger().error(f"Error loading goal model: {e}")
    
    def now_nanoseconds(self) -> int:
        """Get current time in nanoseconds"""
        return int(time.time() * 1_000_000_000)
    
    def now_seconds(self) -> float:
        """Get current time in seconds"""
        return time.time()
    
    def timer_callback(self):
        """Periodic timer callback for maintenance tasks"""
        self.count_to_calc_and_reset += 1
        self.count_to_fetch += 1
        
        # Calculate reliability periodically
        if self.count_to_calc_and_reset >= self.frequency:
            self._apply_time_window()
            for component in self.status.keys():
                self._calculate_component_reliability(component)
            self.count_to_calc_and_reset = 0
        
        # Reload formulas periodically
        if self.count_to_fetch >= self.frequency * 10:
            self._load_models()
            self.count_to_fetch = 0
        
        # Flush logs periodically
        if self.logical_clock % self.flush_interval == 0:
            self._flush_logs()
    
    def receive_persist_message(self, msg: Persist):
        """Process incoming persist messages"""
        self.logical_clock += 1
        
        try:
            if msg.type == "Status":
                self.arrived_status += 1
                self._persist_status(msg.timestamp, msg.source, msg.target, msg.content)
                self.status[msg.source].append((self.now_seconds(), msg.content))
                
            elif msg.type == "EnergyStatus":
                if msg.source != "/engine":
                    component_name = msg.source.lstrip('/')
                    cost = float(msg.content)
                    self.components_costs_engine[component_name] += cost
                    self.components_costs_enactor[component_name] += cost
                else:
                    self._process_engine_energy_status(msg)
                    
            elif msg.type == "Event":
                self._persist_event(msg.timestamp, msg.source, msg.target, msg.content)
                
                if len(self.events[msg.source]) <= self.buffer_size:
                    self.events[msg.source].append(msg.content)
                else:
                    self.events[msg.source].popleft()
                    self.events[msg.source].append(msg.content)
                
                # Update contexts
                key = msg.source.lstrip('/')
                self.contexts[key] = 1 if msg.content == "activate" else 0
                
            elif msg.type == "Uncertainty":
                self._persist_uncertainty(msg.timestamp, msg.source, msg.target, msg.content)
                
            elif msg.type == "AdaptationCommand":
                self._persist_adaptation(msg.timestamp, msg.source, msg.target, msg.content)
                
            else:
                self.get_logger().warn(f"Unknown message type: {msg.type}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing persist message: {e}")
    
    def _process_engine_energy_status(self, msg: Persist):
        """Process energy status from engine with multiple components"""
        try:
            content = msg.content.replace(';', ' ')
            costs = content.split()
            
            for cost_str in costs:
                if ':' in cost_str:
                    component, cost_val = cost_str.split(':', 1)
                    cost_float = float(cost_val)
                    self._persist_energy_status(msg.timestamp, component, msg.target, str(cost_float))
                    
        except Exception as e:
            self.get_logger().error(f"Error processing engine energy status: {e}")
    
    def process_target_system_data(self, msg: TargetSystemData):
        """Process target system data to update battery levels"""
        try:
            self.components_batteries["g3t1_1"] = msg.trm_batt
            self.components_batteries["g3t1_2"] = msg.ecg_batt
            self.components_batteries["g3t1_3"] = msg.oxi_batt
            self.components_batteries["g3t1_4"] = msg.abps_batt
            self.components_batteries["g3t1_5"] = msg.abpd_batt
            self.components_batteries["g3t1_6"] = msg.glc_batt
            
        except Exception as e:
            self.get_logger().error(f"Error processing target system data: {e}")
    
    def process_query(self, request: DataAccessRequest.Request, response: DataAccessRequest.Response):
        """Process data access queries from engines and enactor"""
        response.content = ""
        
        try:
            if request.name in ["/engine", "/enactor"]:
                query_parts = request.query.split(':')
                
                if len(query_parts) == 1:
                    if query_parts[0] == "reliability_formula":
                        response.content = self.reliability_formula_text
                    elif query_parts[0] == "cost_formula":
                        response.content = self.cost_formula_text
                
                elif len(query_parts) > 1:
                    if query_parts[1] == "reliability":
                        self._apply_time_window()
                        for component in self.status.keys():
                            response.content += self._calculate_component_reliability(component)
                            
                    elif query_parts[1] == "event":
                        num = int(query_parts[2]) if len(query_parts) > 2 else 1
                        
                        for source, event_deque in self.events.items():
                            if event_deque:
                                aux = f"{source}:"
                                content = ""
                                
                                # Get last 'num' events
                                events_to_process = list(event_deque)[-num:]
                                
                                for i, event in enumerate(events_to_process):
                                    aux += event
                                    content += event
                                    if i < len(events_to_process) - 1:
                                        aux += ","
                                
                                aux += ";"
                                
                                # Update context
                                key = source.lstrip('/')
                                self.contexts[key] = 1 if content == "activate" else 0
                                response.content += aux
                    
                    elif query_parts[1] == "cost":
                        self._apply_time_window()
                        for component in self.status.keys():
                            response.content += self._calculate_component_cost(component, request.name)
            
        except Exception as e:
            self.get_logger().error(f"Error processing query: {e}")
        
        return response
    
    def _calculate_component_reliability(self, component: str) -> str:
        """Calculate and return component reliability"""
        try:
            aux = f"{component}:"
            success_count = 0
            total_count = 0
            
            for _, status_content in self.status[component]:
                if status_content == "success":
                    success_count += 1
                    total_count += 1
                elif status_content == "fail":
                    total_count += 1
                else:
                    aux += f"{status_content},"
            
            reliability = success_count / total_count if total_count > 0 else 0
            aux += f"{reliability};"
            
            # Update component reliability
            key = component.lstrip('/')
            self.components_reliabilities[key] = reliability
            
            return aux
            
        except Exception as e:
            self.get_logger().error(f"Error calculating reliability for {component}: {e}")
            return f"{component}:0;"
    
    def _calculate_component_cost(self, component: str, requester: str) -> str:
        """Calculate and return component cost"""
        try:
            aux = f"{component}:"
            key = component.lstrip('/')
            
            if requester == "/engine":
                cost = self.components_costs_engine.get(key, 0)
                aux += f"{cost};"
                self.components_costs_engine[key] = 0
            elif requester == "/enactor":
                cost = self.components_costs_enactor.get(key, 0)
                aux += f"{cost};"
                self.components_costs_enactor[key] = 0
            
            return aux
            
        except Exception as e:
            self.get_logger().error(f"Error calculating cost for {component}: {e}")
            return f"{component}:0;"
    
    def _apply_time_window(self):
        """Apply time window to remove old status data"""
        current_time = self.now_seconds()
        
        for component, status_deque in self.status.items():
            while status_deque:
                time_arrived, _ = status_deque[0]
                time_span = current_time - time_arrived
                
                if time_span >= self.time_window:
                    status_deque.popleft()
                else:
                    break
    
    def _persist_status(self, timestamp: int, source: str, target: str, content: str):
        """Persist status message"""
        msg = StatusMessage(timestamp, self.logical_clock, source, target, content)
        self.status_messages.append(msg)
    
    def _persist_event(self, timestamp: int, source: str, target: str, content: str):
        """Persist event message"""
        msg = EventMessage(timestamp, self.logical_clock, source, target, content)
        self.event_messages.append(msg)
    
    def _persist_energy_status(self, timestamp: int, source: str, target: str, content: str):
        """Persist energy status message"""
        msg = EnergyStatusMessage(timestamp, self.logical_clock, source, target, content)
        self.energy_messages.append(msg)
    
    def _persist_uncertainty(self, timestamp: int, source: str, target: str, content: str):
        """Persist uncertainty message"""
        msg = UncertaintyMessage(timestamp, self.logical_clock, source, target, content)
        self.uncertainty_messages.append(msg)
    
    def _persist_adaptation(self, timestamp: int, source: str, target: str, content: str):
        """Persist adaptation message"""
        msg = AdaptationMessage(timestamp, self.logical_clock, source, target, content)
        self.adaptation_messages.append(msg)
    
    def _flush_logs(self):
        """Flush message buffers to log files"""
        try:
            # Flush status messages
            if self.status_messages:
                with open(self.log_files['status'], 'a') as f:
                    writer = csv.writer(f)
                    for msg in self.status_messages:
                        writer.writerow([
                            msg.msg_type, msg.logical_clock, msg.timestamp,
                            msg.source, msg.target, msg.content
                        ])
                self.status_messages.clear()
            
            # Flush event messages
            if self.event_messages:
                with open(self.log_files['event'], 'a') as f:
                    writer = csv.writer(f)
                    for msg in self.event_messages:
                        writer.writerow([
                            msg.msg_type, msg.logical_clock, msg.timestamp,
                            msg.source, msg.target, msg.content
                        ])
                self.event_messages.clear()
            
            # Flush energy messages
            if self.energy_messages:
                with open(self.log_files['energy'], 'a') as f:
                    writer = csv.writer(f)
                    for msg in self.energy_messages:
                        writer.writerow([
                            msg.msg_type, msg.logical_clock, msg.timestamp,
                            msg.source, msg.target, msg.content
                        ])
                self.energy_messages.clear()
            
            # Flush uncertainty messages
            if self.uncertainty_messages:
                with open(self.log_files['uncertainty'], 'a') as f:
                    writer = csv.writer(f)
                    for msg in self.uncertainty_messages:
                        writer.writerow([
                            msg.msg_type, msg.logical_clock, msg.timestamp,
                            msg.source, msg.target, msg.content
                        ])
                self.uncertainty_messages.clear()
            
            # Flush adaptation messages
            if self.adaptation_messages:
                with open(self.log_files['adaptation'], 'a') as f:
                    writer = csv.writer(f)
                    for msg in self.adaptation_messages:
                        writer.writerow([
                            msg.msg_type, msg.logical_clock, msg.timestamp,
                            msg.source, msg.target, msg.content
                        ])
                self.adaptation_messages.clear()
                
        except Exception as e:
            self.get_logger().error(f"Error flushing logs: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        data_access = DataAccess()
        
        # Use MultiThreadedExecutor for handling multiple callbacks
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(data_access)
        
        data_access.get_logger().info("DataAccess node started")
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if 'data_access' in locals():
            data_access.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
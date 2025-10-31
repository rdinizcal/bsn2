"""Shared pytest fixtures and helpers extracted from the system_monitor unit tests.

Import this module in other packages' tests to reuse the same setup and
utility functions from `test_node_monitor.py` and `test_logger.py`.

Example:
    from system_monitor.shared_tests import setup_ros, monitor_node, wait_for

Fixtures provided:
- setup_ros: ensures rclpy is initialized and shuts it down after the test
- monitor_node: yields a SystemMonitor instance
- logger_node: yields a Logger instance

Helpers provided:
- wait_for(condition, timeout)
- start_executor_with_nodes(nodes)
- stop_executor_and_cleanup(executor, spin_thread, nodes)
- publish_repeated(publisher, msg, times, delay)
- create_persist_collector(node, msg_type) -> (received_list, unsubscribe_fn)
"""
from __future__ import annotations

import threading
import time
from typing import Callable, List, Tuple

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Local imports for the classes under test
from system_monitor.node_monitor import SystemMonitor


class SharedNodeMonitorTests:
    """Shared test methods for SystemMonitor testing"""

    @staticmethod
    def assert_monitor_initialized(monitor: SystemMonitor) -> None:
        """Test initialization of SystemMonitor"""
        assert hasattr(monitor, 'log_status_pub')
        assert hasattr(monitor, 'log_event_pub')
        assert hasattr(monitor, 'log_energy_pub')
        assert hasattr(monitor, 'status_sub')
        assert hasattr(monitor, 'event_sub')
        assert hasattr(monitor, 'energy_subscribers')
        assert hasattr(monitor, 'monitored_nodes') and isinstance(monitor.monitored_nodes, dict)
        
        # Check for parameter loading
        assert hasattr(monitor, 'node_list')
        assert isinstance(monitor.node_list, list)
        assert hasattr(monitor, 'heartbeat_timeout')
        assert hasattr(monitor, 'check_interval')
        assert hasattr(monitor, 'debug')
    
    @staticmethod
    def assert_status_forwarding_works(received_messages: list):
        """Test forwarding of status messages (collector functionality)"""
        # Find the message we actually sent
        test_message = None
        for received_msg in received_messages:
            if (received_msg.source == "thermometer_node" and 
                received_msg.content == "activated" and 
                received_msg.task == "measuring"):
                test_message = received_msg
                break
        
        # Verify our specific message was forwarded
        assert test_message is not None, f"Test message not found in received messages"
        assert test_message.source == "thermometer_node"
        assert test_message.content == "activated"
        assert test_message.task == "measuring"
        return True
    
    @staticmethod
    def assert_event_forwarding_works(received_messages: list):
        """Test forwarding of event messages (collector functionality)"""
        # Verify message was forwarded
        assert len(received_messages) > 0, "No event messages forwarded"
        assert received_messages[0].source == "test_sensor"
        assert received_messages[0].content == "activate"
        return True
    
    @staticmethod
    def assert_energy_forwarding_works(received_messages: list):
        """Test forwarding of energy status messages (collector functionality)"""
        # Verify message was forwarded
        assert len(received_messages) > 0, "No energy status messages forwarded"
        assert received_messages[0].source == "test_sensor"
        assert received_messages[0].content == "energy:75.0:cost:0.5"
        return True
    
    @staticmethod
    def assert_node_state_tracking_works(monitor: SystemMonitor):
        """Test tracking of node states (monitor functionality)"""
        # Verify test node exists in monitored nodes
        test_node_name = "test_sensor"
        assert test_node_name in monitor.monitored_nodes, f"Test node {test_node_name} not found in monitored nodes"
        
        node_state = monitor.monitored_nodes[test_node_name]
        
        # Verify node state has expected structure and values
        assert 'state' in node_state
        assert 'last_heartbeat' in node_state
        assert 'active' in node_state
        assert 'task' in node_state
        assert 'content' in node_state
        assert 'is_recharging' in node_state
        
        return True
    
    @staticmethod
    def assert_status_publishing_works(monitor: SystemMonitor):
        """Test that monitor has status publishing capability"""
        # Verify monitor has the publish_monitor_status method
        assert hasattr(monitor, 'publish_monitor_status'), "Monitor missing publish_monitor_status method"
        
        # Verify monitor has monitored_nodes
        assert hasattr(monitor, 'monitored_nodes'), "Monitor missing monitored_nodes"
        assert isinstance(monitor.monitored_nodes, dict), "monitored_nodes should be a dict"
        
        return True
    
    @staticmethod
    def assert_heartbeat_checking_works(monitor: SystemMonitor):
        """Test that monitor has heartbeat checking capability"""
        # Verify monitor has the check_heartbeats method
        assert hasattr(monitor, 'check_heartbeats'), "Monitor missing check_heartbeats method"
        
        # Verify monitor has required attributes for heartbeat checking
        assert hasattr(monitor, 'node_list'), "Monitor missing node_list"
        assert hasattr(monitor, 'monitored_nodes'), "Monitor missing monitored_nodes"
        assert hasattr(monitor, 'heartbeat_timeout'), "Monitor missing heartbeat_timeout"
        
        return True

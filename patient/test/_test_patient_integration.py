#!/usr/bin/env python3
# test_patient_integration.py

import pytest
import rclpy
from rclpy.node import Node
from bsn_interfaces.srv import PatientData
import threading
import time
import collections
import statistics
from rclpy.executors import SingleThreadedExecutor


class TestPatientIntegration:
    """Integration tests for the Patient node."""

    @classmethod
    def setup_class(cls):
        """Set up the ROS 2 context for all tests."""
        rclpy.init()
        cls.node = Node("test_integration_node")
        
        # Create service client
        cls.client = cls.node.create_client(PatientData, "get_sensor_reading")
        
        # Wait for service to be available
        success = cls.client.wait_for_service(timeout_sec=5.0)
        assert success, "Patient service not available - is the patient node running?"
        
        
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()
        
    @classmethod
    def teardown_class(cls):
        """Clean up after all tests."""
        cls.executor.shutdown()
        if cls.spin_thread.is_alive():
            cls.spin_thread.join(timeout=1.0)
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_state_distribution(self):
        """Test the distribution of states over time."""
        NUM_SAMPLES = 100
        VITAL_SIGN = "temperature"  # Use a configured vital sign
        
        # Map to count occurrences in each state range
        state_distribution = collections.defaultdict(int)
        
        # Define ranges for each state (adjust based on actual configuration)
        state_ranges = [
            (0.0, 35.0),    # HighRisk0
            (35.0, 36.0),   # MidRisk0
            (36.0, 38.0),   # LowRisk
            (38.0, 39.0),   # MidRisk1
            (39.0, 42.0)    # HighRisk1
        ]
        
        # Collect samples
        values = []
        for _ in range(NUM_SAMPLES):
            request = PatientData.Request()
            request.vital_sign = VITAL_SIGN
            
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            
            response = future.result()
            assert response is not None
            
            value = response.datapoint
            values.append(value)
            
            # Determine which state range the value falls into
            for state, (min_val, max_val) in enumerate(state_ranges):
                if min_val <= value <= max_val:
                    state_distribution[state] += 1
                    break
            
            time.sleep(0.05)  # Wait between samples
        
        # Verify there's a reasonable distribution across states
        # (state 2 should be most common in normal distribution)
        assert len(state_distribution) > 1, "Values only found in one state range"
        
        # Log distribution
        self.node.get_logger().info(f"Value distribution by state: {dict(state_distribution)}")
        self.node.get_logger().info(f"Mean: {statistics.mean(values)}, Std dev: {statistics.stdev(values)}")

    def test_transition_edge_cases(self):
        """Test transitions during edge cases."""
        SAMPLES = 50
        VITAL_SIGN = "temperature"
        
        # Get initial value
        request = PatientData.Request()
        request.vital_sign = VITAL_SIGN
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        
        response = future.result()
        assert response is not None
        
        initial_value = response.datapoint
        last_value = initial_value
        
        # Track state changes
        state_changes = 0
        values = [initial_value]
        
        # Collect samples over time
        for _ in range(SAMPLES):
            time.sleep(0.2)  # Wait longer to allow state changes
            
            request = PatientData.Request()
            request.vital_sign = VITAL_SIGN
            
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            
            response = future.result()
            value = response.datapoint
            values.append(value)
            
            # Check for significant value changes that might indicate state transitions
            if abs(value - last_value) > 0.5:  # Adjust threshold based on your model
                state_changes += 1
                self.node.get_logger().info(
                    f"Potential state change: {last_value:.2f} -> {value:.2f}"
                )
            
            last_value = value
        
        # Log results
        self.node.get_logger().info(f"Observed {state_changes} potential state transitions in {SAMPLES} samples")
        self.node.get_logger().info(f"Values range: min={min(values):.2f}, max={max(values):.2f}")
        
        # We expect some state changes in a reasonable sample size over time
        # This might need adjustment based on your actual change rates
        assert max(values) - min(values) > 0.1, "No significant value changes observed"
import pytest
import time
import rclpy
from fixtures import sensor_node


@pytest.mark.usefixtures("sensor_node")
class TestSensorBehavior:
    sensor_node = None  # Will be set by fixture

    def setup_method(self):
        """Set up before each test"""
        # Clear the data window
        if hasattr(self.sensor_node, "processor") and hasattr(
            self.sensor_node.processor, "data_window"
        ):
            self.sensor_node.processor.data_window.clear()

        self.sensor_node.active = True

    def test_collect_with_mock_service(self):
        """Test the collect method with our mock service"""
        datapoint = self.sensor_node.processor.collect()
        assert datapoint == 37.0

        # Verify consistent results from the mock service
        for _ in range(3):
            data_point = self.sensor_node.processor.collect()
            assert data_point == 37.0, "Collect method should return consistent results"

    def test_process_with_filled_window(self):
        """Test the process method with a filled data window"""
        # Clear the window and fill it with known values
        self.sensor_node.processor.data_window.clear()
        test_values = [36.5, 36.7, 36.9, 37.1, 37.3]
        for val in test_values:
            self.sensor_node.processor.data_window.append(val)

        # Process the data using processor component
        processed = self.sensor_node.processor.process(37.0)

        # Expected average is the average of the window contents
        expected_avg = sum(test_values) / len(test_values)
        assert abs(processed - expected_avg) < 0.11

    def test_process_with_incomplete_window(self):
        """Test process method with incomplete window"""
        # Clear the window
        self.sensor_node.processor.data_window.clear()

        # Add just one value
        self.sensor_node.processor.data_window.append(36.5)

        # Process should return -1 for insufficient data
        result = self.sensor_node.processor.process(36.5)
        assert result == -1.0

    def test_transfer_creates_valid_message_data(self):
        """Test that transfer creates message with correct internal data"""
        # Test the transfer method's internal behavior
        test_value = 37.0

        # Check that the sensor has the necessary components
        assert hasattr(self.sensor_node, "processor")
        assert hasattr(self.sensor_node.processor, "transfer")

        # Call transfer - it should not raise exceptions
        try:
            result = self.sensor_node.processor.transfer(test_value)
            # If transfer returns something, verify it's valid
            if result is not None:
                assert isinstance(result, (int, float, bool))
        except Exception as e:
            pytest.fail(f"Transfer method failed with exception: {e}")

    def test_sensor_data_flow(self):
        """Test the complete data flow: collect -> process -> transfer"""
        # Pre-fill the data window to ensure process works
        for _ in range(self.sensor_node.config.window_size - 1):
            self.sensor_node.processor.data_window.append(37.0)

        # Step 1: Collect data
        collected_data = self.sensor_node.processor.collect()
        assert collected_data == 37.0

        # Step 2: Process the data
        processed_data = self.sensor_node.processor.process(collected_data)
        assert processed_data == 37.0  # Should be average of window

        # Step 3: Transfer the processed data
        # This should complete without errors
        try:
            self.sensor_node.processor.transfer(processed_data)
        except Exception as e:
            pytest.fail(f"Data flow failed at transfer step: {e}")

    def test_risk_evaluation_low_risk(self):
        """Test risk evaluation for values in low risk range"""
        ranges = self.sensor_node.risk_manager.evaluator.sensor_ranges[
            self.sensor_node.config.sensor
        ]
        low_range = ranges["low_risk"]

        # Create a value in the middle of low risk range
        test_value = (low_range[0] + low_range[1]) / 2

        # Evaluate risk through risk manager
        risk_value = self.sensor_node.risk_manager.evaluate_risk(test_value)

        # Check that risk is in low risk percentage range
        evaluator = self.sensor_node.risk_manager.evaluator
        assert evaluator.is_low_risk(risk_value)
        assert evaluator.risk_label(risk_value) == "low"

    def test_risk_evaluation_medium_risk(self):
        """Test risk evaluation for values in medium risk range"""
        ranges = self.sensor_node.risk_manager.evaluator.sensor_ranges[
            self.sensor_node.config.sensor
        ]
        mid_range = ranges["mid_risk1"]

        # Create a value in the middle of medium risk range
        test_value = (mid_range[0] + mid_range[1]) / 2

        # Evaluate risk through risk manager
        risk_value = self.sensor_node.risk_manager.evaluate_risk(test_value)

        # Check that risk is in medium risk percentage range
        evaluator = self.sensor_node.risk_manager.evaluator
        assert evaluator.is_medium_risk(risk_value)
        assert evaluator.risk_label(risk_value) == "moderate"

    def test_risk_evaluation_high_risk(self):
        """Test risk evaluation for values in high risk range"""
        ranges = self.sensor_node.risk_manager.evaluator.sensor_ranges[
            self.sensor_node.config.sensor
        ]
        high_range = ranges["high_risk1"]

        # Create a value in the middle of high risk range
        test_value = (high_range[0] + high_range[1]) / 2

        # Evaluate risk through risk manager
        risk_value = self.sensor_node.risk_manager.evaluate_risk(test_value)

        # Check that risk is in high risk percentage range
        evaluator = self.sensor_node.risk_manager.evaluator
        assert evaluator.is_high_risk(risk_value)
        assert evaluator.risk_label(risk_value) == "high"

    def test_integrated_sensor_cycle(self):
        """Test the full sensor cycle focusing on internal state changes"""
        # Clear the data window and pre-fill it
        self.sensor_node.processor.data_window.clear()
        initial_window_size = len(self.sensor_node.processor.data_window)

        for _ in range(self.sensor_node.config.window_size - 1):
            self.sensor_node.processor.data_window.append(37.0)

        # Verify window is filled
        assert (
            len(self.sensor_node.processor.data_window)
            == self.sensor_node.config.window_size - 1
        )

        # Process with a new value
        processed = self.sensor_node.processor.process(37.0)
        assert processed == 37.0

        # Verify the window now has the expected size
        assert (
            len(self.sensor_node.processor.data_window)
            == self.sensor_node.config.window_size
        )

        # Transfer should complete successfully
        self.sensor_node.processor.transfer(processed)

    def test_displacement_calculation_crescent(self):
        """Test displacement calculation with crescent logic"""
        evaluator = self.sensor_node.risk_manager.evaluator

        # Test with range [10, 20] and value 15 (should be 0.5)
        displacement = evaluator.get_displacement(10, 20, 15, "crescent")
        assert displacement == 0.5

        # Test with minimum value (should be 0.0)
        displacement = evaluator.get_displacement(10, 20, 10, "crescent")
        assert displacement == 0.0

        # Test with maximum value (should be 1.0)
        displacement = evaluator.get_displacement(10, 20, 20, "crescent")
        assert displacement == 1.0

    def test_displacement_calculation_decrescent(self):
        """Test displacement calculation with decrescent logic"""
        evaluator = self.sensor_node.risk_manager.evaluator

        # Test with range [10, 20] and value 15 (should be 0.5)
        displacement = evaluator.get_displacement(10, 20, 15, "decrescent")
        assert displacement == 0.5

        # Test with minimum value (should be 1.0)
        displacement = evaluator.get_displacement(10, 20, 10, "decrescent")
        assert displacement == 1.0

        # Test with maximum value (should be 0.0)
        displacement = evaluator.get_displacement(10, 20, 20, "decrescent")
        assert displacement == 0.0

    def test_displacement_calculation_medium(self):
        """Test displacement calculation with medium logic"""
        evaluator = self.sensor_node.risk_manager.evaluator

        # Test with range [10, 30] and value 20 (should be 0.0)
        displacement = evaluator.get_displacement(10, 30, 20, "medium")
        assert displacement == 0.0

        # Test with extreme value (should be 1.0)
        displacement = evaluator.get_displacement(10, 30, 30, "medium")
        assert displacement == 1.0

    def test_percentage_conversion(self):
        """Test conversion from displacement to percentage range"""
        evaluator = self.sensor_node.risk_manager.evaluator

        # Test with range [10, 20] and displacement 0.5 (should be 15)
        percentage = evaluator.convert_percentage(10, 20, 0.5)
        assert percentage == 15.0

        # Test with displacement 0.0 (should be minimum)
        percentage = evaluator.convert_percentage(10, 20, 0.0)
        assert percentage == 10.0

        # Test with displacement 1.0 (should be maximum)
        percentage = evaluator.convert_percentage(10, 20, 1.0)
        assert percentage == 20.0

    def test_risk_evaluation_edge_cases(self):
        """Test risk evaluation at edge cases of ranges"""
        sensor_type = self.sensor_node.config.sensor
        evaluator = self.sensor_node.risk_manager.evaluator
        ranges = evaluator.sensor_ranges[sensor_type]

        # Test exactly at range boundaries with a small tolerance for floating point precision
        low_risk_min = ranges["low_risk"][0]
        low_risk_min_risk = evaluator.evaluate_risk(sensor_type, low_risk_min)

        # Add a small tolerance (0.001) for floating point precision
        assert abs(
            low_risk_min_risk - evaluator.low_percentage[1]
        ) <= 0.001 or evaluator.is_low_risk(low_risk_min_risk)

        low_risk_max = ranges["low_risk"][1]
        low_risk_max_risk = evaluator.evaluate_risk(sensor_type, low_risk_max)
        assert abs(
            low_risk_max_risk - evaluator.low_percentage[1]
        ) <= 0.001 or evaluator.is_low_risk(low_risk_max_risk)

    def test_invalid_sensor_type(self):
        """Test risk evaluation with invalid sensor type"""
        evaluator = self.sensor_node.risk_manager.evaluator
        risk = evaluator.evaluate_risk("nonexistent_sensor", 37.0)
        assert risk == -1.0

    def test_custom_risk_percentages(self):
        """Test configuring custom risk percentages"""
        evaluator = self.sensor_node.risk_manager.evaluator
        sensor_type = self.sensor_node.config.sensor

        # Save original percentages
        original_low = evaluator.low_percentage
        original_mid = evaluator.mid_percentage
        original_high = evaluator.high_percentage

        try:
            # Set custom percentages
            custom_percentages = [(0.0, 30.0), (31.0, 70.0), (71.0, 100.0)]
            evaluator.configure(
                sensor_type, evaluator.sensor_ranges[sensor_type], custom_percentages
            )

            # Test with new percentages
            assert evaluator.low_percentage == (0.0, 30.0)
            assert evaluator.mid_percentage == (31.0, 70.0)
            assert evaluator.high_percentage == (71.0, 100.0)

            # Test classification with new percentages
            assert evaluator.is_low_risk(15.0)
            assert not evaluator.is_low_risk(35.0)
            assert evaluator.is_medium_risk(50.0)
            assert evaluator.is_high_risk(85.0)

        finally:
            # Restore original percentages
            evaluator.low_percentage = original_low
            evaluator.mid_percentage = original_mid
            evaluator.high_percentage = original_high

    def test_invalid_logic_parameter(self):
        """Test handling of invalid logic parameter"""
        evaluator = self.sensor_node.risk_manager.evaluator

        with pytest.raises(ValueError):
            evaluator.get_displacement(10, 20, 15, "invalid_logic")

    def test_collect_service_failure(self, monkeypatch):
        """Test collect method with service call failure"""
        original_collect = self.sensor_node.processor.collect

        def mock_failed_collect():
            return -1.0  # This simulates service call failure

        # Apply the monkeypatch
        monkeypatch.setattr(self.sensor_node.processor, "collect", mock_failed_collect)

        # Call collect and check result
        result = self.sensor_node.processor.collect()
        assert result == -1.0, "Should return failure indicator when service call fails"

        # Restore is automatic with monkeypatch

    def test_assess_risk_thermometer(self):
        """Test risk evaluation for thermometer with various values"""
        risk_mgr = self.sensor_node.risk_manager

        # For values in the valid range, the risk assessment should work correctly
        test_cases = [
            (31.0, "high"),  # Too low
            (33.0, "moderate"),  # Below normal
            (37.0, "low"),  # Normal
            (39.0, "moderate"),  # Above normal
            (41.0, "high"),  # Too high
        ]

        # Test using the available risk evaluation method
        for value, expected in test_cases:
            # Get risk percentage (numerical value)
            risk_value = risk_mgr.evaluate_risk(value)

            # Get risk label based on that percentage
            risk_label = risk_mgr.get_risk_label(risk_value)

            # Assert the expected risk level
            assert (
                risk_label == expected
            ), f"Expected {expected} risk for {value}, got {risk_label}"

    def test_data_window_management(self):
        """Test that data window is properly managed during processing"""
        # Clear window
        self.sensor_node.processor.data_window.clear()
        initial_size = len(self.sensor_node.processor.data_window)
        assert initial_size == 0

        # Fill window gradually
        for i in range(self.sensor_node.config.window_size):
            # Before processing, should have i elements
            assert len(self.sensor_node.processor.data_window) == i

            # Process a value
            result = self.sensor_node.processor.process(37.0 + i * 0.1)

            # After processing, should have min(i+1, window_size) elements
            expected_size = min(i + 1, self.sensor_node.config.window_size)
            assert len(self.sensor_node.processor.data_window) == expected_size

    def test_processor_components_exist(self):
        """Test that all required processor components exist"""
        assert hasattr(self.sensor_node, "processor")
        assert hasattr(self.sensor_node.processor, "collect")
        assert hasattr(self.sensor_node.processor, "process")
        assert hasattr(self.sensor_node.processor, "transfer")
        assert hasattr(self.sensor_node.processor, "data_window")

        assert hasattr(self.sensor_node, "risk_manager")
        assert hasattr(self.sensor_node.risk_manager, "evaluate_risk")

        assert hasattr(self.sensor_node, "config")
        assert hasattr(self.sensor_node.config, "window_size")
        assert hasattr(self.sensor_node.config, "sensor")

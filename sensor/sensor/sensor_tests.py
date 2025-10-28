"""Shared test methods that can be used by both BDD and characteristic tests"""


class SharedSensorTests:
    """Shared test methods for sensor testing"""

    @staticmethod
    def assert_collect_works(sensor_node):
        """Assert that collect method returns expected value"""
        datapoint = sensor_node.processor.collect()
        assert datapoint == 37.0
        return True

    @staticmethod
    def assert_process_with_filled_window_works(sensor_node):
        """Assert that process method works with filled window"""
        # Clear and fill window with known values
        sensor_node.processor.data_window.clear()
        test_values = [36.5, 36.7, 36.9, 37.1, 37.3]
        for val in test_values:
            sensor_node.processor.data_window.append(val)

        # Process the data
        processed = sensor_node.processor.process(37.0)

        # Expected average
        expected_avg = sum(test_values) / len(test_values)
        assert abs(processed - expected_avg) < 0.11
        return processed

    @staticmethod
    def assert_sensor_data_flow_works(sensor_node):
        """Assert that complete sensor data flow works"""
        # Pre-fill the data window to ensure process works
        for _ in range(sensor_node.config.window_size - 1):
            sensor_node.processor.data_window.append(37.0)

        # Step 1: Collect data
        collected_data = sensor_node.processor.collect()
        assert collected_data == 37.0

        # Step 2: Process the data
        processed_data = sensor_node.processor.process(collected_data)
        assert processed_data == 37.0  # Should be average of window

        # Step 3: Transfer the processed data
        # This should complete without errors
        try:
            sensor_node.processor.transfer(processed_data)
        except Exception as e:
            raise AssertionError(f"Data flow failed at transfer step: {e}")

        return processed_data

    @staticmethod
    def assert_data_within_valid_range(sensor_node, min_val=35.0, max_val=39.0):
        """Assert that data window contains values within valid range"""
        data_window = sensor_node.processor.data_window
        assert len(data_window) > 0, "Data window is empty"

        for value in data_window:
            assert (
                min_val <= value <= max_val
            ), f"Value {value} is outside valid range [{min_val}, {max_val}]"

        return True

    @staticmethod
    def assert_risk_evaluation_works(sensor_node, test_value, expected_risk_level):
        """Assert that risk evaluation works for given value and expected level"""
        risk_value = sensor_node.risk_manager.evaluate_risk(test_value)
        evaluator = sensor_node.risk_manager.evaluator
        actual_risk_level = evaluator.risk_label(risk_value)

        assert (
            actual_risk_level == expected_risk_level
        ), f"Expected {expected_risk_level} risk for {test_value}, got {actual_risk_level}"
        return True

    @staticmethod
    def assert_process_with_incomplete_window_fails(sensor_node):
        """Assert that process method handles incomplete window correctly"""
        # Clear the window
        sensor_node.processor.data_window.clear()

        # Add just one value (insufficient)
        sensor_node.processor.data_window.append(36.5)

        # Process should return -1 for insufficient data
        result = sensor_node.processor.process(36.5)
        assert result == -1.0, "Process should return -1.0 for insufficient data"
        return True

    @staticmethod
    def assert_transfer_works(sensor_node, test_value=37.0):
        """Assert that transfer method works without exceptions"""
        assert hasattr(sensor_node, "processor")
        assert hasattr(sensor_node.processor, "transfer")

        try:
            result = sensor_node.processor.transfer(test_value)
            # If transfer returns something, verify it's valid
            if result is not None:
                assert isinstance(result, (int, float, bool))
            return True
        except Exception as e:
            raise AssertionError(f"Transfer method failed with exception: {e}")

    @staticmethod
    def assert_processor_components_exist(sensor_node):
        """Assert that all required processor components exist"""
        assert hasattr(
            sensor_node, "processor"
        ), "Sensor node missing processor component"
        assert hasattr(
            sensor_node.processor, "collect"
        ), "Processor missing collect method"
        assert hasattr(
            sensor_node.processor, "process"
        ), "Processor missing process method"
        assert hasattr(
            sensor_node.processor, "transfer"
        ), "Processor missing transfer method"
        assert hasattr(
            sensor_node.processor, "data_window"
        ), "Processor missing data_window"

        assert hasattr(sensor_node, "risk_manager"), "Sensor node missing risk_manager"
        assert hasattr(
            sensor_node.risk_manager, "evaluate_risk"
        ), "Risk manager missing evaluate_risk method"

        assert hasattr(sensor_node, "config"), "Sensor node missing config"
        assert hasattr(sensor_node.config, "window_size"), "Config missing window_size"
        assert hasattr(sensor_node.config, "component"), "Config missing sensor type"

        return True

"""
Data visualization and formatting for central hub logging.

This module provides data formatting capabilities for the central hub,
creating readable tabular displays of sensor data, risk levels, and
system status for logging and monitoring purposes.
"""


class Visualizer:
    """
    Formats and visualizes sensor data for logging and monitoring.
    
    This class creates formatted table displays of sensor readings and
    risk assessments for easy human readability in log outputs. It handles
    missing data gracefully and includes appropriate units for each sensor type.
    
    Attributes:
        node: Reference to the parent central hub node.
        
    Examples:
        ```python
        visualizer = Visualizer(central_hub_node)
        
        # Generate formatted log message
        log_table = visualizer.format_log_message()
        central_hub_node.get_logger().info(log_table)
        ```
    """
    
    def __init__(self, node):
        """
        Initialize visualizer for data formatting.
        
        Sets up the visualizer with reference to the parent node
        for accessing sensor data and battery management.
        
        Args:
            node: The parent central hub node instance.
        """
        self.node = node
    
    def format_log_message(self):
        """
        Format sensor data into a readable tabular display.
        
        Creates a formatted ASCII table containing sensor readings, their
        values with appropriate units, and risk level assessments. Handles
        missing or insufficient data by displaying "waiting data" status.
        
        The table includes:
        - Sensor names (vital signs)
        - Current readings with units
        - Risk level assessments
        
        Consumes minimal battery for the formatting operation.
        
        Returns:
            str: Formatted table as a multi-line string ready for logging.
        """
        # Consume minimal battery for formatting
        self.node.battery_manager.consume(0.001)
        
        log_message = " \n"
        border = "+-----------------+-----------------+-----------------------------+"
        header = "| {0:<15} | {1:<15} | {2:<27} |".format(
            "Vital Sign", "Value", "Risk Level"
        )

        log_message += border + "\n"
        log_message += header + "\n"
        log_message += border + "\n"

        # Iterate over the latest data to process received data
        for signal, value in self.node.sensor_handler.latest_data.items():
            if value == self.node.sensor_handler.NOT_USED:
                continue
            if value == -1:  # Insufficient data for moving average
                log_message += f"| {signal:<15} | waiting data      |                           |\n"
                continue
            else:
                risk = self.node.sensor_handler.latest_risks_labels[signal]
                unit = ""
                if signal == "thermometer":
                    unit = "°C"
                elif signal in ["abpd", "abps"]:
                    unit = "mmHg"
                elif signal == "ecg":
                    unit = "bpm"
                elif signal == "glucosemeter":
                    unit = "mg/dL"
                elif signal == "oximeter":
                    unit = "% SpO2"

                log_message += (
                    f"| {signal:<15} | {value:>7.2f} {unit:<7} | Risk: {risk:<21} |\n"
                )

        log_message += (
            "+-----------------+-----------------+-----------------------------+"
        )
        return log_message
    
    def format_summary_message(self, patient_status):
        """
        Format a summary message with overall patient status.
        
        Creates a concise summary of the current patient monitoring
        status including the overall risk percentage and risk category.
        
        Args:
            patient_status (float): Overall patient risk percentage.
            
        Returns:
            str: Formatted summary message for logging.
        """
        if hasattr(self.node, 'risk_analyzer'):
            risk_category = self.node.risk_analyzer.get_risk_category(patient_status)
        else:
            risk_category = "UNKNOWN"
            
        return f"Patient Status: {patient_status:.1f}% - {risk_category}"
    
    def format_battery_status(self):
        """
        Format battery status information for all components.
        
        Creates a formatted display of battery levels for the central hub
        and all connected sensors for system monitoring purposes.
        
        Returns:
            str: Formatted battery status table.
        """
        log_message = "\nBattery Status:\n"
        log_message += f"Central Hub: {self.node.battery_manager.battery.current_level:.1f}%\n"
        
        for sensor, level in self.node.sensor_handler.sensor_battery_levels.items():
            log_message += f"{sensor.capitalize()}: {level:.1f}%\n"
            
        return log_message
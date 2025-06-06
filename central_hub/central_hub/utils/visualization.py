class Visualizer:
    """Formats and visualizes data for logging"""
    
    def __init__(self, node):
        self.node = node
    
    def format_log_message(self):
        """Format sensor data into a readable table"""
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
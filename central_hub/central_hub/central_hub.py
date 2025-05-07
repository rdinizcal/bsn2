import rclpy
import threading
from rclpy.node import Node
from bsn_interfaces.msg import SensorData
from bsn_interfaces.msg import TargetSystemData
from std_msgs.msg import Header


class CentralHub(Node):

    def __init__(self):
        super().__init__("central_hub")
        self.sub_abpd = self.create_subscription(
            SensorData, "sensor_data/abpd", self.receive_datapoint, 10
        )
        self.sub_abps = self.create_subscription(
            SensorData, "sensor_data/abps", self.receive_datapoint, 10
        )
        self.sub_ecg = self.create_subscription(
            SensorData, "sensor_data/ecg", self.receive_datapoint, 10
        )
        self.sub_glucosemeter = self.create_subscription(
            SensorData, "sensor_data/glucosemeter", self.receive_datapoint, 10
        )
        self.sub_oximeter = self.create_subscription(
            SensorData, "sensor_data/oximeter", self.receive_datapoint, 10
        )
        self.sub_thermometer = self.create_subscription(
            SensorData, "sensor_data/thermometer", self.receive_datapoint, 10
        )
        self.NOT_USED = ""

        self.target_system_publisher = self.create_publisher(
            TargetSystemData, "target_system_data", 10
        )
        

        # Add risk percentage storage
        self.latest_risk = {
            "abpd": 0.0,
            "abps": 0.0,
            "ecg": 0.0,
            "glucosemeter": 0.0,
            "oximeter": 0.0,
            "thermometer": 0.0,
        }

        self.latest_data = {
            "abpd": -1.0,
            "abps": -1.0,
            "ecg": -1.0,
            "glucosemeter": -1.0,
            "oximeter": -1.0,
            "thermometer": -1.0,
        }

        self.latest_risks_labels = {
            "abpd": "unknown",
            "abps": "unknown",
            "ecg": "unknown",
            "glucosemeter": "unknown",
            "oximeter": "unknown",
            "thermometer": "unknown",
        }

    def receive_datapoint(self, msg):
        if msg.sensor_type == "":
            self.get_logger().debug(
                f"null value received: {msg.sensor_type} with {msg.sensor_datapoint}"
            )
        else:
            # Update the latest data and risk for the corresponding sensor type
            self.latest_data[msg.sensor_type] = msg.sensor_datapoint
            self.latest_risks_labels[msg.sensor_type] = msg.risk_level
            self.latest_risk[msg.sensor_type] = msg.risk

            self.get_logger().info(
                f"Received data from {msg.sensor_type}: {msg.sensor_datapoint} with risk: {msg.risk_level}"
            )
    def data_fuse(self):
        """Calculate patient status using the original BSN fusion algorithm"""
        sensor_types = ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]

        # Get risk values in order
        packets_received = []
        for sensor_type in sensor_types:
            packets_received.append(self.latest_risk.get(sensor_type, -1.0))

        # Special handling and calculations
        average = 0.0
        count = 0
        index = 0
        bpr_avg = 0.0
        values = []

        for risk in packets_received:
            if risk >= 0:
                # Special handling for blood pressure sensors
                if index == 3 or index == 4:  # abps or abpd
                    bpr_avg += risk
                else:
                    average += risk
                    values.append(risk)
                count += 1

            # Process blood pressure after both readings
            if index == 4 and bpr_avg >= 0.0:
                bpr_avg /= 2  # Average of systolic and diastolic
                average += bpr_avg
                values.append(bpr_avg)

            index += 1

        if count == 0:
            return 0.0  # No data

        # Calculate average
        avg = average / count

        # Calculate deviations
        deviations = []
        min_dev = float('inf')
        max_dev = -float('inf')

        for value in values:
            dev = abs(value - avg)
            deviations.append(dev)

            if dev > max_dev:
                max_dev = dev
            if dev < min_dev:
                min_dev = dev

        # Calculate weighted average
        weighted_average = 0.0
        weight_sum = 0.0

        # If all values are the same, return simple average
        if max_dev - min_dev <= 0.0:
            return avg

        # Otherwise calculate weighted average based on deviations
        for i in range(len(values)):
            # Normalize deviation to 0-1 range
            norm_dev = (deviations[i] - min_dev) / (max_dev - min_dev)
            weight_sum += norm_dev
            weighted_average += values[i] * norm_dev

        # Final weighted risk status
        if weight_sum > 0:
            risk_status = weighted_average / weight_sum
        else:
            risk_status = avg

        return risk_status

    def format_log_message(self):
        """Format sensor data into a readable table"""
        log_message = " \n"
        border = "+-----------------+-----------------+-----------------------------+"
        header = "| {0:<15} | {1:<15} | {2:<27} |".format(
            "Vital Sign", "Value", "Risk Level"
        )

        log_message += border + "\n"
        log_message += header + "\n"
        log_message += border + "\n"

        # Iterate over the latest data to process received data
        for signal, value in self.latest_data.items():
            if value == self.NOT_USED:
                continue
            if value == -1:  # Insufficient data for moving average
                log_message += f"| {signal:<15} | waiting data      |                           |\n"
                continue
            else:
                risk = self.latest_risks_labels[signal]
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

    def detect(self):
        # Create a TargetSystemData message
        msg = TargetSystemData()

        # Add header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "central_hub"
        msg.header = header
        
        patient_status = self.data_fuse()
        

        # Display formatted log message
        log_message = self.format_log_message()
        self.get_logger().info(log_message)

        # Emit alerts for high/moderate risks
        self.emit_alert(patient_status)

        # Populate the message fields
        msg.trm_data = self.latest_data.get("thermometer", -1.0)
        msg.ecg_data = self.latest_data.get("ecg", -1.0)
        msg.oxi_data = self.latest_data.get("oximeter", -1.0)
        msg.abps_data = self.latest_data.get("abps", -1.0)
        msg.abpd_data = self.latest_data.get("abpd", -1.0)
        msg.glc_data = self.latest_data.get("glucosemeter", -1.0)

        # Set risk values from risk_percentages
        msg.trm_risk = self.latest_risk.get("thermometer", 0.0)
        msg.ecg_risk = self.latest_risk.get("ecg", 0.0)
        msg.oxi_risk = self.latest_risk.get("oximeter", 0.0)
        msg.abps_risk = self.latest_risk.get("abps", 0.0)
        msg.abpd_risk = self.latest_risk.get("abpd", 0.0)
        msg.glc_risk = self.latest_risk.get("glucosemeter", 0.0)

        # Placeholder values for battery levels and patient status
        msg.trm_batt = 100.0
        msg.ecg_batt = 100.0
        msg.oxi_batt = 100.0
        msg.abps_batt = 100.0
        msg.abpd_batt = 100.0
        msg.glc_batt = 100.0
        
        
        msg.patient_status = patient_status 

        # Publish the message
        self.target_system_publisher.publish(msg)
        self.get_logger().info("Published TargetSystemData")

    def emit_alert(self, patient_status):
        """Emit alerts based on patient status and sensor risk levels"""
        # Handle overall patient status if provided
        
        # Categorize patient status
        if patient_status <= 20.0:
            risk_category = "VERY LOW RISK"
        elif 20.0 < patient_status <= 40.0:
            risk_category = "LOW RISK"
        elif 40.0 < patient_status <= 60.0:
            risk_category = "MODERATE RISK"
        elif 60.0 < patient_status <= 80.0:
            risk_category = "CRITICAL RISK"
            self.get_logger().fatal(
                f"[Emergency Detection]\n SYSTEM ALERT: {risk_category} - Patient Status: {patient_status:.1f}%\n"
            )
        elif 80.0 < patient_status <= 100.0:
            risk_category = "VERY CRITICAL RISK"
            self.get_logger().fatal(
                f"[Emergency Detection]\n SYSTEM ALERT: {risk_category} - Patient Status: {patient_status:.1f}%\n"
            )
        else:
            risk_category = "UNKNOWN RISK"
        # Log moderate/low risk (critical alerts already handled above)
        if 40.0 < patient_status <= 60.0:
            self.get_logger().warning(
                f"[Emergency Detection]\n SYSTEM WARNING: {risk_category} - Patient Status: {patient_status:.1f}%\n"
            )
        elif patient_status <= 40.0 and patient_status > 0:
            self.get_logger().info(
                f"[Emergency Detection]\n System Status: {risk_category} - {patient_status:.1f}%\n"
            )
        


def main(args=None):
    rclpy.init(args=args)

    emergency_detector = CentralHub()

    # Run spin in a thread, make thread daemon so we don't have to join it to exit
    thread = threading.Thread(
        target=rclpy.spin, args=(emergency_detector,), daemon=True
    )
    thread.start()
    rate = emergency_detector.create_rate(2)  # 2 Hz

    try:
        while rclpy.ok():
            emergency_detector.detect()
            rate.sleep()
    finally:
        emergency_detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

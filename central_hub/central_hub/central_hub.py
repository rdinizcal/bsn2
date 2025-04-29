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

        self.latest_data = {
            "abpd": -1.0,
            "abps": -1.0,
            "ecg": -1.0,
            "glucosemeter": -1.0,
            "oximeter": -1.0,
            "thermometer": -1.0,
        }
        
        self.latest_risks = {
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
            self.latest_risks[msg.sensor_type] = msg.risk_level
            self.get_logger().info(
                f"Received data from {msg.sensor_type}: {msg.sensor_datapoint} with risk: {msg.risk_level}"
            )

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
                risk = self.latest_risks[signal]
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
                
                log_message += f"| {signal:<15} | {value:>7.2f} {unit:<7} | Risk: {risk:<21} |\n"

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

        # Display formatted log message
        log_message = self.format_log_message()
        self.get_logger().info(log_message)
        
        # Emit alerts for high/moderate risks
        self.emit_alert()
        
        # Populate the message fields
        msg.trm_data = self.latest_data.get("thermometer", -1.0)
        msg.ecg_data = self.latest_data.get("ecg", -1.0)
        msg.oxi_data = self.latest_data.get("oximeter", -1.0)
        msg.abps_data = self.latest_data.get("abps", -1.0)
        msg.abpd_data = self.latest_data.get("abpd", -1.0)
        msg.glc_data = self.latest_data.get("glucosemeter", -1.0)

        # Convert string risk levels to numeric values
        msg.trm_risk = self.risk_to_numeric("thermometer")
        msg.ecg_risk = self.risk_to_numeric("ecg") 
        msg.oxi_risk = self.risk_to_numeric("oximeter")
        msg.abps_risk = self.risk_to_numeric("abps")
        msg.abpd_risk = self.risk_to_numeric("abpd")
        msg.glc_risk = self.risk_to_numeric("glucosemeter")

        # Placeholder values for battery levels and patient status
        msg.trm_batt = 100.0
        msg.ecg_batt = 100.0
        msg.oxi_batt = 100.0
        msg.abps_batt = 100.0
        msg.abpd_batt = 100.0
        msg.glc_batt = 100.0
        msg.patient_status = 0.0  # Placeholder for patient status

        # Publish the message
        self.target_system_publisher.publish(msg)
        self.get_logger().info("Published TargetSystemData")
    
    def risk_to_numeric(self, sensor_type):
        """Convert string risk level to numeric value"""
        risk = self.latest_risks.get(sensor_type, "unknown")
        if risk == "high":
            return 1.0
        elif risk == "moderate":
            return 0.5
        else:  # normal or unknown
            return 0.0

    def emit_alert(self):
        """Emit alerts based on received risk levels"""
        for signal, level in self.latest_risks.items():
            if level == "high":
                self.get_logger().fatal(
                    f"[Emergency Detection]\n ALERT: High risk in {signal} detected!\n"
                )
            elif level == "moderate":
                self.get_logger().warning(
                    f"[Emergency Detection]\n Warning: Abnormal {signal} levels.\n"
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
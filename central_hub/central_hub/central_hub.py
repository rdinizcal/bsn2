import rclpy
import threading
from rclpy.node import Node
from bsn_interfaces.msg import SensorData
from bsn_interfaces.msg import TargetSystemData
from std_msgs.msg import Header

class CentralHub(Node):

    def __init__(self):
        super().__init__('central_hub')
        self.sub_abpd = self.create_subscription(SensorData, 'sensor_data/abpd', self.receive_datapoint, 10)
        self.sub_abps = self.create_subscription(SensorData, 'sensor_data/abps', self.receive_datapoint, 10)
        self.sub_ecg = self.create_subscription(SensorData, 'sensor_data/ecg', self.receive_datapoint, 10)
        self.sub_glucosemeter = self.create_subscription(SensorData, 'sensor_data/glucosemeter', self.receive_datapoint, 10)
        self.sub_oximeter = self.create_subscription(SensorData, 'sensor_data/oximeter', self.receive_datapoint, 10)
        self.sub_thermometer = self.create_subscription(SensorData, 'sensor_data/thermometer', self.receive_datapoint, 10)
        self.NOT_USED = ''
        
        self.target_system_publisher = self.create_publisher(TargetSystemData, 'target_system_data', 10)
        
        self.latest_data = {
            'abpd': -1.0,
            'abps': -1.0,
            'ecg': -1.0,
            'glucosemeter': -1.0,
            'oximeter': -1.0,
            'thermometer': -1.0,
        }

    def receive_datapoint(self, msg):
        if msg.sensor_type == '':
            self.get_logger().debug(f'null value received: {msg.sensor_type} with {msg.sensor_datapoint}')
        else:
            # Update the latest data for the corresponding sensor type
            self.latest_data[msg.sensor_type] = msg.sensor_datapoint
            self.get_logger().info(f'Received data from {msg.sensor_type}: {msg.sensor_datapoint}')
        
    def handle_risks(self, signal: str, avg: float):
        # Each field: signal (15), value (15), risk (20)
        if signal == 'thermometer':
            if avg < 32.0 or avg > 50.0:
                risk = 'high'
            elif 36.0 <= avg <= 37.99:
                risk = 'normal'
            else:
                risk = 'moderate'
            return risk, f"| {signal:<15} | {avg:>7.2f} °C      | Risk: {risk:<21} |\n"

        elif signal == 'abpd':
            if avg > 90:
                risk = 'high'
            elif 80 <= avg < 90:
                risk = 'moderate'
            else:
                risk = 'normal'
            return risk, f"| {signal:<15} | {avg:>7.2f} mmHg    | Risk: {risk:<21} |\n"

        elif signal == 'abps':
            if avg >= 140:
                risk = 'high'
            elif 120 <= avg < 140:
                risk = 'moderate'
            else:
                risk = 'normal'
            return risk, f"| {signal:<15} | {avg:>7.2f} mmHg    | Risk: {risk:<21} |\n"

        elif signal == 'ecg':
            if avg > 115 or avg < 70:
                risk = 'high'
            elif 85 <= avg <= 97:
                risk = 'normal'
            else:
                risk = 'moderate'
            return risk, f"| {signal:<15} | {avg:>7.2f} bpm     | Risk: {risk:<21} |\n"

        elif signal == 'glucosemeter':
            if avg < 40 or avg >= 120:
                risk = 'high'
            elif 55 <= avg < 96:
                risk = 'normal'
            else:
                risk = 'moderate'
            return risk, f"| {signal:<15} | {avg:>7.2f} mg/dL   | Risk: {risk:<21} |\n"

        elif signal == 'oximeter': 
            if avg <= 55:
                risk = 'high'
            elif 55 < avg <= 65:
                risk = 'moderate'
            else:
                risk = 'normal'
            return risk, f"| {signal:<15} | {avg:>7.2f}% SpO2   | Risk: {risk:<21} |\n"

        else:
            if avg < 30 or avg > 180:
                risk = 'high'
            elif 30 <= avg <= 50 or 130 <= avg <= 180:
                risk = 'moderate'
            else:
                risk = 'normal'
            return risk, f"| {signal:<15} | {avg:>7.2f}         | Risk: {risk:<21} |\n"


    
    def fuse_data(self):
        risk_levels = {}
        log_message = " \n"
        border = "+-----------------+-----------------+-----------------------------+"
        header = "| {0:<15} | {1:<15} | {2:<27} |".format("Vital Sign", "Value", "Risk Level")

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
                risk, log = self.handle_risks(signal, value)
                log_message += log
                risk_levels[signal] = risk

        log_message += "+-----------------+-----------------+-----------------------------+"
        self.get_logger().info(log_message)
        return risk_levels
    
    def detect(self):
        # Create a TargetSystemData message
        msg = TargetSystemData()

        # Add header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'central_hub'
        msg.header = header

        # Get risk levels from fuse_data()
        risks = self.fuse_data()
        
        self.emit_alert(risks)
        # Populate the message fields
        msg.trm_data = self.latest_data.get('thermometer', -1.0)
        msg.ecg_data = self.latest_data.get('ecg', -1.0)
        msg.oxi_data = self.latest_data.get('oximeter', -1.0)
        msg.abps_data = self.latest_data.get('abps', -1.0)
        msg.abpd_data = self.latest_data.get('abpd', -1.0)
        msg.glc_data = self.latest_data.get('glucosemeter', -1.0)

        msg.trm_risk = 1.0 if risks.get('thermometer') == 'high' else (0.5 if risks.get('thermometer') == 'moderate' else 0.0)
        msg.ecg_risk = 1.0 if risks.get('ecg') == 'high' else (0.5 if risks.get('ecg') == 'moderate' else 0.0)
        msg.oxi_risk = 1.0 if risks.get('oximeter') == 'high' else (0.5 if risks.get('oximeter') == 'moderate' else 0.0)
        msg.abps_risk = 1.0 if risks.get('abps') == 'high' else (0.5 if risks.get('abps') == 'moderate' else 0.0)
        msg.abpd_risk = 1.0 if risks.get('abpd') == 'high' else (0.5 if risks.get('abpd') == 'moderate' else 0.0)
        msg.glc_risk = 1.0 if risks.get('glucosemeter') == 'high' else (0.5 if risks.get('glucosemeter') == 'moderate' else 0.0)

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
        self.get_logger().info(f'Published TargetSystemData')    
    

    
    def emit_alert(self,risks):
        for signal, level in risks.items():
            if level == 'high':
                self.get_logger().fatal(f'[Emergency Detection]\n ALERT: High risk in {signal} detected!\n')
            elif level == 'moderate':
                self.get_logger().warning(f'[Emergency Detection]\n Warning: Abnormal {signal} levels.\n')
     


def main(args=None):
    rclpy.init(args=args)

    emergency_detector = CentralHub()

    # Run spin in a thread, make thread daemon so we don't have to join it to exit
    thread = threading.Thread(target=rclpy.spin, args=(emergency_detector, ), daemon=True)
    thread.start()
    rate = emergency_detector.create_rate(2) # 1 Hz

    while rclpy.ok():
        emergency_detector.detect()
        rate.sleep()

    sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
import threading
from rclpy.node import Node
from bsn_interfaces.msg import SensorData


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
        self.latest_data = {
            'abpd': self.NOT_USED,
            'abps': self.NOT_USED,
            'ecg': self.NOT_USED,
            'glucosemeter': self.NOT_USED,
            'oximeter': self.NOT_USED,
            'thermometer': self.NOT_USED,
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
            if 35 <= avg < 36:
                risk = 'moderate'
            elif 36 <= avg <= 38:
                risk = 'normal'
            elif 38 < avg <= 40:
                risk = 'moderate'
            else:
                risk = 'high'
            return risk, f"| {signal:<15} | {avg:>7.2f} °C      | Risk: {risk:<21} |\n"

        elif signal == 'abpd':
            if avg < 60 or avg > 120:
                risk = 'high'
            elif 60 <= avg <= 70 or 90 <= avg <= 120:
                risk = 'moderate'
            else:
                risk = 'normal'
            return risk, f"| {signal:<15} | {avg:>7.2f} mmHg    | Risk: {risk:<21} |\n"

        elif signal == 'abps':
            if avg < 90 or avg > 180:
                risk = 'high'
            elif 90 <= avg <= 100 or 140 <= avg <= 180:
                risk = 'moderate'
            else:
                risk = 'normal'
            return risk, f"| {signal:<15} | {avg:>7.2f} mmHg    | Risk: {risk:<21} |\n"

        elif signal == 'ecg':
            if avg < 50 or avg > 120:
                risk = 'high'
            elif 50 <= avg <= 60 or 100 <= avg <= 120:
                risk = 'moderate'
            else:
                risk = 'normal'
            return risk, f"| {signal:<15} | {avg:>7.2f} bpm     | Risk: {risk:<21} |\n"

        elif signal == 'glucosemeter':
            if avg < 70 or avg > 200:
                risk = 'high'
            elif 70 <= avg <= 80 or 140 <= avg <= 200:
                risk = 'moderate'
            else:
                risk = 'normal'
            return risk, f"| {signal:<15} | {avg:>7.2f} mg/dL   | Risk: {risk:<21} |\n"

        elif signal == 'oximeter':
            if avg < 90:
                risk = 'high'
            elif 90 <= avg <= 94:
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

    
    def emit_alert(self):
        risks = self.fuse_data()
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
        emergency_detector.fuse_data()
        emergency_detector.emit_alert()
        rate.sleep()

    sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
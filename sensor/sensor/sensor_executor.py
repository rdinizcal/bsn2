import rclpy
from sensor.sensor import Sensor
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('sensor_executor_node')

    # Declare the 'sensors' parameter as a list of dictionaries with sensor configurations
    node.declare_parameters(
        namespace='',
        parameters=[
                ('thermometer', ['', '1.0']),
                ('oximeter', ['', '1.0']),
                ('ecg', ['', '1.0']),
                ('abps', ['', '1.0']),
                ('abpd', ['', '1.0']),
                ('glucosemeter', ['', '1.0']),
            ]   
        )

    # Retrieve the sensor configuration from the parameters
    sensors_config = [
        ('thermometer', node.get_parameter('thermometer').value),
        ('oximeter', node.get_parameter('oximeter').value),
        ('ecg', node.get_parameter('ecg').value),
        ('abps', node.get_parameter('abps').value),
        ('abpd', node.get_parameter('abpd').value),
        ('glucosemeter', node.get_parameter('glucosemeter').value)
        ]

    if not any(vital_sign for vital_sign, _ in sensors_config):
        node.get_logger().warn('No sensors configured!')
        return

    executor = MultiThreadedExecutor()
    sensor_nodes = []

    # Loop through each sensor configuration
    
    for sensor_name, sensor_values in sensors_config:
        vital_sign = sensor_values[0]
        frequency = sensor_values[1]
            # If vital_sign is empty, skip this sensor (do not activate it)
        if not vital_sign:
            node.get_logger().info(f'Skipping {sensor_name} as vital_sign is empty.')
            continue
        # Create sensor node and set parameters if the vital_sign is valid
        node.get_logger().info(f'capturing sensor_name: {sensor_name}_node')
        sensor_node = Sensor(f'{sensor_name}_node', sensor_name)
        sensor_node.set_parameters([
            Parameter('vital_sign', Parameter.Type.STRING, vital_sign),
            Parameter('frequency', Parameter.Type.STRING, frequency),
        ])
        sensor_nodes.append(sensor_node)
        executor.add_node(sensor_node)
        # Run loop manually per sensor
        def spin_sensor(node):
            rate = node.create_rate(float(node.frequency))
            while rclpy.ok():
                datapoint = node.collect()
                datapoint = node.process(datapoint)
                node.transfer(datapoint)
                rate.sleep()
        import threading
        t = threading.Thread(target=spin_sensor, args=(sensor_node,), daemon=True)
        t.start()

    try:
        executor.spin()
    finally:
        for s in sensor_nodes:
            s.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

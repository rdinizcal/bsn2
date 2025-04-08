import rclpy

from sensor.sensor import Sensor
import threading

class OximeterSensor(Sensor):

    def __init__(self):
        super().__init__('oximeter_sensor', 'oximeter')

    def get_vital_sign(self) -> str:
        return 'oxigenation'


def main(args=None):
    rclpy.init(args=args)

    sensor = OximeterSensor()

    # Run spin in a thread, make thread daemon so we don't have to join it to exit
    thread = threading.Thread(target=rclpy.spin, args=(sensor, ), daemon=True)
    thread.start()
    rate = sensor.create_rate(1) # 1 Hz

    while rclpy.ok():
        datapoint = sensor.collect()
        datapoint = sensor.process(datapoint=datapoint)
        sensor.transfer(datapoint=datapoint)
        rate.sleep()

    sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
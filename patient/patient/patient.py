import threading
import random
import rclpy
from bsn_interfaces.srv import PatientData
from rclpy.node import Node


class Patient(Node):

    def __init__(self, parameters=None):
        super().__init__("patient", parameter_overrides=parameters or [])

        # Declare global frequency
        self.declare_parameter("frequency", 1.0)
        self.frequency = self.get_parameter("frequency").value

        # Declare vital sign list
        self.declare_parameter(
            "vitalSigns",
            ["temperature", "abps", "abpd", "heart_rate", "glucose", "oxigenation"],
        )
        self.vital_signs = self.get_parameter("vitalSigns").value
        self.change_rates = {}
        self.vital_Frequencies = {}
        self.offsets = {}
        self.transition_matrix_states = {}
        self.risk_ranges = {}

        change_frequency = 10
        self.PERIOD = 1.0 / change_frequency

        for vital in self.vital_signs:  # type: ignore
            # Declare change rate and offset
            self.declare_parameter(f"{vital}_Change", 0.1)
            self.declare_parameter(f"{vital}_Offset", 0.0)

            self.vital_Frequencies[vital] = 0
            self.change_rates[vital] = 1 / self.get_parameter(f"{vital}_Change").value
            self.offsets[vital] = self.get_parameter(f"{vital}_Offset").value

            self.transition_matrix_states[vital] = self._gen_sensor_state_matrix(vital)
            self.risk_ranges[vital] = self._set_up_sensor_risk_ranges(vital)

        self.vital_states = {key: 2 for key in self.vital_signs}
        self.vital_datapoints = {key: 0.0 for key in self.vital_signs}

        self.srv = self.create_service(PatientData, "get_sensor_reading", self.get_data)

    def _gen_sensor_state_matrix(self, vital: str):
        state_dict = {}
        for i in range(5):
            self.declare_parameter(f"{vital}_State{i}", [0.0, 0.0, 0.0, 0.0, 0.0])
            state_dict[i] = self.get_parameter(f"{vital}_State{i}").value
            if sum(state_dict[i]) == 0.0:
                self.get_logger().warn(
                    f"State {i} for {vital} is all zeros and will be ignored."
                )
                state_dict[i] = None  # Mark the state as invalid
            elif sum(state_dict[i]) > 1.0:
                self.get_logger().warn(f"State {i} for {vital} does is higher than 1")
        self.get_logger().warn(f"{vital} transition matrix: {state_dict}")
        return state_dict

    def _set_up_sensor_risk_ranges(self, vital: str):
        risks = {}
        labels = ["HighRisk0", "MidRisk0", "LowRisk", "MidRisk1", "HighRisk1"]
        for i in range(len(labels)):
            param_name = f"{vital}_{labels[i]}"
            self.declare_parameter(param_name, [-1.0, -1.0])
            risks[i] = self.get_parameter(param_name).value
        self.get_logger().debug(f"risks for {vital} is {risks}")
        return risks

    def _should_change_state(self, vital_sign):
        """Check if it's time to change state for a given vital sign based on change rate and offset.

        Args:
            vital_sign (str): The vital sign to check

        Returns:
            bool: True if state should change, False otherwise
        """
        accumulated_time = self.vital_Frequencies[vital_sign]
        change_threshold = self.change_rates[vital_sign] + self.offsets[vital_sign]

        return accumulated_time >= change_threshold

    def gen_data(self):
        for vital_sign in self.vital_signs:
            self.vital_Frequencies[vital_sign] += self.PERIOD
            curr_state = self.vital_states[vital_sign]
            # Check if the accumulated time exceeds the change rate + offset
            self.get_logger().debug(
                f"vital_Frequencies: {self.vital_Frequencies[vital_sign]}, change_rates: {self.change_rates[vital_sign]}, offsets: {self.offsets[vital_sign]}"
            )

            if self._should_change_state(vital_sign):
                self.get_logger().debug(
                    f"transition matrix: {self.transition_matrix_states[vital_sign]}, curr_state: {curr_state}"
                )
                probs = self.transition_matrix_states[vital_sign][curr_state]

                if probs is None:
                    continue

                next_state = self._determine_possible_next_state(probs, vital_sign)

                if -1.0 in self.risk_ranges[vital_sign][next_state]:
                    continue
                elif next_state != curr_state:
                    self.get_logger().info(
                        f"State transition for {vital_sign}: {curr_state} -> {next_state}"
                    )

                # Calculate the datapoint for the next state
                self._calculate_datapoint(vital_sign, next_state)
                # Reset the accumulated time to the offset value
                self.vital_Frequencies[vital_sign] = self.offsets[vital_sign]
            else:
                self._calculate_datapoint(vital_sign, curr_state)

    def _determine_possible_next_state(self, probs, vital_sign: str):
        x = random.random()
        if x <= probs[0]:
            self.vital_states[vital_sign] = 0
        elif x <= sum(probs[:2]):
            self.vital_states[vital_sign] = 1
        elif x <= sum(probs[:3]):
            self.vital_states[vital_sign] = 2
        elif x <= sum(probs[:4]):
            self.vital_states[vital_sign] = 3
        else:
            self.vital_states[vital_sign] = 4
        return self.vital_states[vital_sign]

    def _calculate_datapoint(self, vital_sign: str, curr_state: int):
        range_min, range_max = self.risk_ranges[vital_sign][curr_state]

        if range_min != -1.0 and range_max != -1.0 and range_min <= range_max:
            self.vital_datapoints[vital_sign] = random.uniform(range_min, range_max)
            self.get_logger().debug(
                f"Generated {vital_sign}: {self.vital_datapoints[vital_sign]:.2f}"
            )
        else:
            self.get_logger().fatal(
                f"Invalid risk range for {vital_sign} state {curr_state}: [{range_min}, {range_max}]"
            )
            self.vital_datapoints[vital_sign] = -1.0

    def get_data(self, request, response):
        vital = request.vital_sign.lower()
        if vital not in self.vital_datapoints:
            self.get_logger().warn(f"Requested unknown vital sign: {vital}")
            response.datapoint = -1.0
        else:
            response.datapoint = float(self.vital_datapoints[vital])
            self.get_logger().debug(f"Request: ({vital}) -> {response.datapoint:.4f}")
        return response

    def spin_patient(self):
        rate = self.create_rate(self.frequency)
        while rclpy.ok():
            self.gen_data()
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    patient = Patient()

    # Run spin in a thread, make thread daemon so we don't have to join it to exit
    thread = threading.Thread(target=rclpy.spin, args=(patient,), daemon=True)
    thread.start()

    try:
        patient.spin_patient()
    finally:
        patient.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

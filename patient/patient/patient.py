"""
Patient simulation node for Body Sensor Network.

This module provides a realistic patient simulation with vital sign generation
based on Markov chain state transitions. It simulates various health conditions
and provides sensor data through ROS services for testing and demonstration
of the Body Sensor Network system.
"""

import threading
import random
import rclpy
from bsn_interfaces.srv import PatientData
from rclpy.node import Node


class Patient(Node):
    """
    Patient simulator node with Markov-based vital sign generation.
    
    This class simulates a patient with multiple vital signs that change over time
    according to configurable Markov chain transition matrices. Each vital sign
    has its own state space representing different health conditions, from low-risk
    normal values to high-risk critical values.
    
    The simulator provides realistic transitions between health states with
    configurable probabilities and timing, making it suitable for testing
    emergency detection algorithms and sensor fusion systems.
    
    Attributes:
        frequency (float): Update frequency for vital sign generation.
        vital_signs (list): List of vital sign names to simulate.
        change_rates (dict): Change rate thresholds for each vital sign.
        vital_Frequencies (dict): Accumulated time counters for state transitions.
        offsets (dict): Initial delay offsets before transitions can occur.
        transition_matrix_states (dict): Markov transition matrices for each vital.
        risk_ranges (dict): Value ranges for each risk state and vital sign.
        vital_states (dict): Current state (0-4) for each vital sign.
        vital_datapoints (dict): Current generated values for each vital sign.
        PERIOD (float): Time increment per generation cycle.
        srv: ROS service server for providing patient data.
        
    Examples:
        Basic usage:
        ```python
        import rclpy
        from patient.patient import Patient
        
        rclpy.init()
        patient = Patient()
        patient.spin_patient()
        ```
        
        With custom parameters:
        ```python
        from rclpy.parameter import Parameter
        
        params = [
            Parameter('frequency', 10.0),
            Parameter('temperature_Change', 0.05)
        ]
        patient = Patient(parameters=params)
        ```
    """

    def __init__(self, parameters=None):
        """
        Initialize the patient simulator node.
        
        Sets up vital sign simulation parameters, Markov transition matrices,
        risk ranges, and the ROS service for providing patient data.
        
        Args:
            parameters (list, optional): List of ROS Parameter objects for
                configuration override. Defaults to None.
        """
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

        change_frequency = 5
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
        """
        Generate Markov transition matrix for a vital sign.
        
        Creates a 5-state transition matrix where each state represents
        a different risk level (HighRisk0, MidRisk0, LowRisk, MidRisk1, HighRisk1).
        Probabilities are loaded from ROS parameters and validated.
        
        Args:
            vital (str): Name of the vital sign to generate matrix for.
            
        Returns:
            dict: Dictionary mapping state numbers (0-4) to probability lists,
                  or None for invalid states with all-zero probabilities.
                  
        Examples:
            ```python
            # For temperature vital sign
            matrix = patient._gen_sensor_state_matrix("temperature")
            # matrix[2] might be [0.0, 0.04, 0.85, 0.11, 0.0]
            # indicating 85% chance to stay in state 2 (normal)
            ```
        """
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
        """
        Configure value ranges for each risk state of a vital sign.
        
        Sets up the numerical ranges that correspond to each health risk
        level. Values are generated within these ranges when the patient
        is in the corresponding state.
        
        Args:
            vital (str): Name of the vital sign to configure ranges for.
            
        Returns:
            dict: Dictionary mapping state numbers (0-4) to [min, max] ranges.
                  Invalid ranges are marked with [-1.0, -1.0].
                  
        Examples:
            ```python
            # For temperature
            ranges = patient._set_up_sensor_risk_ranges("temperature")
            # ranges[2] might be [36.0, 37.99] for normal temperature
            # ranges[4] might be [41.0, 50.0] for high fever
            ```
        """
        risks = {}
        labels = ["HighRisk0", "MidRisk0", "LowRisk", "MidRisk1", "HighRisk1"]
        for i in range(len(labels)):
            param_name = f"{vital}_{labels[i]}"
            self.declare_parameter(param_name, [-1.0, -1.0])
            risks[i] = self.get_parameter(param_name).value

        return risks

    def _should_change_state(self, vital_sign):
        """
        Check if it's time to change state for a given vital sign.
        
        Determines whether enough time has accumulated based on the change
        rate and offset parameters to trigger a potential state transition
        according to the Markov chain model.
        
        Args:
            vital_sign (str): The vital sign to check for state change readiness.
            
        Returns:
            bool: True if state should change based on timing, False otherwise.
            
        Notes:
            The decision is based on accumulated time since last transition
            compared to (change_rate + offset) threshold. Offsets provide
            initial delays before first transitions can occur.
        """
        accumulated_time = self.vital_Frequencies[vital_sign]
        change_threshold = self.change_rates[vital_sign] + self.offsets[vital_sign]
        self.get_logger().debug(
            f"Accumulated time for {vital_sign}: {accumulated_time}, change threshold: {change_threshold}"
        )
        return accumulated_time >= change_threshold

    def gen_data(self):
        """
        Generate new vital sign data for all configured vital signs.
        
        This is the main data generation method that:
        1. Increments time counters for all vital signs
        2. Checks if state transitions should occur based on timing
        3. Performs probabilistic state transitions using Markov chains
        4. Generates new data values within the current state's ranges
        5. Resets timing counters after state changes
        
        The method ensures realistic patient behavior by simulating
        gradual health state changes over time rather than random jumps.
        
        Notes:
            Called periodically by the main simulation loop at the configured
            frequency. Each call advances the simulation by PERIOD time units.
        """
        for vital_sign in self.vital_signs:
            self.vital_Frequencies[vital_sign] += self.PERIOD
            curr_state = self.vital_states[vital_sign]
            # Check if the accumulated time exceeds the change rate + offset
            # self.get_logger().debug(
            #    f"vital_Frequencies: {self.vital_Frequencies[vital_sign]}, change_rates: {self.change_rates[vital_sign]}, offsets: {self.offsets[vital_sign]}"
            # )

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
        """
        Determine the next state using probabilistic selection.
        
        Uses the Markov chain transition probabilities to randomly select
        the next state. The selection is based on cumulative probability
        ranges, ensuring proper distribution according to the configured
        transition matrix.
        
        Args:
            probs (list): List of 5 transition probabilities for states 0-4.
            vital_sign (str): Name of vital sign being transitioned.
            
        Returns:
            int: The selected next state (0-4).
            
        Examples:
            ```python
            # With probabilities [0.1, 0.2, 0.4, 0.2, 0.1]
            # Random value 0.25 would select state 1
            # Random value 0.65 would select state 3
            next_state = patient._determine_possible_next_state(probs, "temperature")
            ```
        """
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
        """
        Generate a random value within the current state's range.
        
        Calculates a realistic vital sign measurement by randomly sampling
        from the configured range for the current health state. Handles
        invalid ranges by setting error values.
        
        Args:
            vital_sign (str): Name of the vital sign to generate data for.
            curr_state (int): Current health state (0-4) for the vital sign.
            
        Notes:
            Generated values are stored in vital_datapoints dictionary.
            Invalid ranges (containing -1.0) result in -1.0 datapoint
            to indicate measurement unavailable.
            
        Examples:
            ```python
            # For temperature in state 2 (normal) with range [36.0, 37.99]
            patient._calculate_datapoint("temperature", 2)
            # vital_datapoints["temperature"] might become 36.8
            ```
        """
        range_min, range_max = self.risk_ranges[vital_sign][curr_state]

        if range_min != -1.0 and range_max != -1.0 and range_min <= range_max:
            self.vital_datapoints[vital_sign] = random.uniform(range_min, range_max)
        else:
            self.get_logger().fatal(
                f"Invalid risk range for {vital_sign} state {curr_state}: [{range_min}, {range_max}]"
            )
            self.vital_datapoints[vital_sign] = -1.0

    def get_data(self, request, response):
        """
        ROS service handler for providing patient vital sign data.
        
        Responds to service requests from sensors or other components
        that need current patient vital sign measurements. Handles
        both valid and invalid vital sign requests.
        
        Args:
            request (PatientData.Request): Service request containing
                the vital_sign field specifying which measurement is needed.
            response (PatientData.Response): Service response object to
                populate with the datapoint field.
                
        Returns:
            PatientData.Response: Response object with datapoint set to
                the current vital sign value or -1.0 for unknown vitals.
                
        Examples:
            ```python
            # Service call from sensor:
            # request.vital_sign = "temperature"
            # response.datapoint = 37.2  # Current temperature value
            ```
        """
        vital = request.vital_sign.lower()
        if vital not in self.vital_datapoints:
            self.get_logger().warn(f"Requested unknown vital sign: {vital}")
            response.datapoint = -1.0
        else:
            response.datapoint = float(self.vital_datapoints[vital])
        return response

    def spin_patient(self):
        """
        Main simulation loop for continuous patient data generation.
        
        Runs the patient simulation at the configured frequency, continuously
        generating new vital sign data based on Markov state transitions.
        This method blocks and runs until ROS is shut down.
        
        The loop maintains precise timing using ROS rate control to ensure
        consistent simulation behavior regardless of processing time variations.
        
        Notes:
            This method should be called after ROS spinning is started in
            a separate thread to ensure proper service handling while
            the simulation runs.
        """
        rate = self.create_rate(self.frequency)
        while rclpy.ok():
            self.gen_data()
            rate.sleep()


def main(args=None):
    """
    Main entry point for the patient simulation node.
    
    Initializes ROS, creates the patient simulator, and runs both the
    ROS communication loop and the simulation loop in coordinated threads.
    The ROS spinning runs in a daemon thread to handle service requests
    while the main thread runs the patient simulation.
    
    Args:
        args: Command line arguments passed to ROS initialization.
        
    Examples:
        ```python
        # Run with default parameters
        python -m patient.patient
        
        # Run with ROS arguments
        python -m patient.patient --ros-args -p frequency:=10.0
        ```
    """
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

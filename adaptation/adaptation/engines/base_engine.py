import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from abc import ABC, abstractmethod
from typing import Dict, List, Optional
import time


from adaptation.model.formula import Formula


# ROS2 interfaces
from bsn_interfaces.srv import DataAccessRequest, EngineRequest
from bsn_interfaces.msg import Exception as BSNException


class Engine(Node, ABC):
    """
    Abstract Engine class - ROS2 implementation matching BSN1 Engine
    Base class for all adaptation engines (ReliabilityEngine, CostEngine)
    """

    def __init__(self, node_name: str):
        super().__init__(node_name)

        # Core attributes matching Engine.hpp
        self.qos_attribute = ""
        self.info_quant = 0.0
        self.monitor_freq = 1.0
        self.actuation_freq = 1.0

        # Formula and strategy matching Engine.hpp
        self.target_system_model: Optional[Formula] = None
        self.strategy: Dict[str, float] = {}
        self.priority: Dict[str, int] = {}
        self.deactivated_components: Dict[str, int] = {}

        # ROS2 interfaces
        #self.data_access_client: Optional[rclpy.client.Client] = None
        self.exception_subscriber: Optional[rclpy.subscription.Subscription] = None
        self.enactor_server: Optional[rclpy.service.Service] = None

        # Initialize parameters
        self.declare_parameter("qos_attribute", "")
        self.declare_parameter("info_quant", 0.0)
        self.declare_parameter("monitor_freq", 1.0)
        self.declare_parameter("actuation_freq", 1.0)

        self.get_logger().info(f"Engine {node_name} initialized")

    def setup(self):
        """Setup engine"""
        # Get parameters
        self.qos_attribute = self.get_parameter("qos_attribute").value
        self.info_quant = self.get_parameter("info_quant").value
        self.monitor_freq = self.get_parameter("monitor_freq").value
        self.actuation_freq = self.get_parameter("actuation_freq").value

        # Setup ROS2 interfaces
        self._setup_ros_interfaces()

        # Load formula
        formula_str = ""
        while not formula_str:
            formula_str = self.fetch_formula(self.qos_attribute)
            if not formula_str:
                self.get_logger().warn("Waiting for formula...")
                time.sleep(1.0)

        self.setup_formula(formula_str)

        self.get_logger().info(f"Engine setup complete - QoS: {self.qos_attribute}")

    def _setup_ros_interfaces(self):
        """Setup ROS2 interfaces"""
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
        )

        # DataAccess client
        self.data_access_client = self.create_client(
            DataAccessRequest, "DataAccessRequest"
        )

        # Exception subscriber matching Engine.cpp body()
        self.exception_subscriber = self.create_subscription(
            BSNException, "exception", self.receive_exception, qos_profile
        )

        # Engine service matching Engine.cpp setUp()
        self.enactor_server = self.create_service(
            EngineRequest, "EngineRequest", self.send_adaptation_parameter
        )

    def fetch_formula(self, name: str) -> str:
        """Fetch formula from DataAccess"""
        try:
            #if not self.data_access_client.wait_for_service(timeout_sec=5.0):
            #    self.get_logger().warn("DataAccess service not available")
            #    return ""

            request = DataAccessRequest.Request()
            request.name = "/engine"
            request.query = f"{name}_formula"

            future = self.data_access_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result():
                formula_str = future.result()
                if not formula_str:
                    self.get_logger().error("ERROR: Empty formula string received.")
                return formula_str
            else:
                self.get_logger().error(
                    "Tried to fetch formula string, but Data Access is not responding."
                )
                return ""

        except Exception as e:
            self.get_logger().error(f"Error fetching formula: {e}")
            return ""

    def setup_formula(self, formula_str: str):
        """Setup formula - matching Engine.cpp setUp_formula()"""
        try:
            # Create Formula object (already implemented in BSN2)
            self.target_system_model = Formula(formula_str)

            # Extract terms that will compose the strategy
            terms = self.target_system_model.get_terms()

            # Initialize strategy and priority
            self.strategy = self.initialize_strategy(terms)
            self.priority = self.initialize_priority(terms)

            # Initialize the target system model
            self.calculate_qos(self.target_system_model, self.strategy)

            self.get_logger().info(f"Formula setup complete with {len(terms)} terms")

        except Exception as e:
            self.get_logger().error(f"Error setting up formula: {e} \nFormula: {formula_str}")

    def calculate_qos(self, model: Formula, conf: Dict[str, float]) -> float:
        """Calculate QoS - matching Engine.cpp calculate_qos()"""
        try:
            model.set_term_value_map_dict(conf)
            return model.evaluate()
        except Exception as e:
            self.get_logger().error(f"Error calculating QoS: {e}")
            return 0.0

    def receive_exception(self, msg: BSNException):
        """Receive exception - matching Engine.cpp receiveException()"""
        try:
            content = msg.content
            if "=" in content:
                param = content.split("=", 1)

                # Process component name (matching C++ logic)
                first = param[0].upper().lstrip("/")
                # Insert underscore before 'T' (matching C++ logic)
                t_index = first.find("T")
                if t_index != -1:
                    first = first[:t_index] + "_" + first[t_index:]

                first = self.get_prefix() + first

                if first in self.priority:
                    self.priority[first] += int(param[1])
                    # Clamp between 0 and 100
                    self.priority[first] = max(0, min(100, self.priority[first]))

                    self.get_logger().info(
                        f"Updated priority: {first} = {self.priority[first]}"
                    )

        except Exception as e:
            self.get_logger().error(f"Error processing exception: {e}")

    def send_adaptation_parameter(
        self, request: EngineRequest.Request, response: EngineRequest.Response
    ):
        """Send adaptation parameter - matching Engine.cpp sendAdaptationParameter()"""
        try:
            response.content = self.qos_attribute
            return response
        except Exception as e:
            self.get_logger().error(f"Error sending adaptation parameter: {e}")
            response.content = ""
            return response

    def body(self):
        """Main execution loop - matching Engine.cpp body()"""
        rate = self.create_rate(self.monitor_freq)
        update_counter = 0

        self.get_logger().info("Starting engine body loop...")

        try:
            while rclpy.ok():
                update_counter += 1

                # Reload formula every 10 seconds (matching C++ logic)
                if update_counter >= self.monitor_freq * 10:
                    update_counter = 0
                    formula_str = self.fetch_formula(self.qos_attribute)
                    if formula_str:
                        self.setup_formula(formula_str)

                # Execute MAPE-K monitor phase
                self.monitor()

                # Spin once to handle callbacks
                rclpy.spin_once(self, timeout_sec=0.01)

                rate.sleep()

        except KeyboardInterrupt:
            self.get_logger().info("Engine body loop interrupted")
        except Exception as e:
            self.get_logger().error(f"Error in engine body: {e}")

    # Abstract methods that must be implemented by subclasses

    @abstractmethod
    def get_prefix(self) -> str:
        """Get engine prefix - matching Engine.cpp get_prefix()"""
        pass

    @abstractmethod
    def initialize_strategy(self, terms: List[str]) -> Dict[str, float]:
        """Initialize strategy - matching Engine.cpp initialize_strategy()"""
        pass

    @abstractmethod
    def initialize_priority(self, terms: List[str]) -> Dict[str, int]:
        """Initialize priority - matching Engine.cpp initialize_priority()"""
        pass

    @abstractmethod
    def monitor(self):
        """MAPE-K Monitor phase - matching Engine.cpp monitor()"""
        pass

    @abstractmethod
    def analyze(self):
        """MAPE-K Analyze phase - matching Engine.cpp analyze()"""
        pass

    @abstractmethod
    def plan(self):
        """MAPE-K Plan phase - matching Engine.cpp plan()"""
        pass

    @abstractmethod
    def execute(self):
        """MAPE-K Execute phase - matching Engine.cpp execute()"""
        pass

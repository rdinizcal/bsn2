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


class BasePublisher(ABC):
    def __init__(self, node: Node):
        """
        Abstract base class for BSN data publishers.
        
        This class provides a common interface and shared functionality for
        all BSN data publishers, including lifecycle management, data
        publishing, and exception handling.
        
        Args:
            node (Node): Reference to the ROS node using this publisher.
        """
        self.node = node
        self.publisher = None
        self.active = False
        self.config = node.config  # Access configuration from the node
        self.lifecycle_manager = node.lifecycle_manager  # Access lifecycle manager
        
        # Publisher parameters (to be set by subclasses)
        self.topic_name: str = ""
        self.qos_profile: QoSProfile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Exception publisher
        self.exception_publisher = self.node.create_publisher(
            BSNException,
            "bsn_exceptions",
            10
        )
        
        # Register with lifecycle manager
        self.lifecycle_manager.register_component(self)
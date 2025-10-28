"""
Configuration management for central hub node.

This module implements the Central Hub config manager using the shared
`ConfigManagerBase` so common parameters (battery, frequency, adaptation)
are declared and read in one place.
"""

from shared_components.core.config_manager_base import ConfigManagerBase


class ConfigManager(ConfigManagerBase):
    """Central Hub configuration manager using shared base behavior.

    The base class declares and reads shared parameters. This class only
    applies hub-specific adjustments (component identity and defaults).
    """

    def __init__(self, node):
        # Initialize shared parameters via base
        super().__init__(node)
        # Central-hub specific adjustments and validate
        self.load(node)
        try:
            self.validate()
        except ValueError as e:
            node.get_logger().error(f"Configuration error: {e}")
            raise

    def load(self, node) -> None:
        """Apply central-hub specific defaults.

        This method is intentionally small because the base already handles
        the common declarations and reads.
        """
        # Explicit identity for logging and other consumers

        # Ensure a sensible battery id for hub if none provided
        if not self.battery_id:
            self.battery_id = "hub_battery"

        node.get_logger().info(f"Initialized Central Hub with frequency: {self.frequency}Hz")
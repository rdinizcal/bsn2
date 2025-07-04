"""
Battery simulation component for Body Sensor Network energy management.

This module provides a realistic battery simulation with configurable capacity,
consumption, and generation rates. It supports energy-aware computing by tracking
battery levels and providing consumption/recharge capabilities for BSN components.
"""


class Battery:
    """
    Energy simulation component for BSN nodes with realistic battery behavior.
    
    This class simulates a rechargeable battery with configurable capacity,
    current level, and energy consumption/generation units. It provides
    realistic energy management for BSN components including sensors,
    central hub, and monitoring systems.
    
    The battery enforces realistic constraints including capacity limits,
    non-negative charge levels, and unit-based energy operations to ensure
    consistent energy simulation across the BSN system.
    
    Attributes:
        id (str): Unique identifier for the battery instance.
        capacity (float): Maximum battery capacity (typically 100%).
        current_level (float): Current battery charge level (0 to capacity).
        unit (float): Base energy unit for consumption/generation operations.
        
    Examples:
        Basic battery usage:
        ```python
        from shared_components.battery import Battery
        
        # Create battery with default settings
        battery = Battery()
        
        # Custom battery configuration
        battery = Battery("sensor_bat", 200.0, 150.0, 2.0)
        
        # Energy operations
        battery.consume(1.5)  # Use 1.5x unit energy
        battery.generate(2.0) # Recharge 2.0x unit energy
        ```
        
        Energy monitoring:
        ```python
        battery = Battery("hub_battery", 100.0, 75.0, 1.0)
        print(f"Level: {battery.current_level}%")
        
        # Check if operation is possible
        if battery.current_level > 10.0:
            battery.consume(5.0)
        ```
    """
    
    def __init__(self, id_name="sensor_battery", capacity=100.0, current_level=100.0, unit=1.0):
        """
        Initialize a battery with specific energy parameters.
        
        Creates a battery instance with configurable capacity, initial charge,
        and energy unit size. Validates all parameters to ensure realistic
        battery behavior and prevents invalid configurations.
        
        Args:
            id_name (str): Unique battery identifier for tracking purposes.
                          Defaults to "sensor_battery".
            capacity (float): Maximum battery capacity in percentage or units.
                             Must be positive. Defaults to 100.0.
            current_level (float): Initial battery charge level. Must be
                                  non-negative and not exceed capacity.
                                  Defaults to 100.0 (fully charged).
            unit (float): Base energy unit for consumption/generation operations.
                         Must be positive and not exceed capacity. Defaults to 1.0.
                         
        Raises:
            ValueError: If capacity is negative or zero.
            ValueError: If current_level is negative or exceeds capacity.
            ValueError: If unit is negative, zero, or exceeds capacity.
            
        Examples:
            ```python
            # Default battery (100% capacity, fully charged, 1.0 unit)
            battery = Battery()
            
            # Custom sensor battery
            sensor_bat = Battery("temp_sensor", 80.0, 60.0, 0.5)
            
            # Hub battery with higher capacity
            hub_bat = Battery("central_hub", 200.0, 180.0, 2.0)
            ```
        """
        if capacity <= 0:
            raise ValueError("Capacity should not be negative or null")
        
        if current_level < 0 or current_level > capacity:
            raise ValueError("Current level should not be negative or null nor bigger than the capacity")
            
        if unit < 0 or unit > capacity:
            raise ValueError("The resolution should not be negative or null nor bigger than the capacity")
            
        self.id = id_name
        self.capacity = capacity
        self.current_level = current_level
        self.unit = unit
    
    def consume(self, multiplier=1.0):
        """
        Consume battery energy based on unit and multiplier.
        
        Reduces the current battery level by the specified amount,
        calculated as unit * multiplier. Ensures battery level never
        goes below zero to maintain realistic battery behavior.
        
        Args:
            multiplier (float): Energy consumption multiplier. Defaults to 1.0.
                               Higher values consume more energy, fractional
                               values consume less than one unit.
                               
        Examples:
            ```python
            battery = Battery("sensor", 100.0, 50.0, 2.0)
            
            # Consume one unit (2.0 energy)
            battery.consume()        # Level: 48.0
            
            # Consume 1.5 units (3.0 energy)
            battery.consume(1.5)     # Level: 45.0
            
            # Large consumption (limited by current level)
            battery.consume(100.0)   # Level: 0.0 (not negative)
            ```
        """
        self.current_level -= self.unit * multiplier
        if self.current_level < 0:
            self.current_level = 0
    
    def generate(self, multiplier=1.0):
        """
        Generate/recharge battery energy based on unit and multiplier.
        
        Increases the current battery level by the specified amount,
        calculated as unit * multiplier. Ensures battery level never
        exceeds capacity to maintain realistic battery constraints.
        
        Args:
            multiplier (float): Energy generation multiplier. Defaults to 1.0.
                               Higher values recharge faster, fractional
                               values provide slower charging.
                               
        Examples:
            ```python
            battery = Battery("sensor", 100.0, 20.0, 5.0)
            
            # Generate one unit (5.0 energy)
            battery.generate()       # Level: 25.0
            
            # Fast charging (2.5 units = 12.5 energy)
            battery.generate(2.5)    # Level: 37.5
            
            # Overcharge protection (limited by capacity)
            battery.generate(20.0)   # Level: 100.0 (not over capacity)
            ```
        """
        self.current_level += self.unit * multiplier
        if self.current_level > self.capacity:
            self.current_level = self.capacity
        
    def __str__(self):
        """
        String representation of battery state.
        
        Provides a human-readable representation of the battery including
        its identifier, capacity, current level, and energy unit for
        debugging and monitoring purposes.
        
        Returns:
            str: Formatted string containing battery ID, capacity, current
                 level, and unit separated by spaces.
                 
        Examples:
            ```python
            battery = Battery("hub_bat", 100.0, 75.5, 1.0)
            print(battery)  # "Battery: hub_bat 100.0 75.5 1.0"
            
            # Useful for logging
            logger.info(f"Battery status: {battery}")
            ```
        """
        return f"Battery: {self.id} {self.capacity} {self.current_level} {self.unit}"
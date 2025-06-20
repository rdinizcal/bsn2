import pytest
from system_monitor.battery import Battery

class TestBattery:
    """Tests for the Battery class"""
    
    def test_init_valid_parameters(self):
        """Test battery initialization with valid parameters"""
        # Test with default parameters
        battery = Battery()
        assert battery.id == "sensor_battery"
        assert battery.capacity == 100.0
        assert battery.current_level == 100.0
        assert battery.unit == 1.0
        
        # Test with custom parameters
        battery = Battery("custom_battery", 200.0, 150.0, 2.0)
        assert battery.id == "custom_battery"
        assert battery.capacity == 200.0
        assert battery.current_level == 150.0
        assert battery.unit == 2.0
    
    def test_init_invalid_parameters(self):
        """Test battery initialization with invalid parameters"""
        # Test with negative capacity
        with pytest.raises(ValueError):
            Battery(capacity=-10.0)
        
        # Test with zero capacity
        with pytest.raises(ValueError):
            Battery(capacity=0.0)
        
        # Test with negative current_level
        with pytest.raises(ValueError):
            Battery(current_level=-10.0)
        
        # Test with current_level > capacity
        with pytest.raises(ValueError):
            Battery(capacity=100.0, current_level=150.0)
        
        # Test with negative unit
        with pytest.raises(ValueError):
            Battery(unit=-1.0)
        
        # Test with unit > capacity
        with pytest.raises(ValueError):
            Battery(capacity=10.0, unit=20.0)
    
    def test_consume(self):
        """Test battery consumption"""
        battery = Battery(capacity=100.0, current_level=50.0, unit=10.0)
        
        # Test normal consumption
        battery.consume()
        assert battery.current_level == 40.0
        
        # Test consumption with multiplier
        battery.consume(2.0)
        assert battery.current_level == 20.0
        
        # Test consumption to zero
        battery.consume(3.0)  # Should consume 30.0, but only 20.0 is available
        assert battery.current_level == 0.0
        
        # Test consumption when already zero
        battery.consume()
        assert battery.current_level == 0.0
    
    def test_generate(self):
        """Test battery generation/recharge"""
        battery = Battery(capacity=100.0, current_level=50.0, unit=10.0)
        
        # Test normal generation
        battery.generate()
        assert battery.current_level == 60.0
        
        # Test generation with multiplier
        battery.generate(2.0)
        assert battery.current_level == 80.0
        
        # Test generation to capacity
        battery.generate(3.0)  # Should generate 30.0, but only 20.0 is needed to reach capacity
        assert battery.current_level == 100.0
        
        # Test generation when already at capacity
        battery.generate()
        assert battery.current_level == 100.0
    
    def test_string_representation(self):
        """Test string representation of battery"""
        battery = Battery("test_id", 100.0, 75.0, 5.0)
        string_repr = str(battery)
        
        # Check that all values are in the string representation
        assert "test_id" in string_repr
        assert "100.0" in string_repr
        assert "75.0" in string_repr
        assert "5.0" in string_repr
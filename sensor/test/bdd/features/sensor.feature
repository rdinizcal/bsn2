Feature: Sensor data processing
  Scenario: Calculate moving average
    Given the Sensor node is running
    When the Sensor collects a data point
    Then the moving average is calculated
    And the data is published
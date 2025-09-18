Feature: Sensor data publishing

    Scenario: Sensor collects and publishes valid data
        Given the sensor node is online
        When the sensor collects data
        Then the sensor publishes the data
        And the data is within valid range
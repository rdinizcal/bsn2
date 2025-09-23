Feature: Sensor data publishing

    Scenario: Sensor collects and publishes valid data
        Given the sensor node is online
        When the sensor process data
        Then the data is within valid range
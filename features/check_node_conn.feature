Feature: ensure /hub_node connects with /sensor_1 and /sensor_2

	Scenario: Ensure that /central_hub_node is connected to /sensor_1,/sensor_2
		Given the /central_hub_node node is online
  		And the /thermometer_node node is online
    	And the /oximeter_node node is online
		When I check if node /thermometer_node publishes to /sensor_data/thermometer,/registration_status_topic
  		And I check if node /oximeter_node publishes to /sensor_data/oximeter,/registration_status_topic
		Then /central_hub_node should have /sensor_data/thermometer subscribed 
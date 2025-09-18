import pytest
from pytest_bdd import scenarios, given, when, then
from fixtures import patient_node_bdd
# Link to your feature file(s)
scenarios('../features/check_bsn.feature')

@given('that all sensors and central hub nodes are online')
def all_nodes_online(patient_node_bdd):
    # Simulate checking if nodes are online
    assert patient_node_bdd is not None

@when('I listen to sensors data')
def listen_sensors_data(patient_node_bdd):
    # Simulate listening
    pass

@then('Sensors will process the risks')
def sensors_process_risks(patient_node_bdd):
    # Assert risk processing
    pass

@then('Central hub will process the risk')
def central_hub_process_risk(patient_node_bdd):
    # Assert hub processing
    pass
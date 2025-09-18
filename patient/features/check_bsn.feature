Feature: BSN patient risk processing

    Scenario: Sensors and central hub process risk data
        Given that all sensors and central hub nodes are online
        When I listen to sensors data
        Then Sensors will process the risks
        And Central hub will process the risk
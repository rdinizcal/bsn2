from adaptation.data_access.data_access import DataAccess

class SharedDataAccessTests:
    """Shared test methods for DataAccess node testing"""

    @staticmethod
    def assert_data_access_initialized(data_access: DataAccess):
        assert hasattr(data_access, 'logical_clock')
        assert hasattr(data_access, 'frequency')
        assert hasattr(data_access, 'buffer_size')
        assert hasattr(data_access, 'time_window')
        assert isinstance(data_access.status, dict)
        assert isinstance(data_access.events, dict)
        assert isinstance(data_access.contexts, dict)
        assert isinstance(data_access.components_reliabilities, dict)
        assert isinstance(data_access.components_batteries, dict)

    @staticmethod
    def assert_goal_model_loaded(data_access: DataAccess):
        assert hasattr(data_access, 'goal_tree')
        assert hasattr(data_access, 'component_mapping')
        expected_sensors = ["g3t1_1", "g3t1_2", "g3t1_3", "g3t1_4", "g3t1_5", "g3t1_6"]
        for sensor in expected_sensors:
            assert sensor in data_access.component_mapping.values()

    @staticmethod
    def assert_component_batteries_initialized(data_access: DataAccess):
        expected_components = ["g3t1_1", "g3t1_2", "g3t1_3", "g3t1_4", "g3t1_5", "g3t1_6", "g4t1"]
        for component in expected_components:
            assert data_access.components_batteries[component] == 100.0

    @staticmethod
    def assert_formula_structure(data_access: DataAccess):
        reliability_formula = data_access.reliability_formula_text
        assert "CTX_G3_T1_1" in reliability_formula
        assert "CTX_G3_T1_2" in reliability_formula
        assert "CTX_G3_T1_3" in reliability_formula
        assert "CTX_G3_T1_4" in reliability_formula
        assert "CTX_G3_T1_5" in reliability_formula
        assert "CTX_G3_T1_6" in reliability_formula
        assert "CTX_G4_T1" in reliability_formula
        assert "R_G3_T1_1" in reliability_formula
        assert "F_G3_T1_1" in reliability_formula
        assert "R_G4_T1" in reliability_formula
        assert "F_G4_T1" in reliability_formula
        cost_formula = data_access.cost_formula_text
        assert "W_G3_T1_1" in cost_formula
        assert "W_G4_T1" in cost_formula

    @staticmethod
    def assert_process_query_reliability(response_content: str):
        assert "/g3t1_1:" in response_content
        assert "success" in response_content
        assert "fail" in response_content
        reli_value = float(response_content.split(";")[0].split(",")[-1])
        assert 0.65 < reli_value < 0.68  # Should be about 0.6667
        assert response_content.count(";") >= 2
        assert "," in response_content
        components = response_content.split(";")
        for component_data in components:
            if component_data:
                assert ":" in component_data
                parts = component_data.split(":")
                assert len(parts) == 2
                assert parts[0].startswith("/") and not parts[0].startswith("//")
                data_part = parts[1]
                if "," in data_part:
                    data_values = data_part.split(",")
                    try:
                        float(data_values[-1])
                    except ValueError:
                        assert False, f"Last value should be reliability: {data_values[-1]}"
                    for status in data_values[:-1]:
                        assert status in ["success", "fail"]

    @staticmethod
    def assert_process_query_context(response_content: str):
        assert "/g3t1_1:activate" in response_content
        assert "/g3t1_2:deactivate" in response_content
        assert "/g4t1:activate" in response_content

    @staticmethod
    def assert_process_query_formula(response_content: str, reliability_formula: str, cost_formula: str):
        assert response_content == reliability_formula or response_content == cost_formula

    @staticmethod
    def assert_receive_persist_message_status(data_access: DataAccess):
        assert "g3t1_1" in data_access.status
        assert len(data_access.status["g3t1_1"]) >= 1

    @staticmethod
    def assert_receive_persist_message_event(data_access: DataAccess):
        assert "g3t1_1" in data_access.events
        assert len(data_access.events["g3t1_1"]) >= 1
        assert data_access.events["g3t1_1"][0] in ["activate", "deactivate"]
        assert data_access.contexts["g3t1_1"] >= 1

    @staticmethod
    def assert_g4t1_special_case(data_access: DataAccess):
        assert data_access.contexts["g4t1"] in [0, 1]
        assert "g4t1" in data_access.status
        assert len(data_access.status["g4t1"]) >= 0

    @staticmethod
    def assert_complete_workflow(response_content: str, components: list):
        for component in components:
            assert f"/{component}" in response_content
        reli_value = float(response_content.split(";")[0].split(",")[-1])
        assert 0.65 < reli_value < 0.68

    @staticmethod
    def assert_data_access_service_integration(response_content: str):
        assert isinstance(response_content, str)
        assert len(response_content) > 0

    @staticmethod
    def assert_receive_persist_message_event_overwrite(data_access: DataAccess):
        assert "g3t1_1" in data_access.events
        assert len(data_access.events["g3t1_1"]) >= 1
        assert "deactivate" in data_access.events["g3t1_1"]

    @staticmethod
    def assert_receive_persist_message_status_event_interaction(data_access: DataAccess):
        assert "g3t1_1" in data_access.status
        assert len(data_access.status["g3t1_1"]) >= 1
        assert "g3t1_1" in data_access.events
        assert len(data_access.events["g3t1_1"]) >= 1
        assert data_access.contexts["g3t1_1"] >= 1

    @staticmethod
    def assert_receive_persist_message_g4t1_special_case(data_access: DataAccess):
        assert "g4t1" in data_access.status
        assert len(data_access.status["g4t1"]) >= 0
        assert "g4t1" in data_access.events
        assert len(data_access.events["g4t1"]) >= 0
        assert data_access.contexts["g4t1"] in [0, 1]

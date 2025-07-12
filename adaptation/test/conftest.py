"""
Pytest configuration for BSN2 Engine tests
"""

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor


@pytest.fixture(scope="session")
def rclpy_context():
    """Initialize rclpy once per test session"""
    if not rclpy.ok():
        rclpy.init()
    yield
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except:
        pass


@pytest.fixture(scope="session")
def executor(rclpy_context):
    """Create executor for the test session"""
    executor = SingleThreadedExecutor()
    yield executor
    try:
        executor.shutdown()
    except:
        pass


@pytest.fixture
def sample_formula():
    """Sample formula for testing"""
    return "R_G3_T1_1 * CTX_G3_T1_1 * F_G3_T1_1 + R_G3_T1_2 * CTX_G3_T1_2 * F_G3_T1_2"


@pytest.fixture
def sample_reliability_data():
    """Sample reliability data matching BSN1 format"""
    return "/g3t1_1:success,fail,success,0.67;/g3t1_2:success,success,0.85;"


@pytest.fixture
def sample_context_data():
    """Sample context data matching BSN1 format"""
    return "/g3t1_1:activate;/g3t1_2:deactivate;/g4t1:activate;"


@pytest.fixture
def sample_strategy():
    """Sample strategy matching BSN1 format"""
    return {
        "R_G3_T1_1": 0.85,
        "R_G3_T1_2": 0.90,
        "CTX_G3_T1_1": 1.0,
        "CTX_G3_T1_2": 1.0,
        "F_G3_T1_1": 1.0,
        "F_G3_T1_2": 1.0,
    }


@pytest.fixture
def sample_priority():
    """Sample priority matching BSN1 format"""
    return {
        "R_G3_T1_1": 50,
        "R_G3_T1_2": 70,
        "R_G4_T1": 30,
    }

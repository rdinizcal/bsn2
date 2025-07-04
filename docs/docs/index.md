## Overview

BSN 2 consists of five main modules:

### Adaptation Module
Provides mathematical expression evaluation and goal-oriented modeling capabilities:
- **Formula**: Parse and evaluate mathematical expressions with variables
- **Goal Model**: Hierarchical goal and task modeling framework

### Sensor Module  
Individual sensor nodes for health monitoring:
- **Sensor Node**: Lifecycle-managed sensor data collection
- **Risk Evaluation**: Real-time risk assessment for sensor readings
- **Battery Management**: Power-aware operation with automatic recharge

### Central Hub Module
Centralized data fusion and emergency detection:
- **Data Fusion**: Combine multiple sensor readings for patient status
- **Risk Analysis**: Emergency detection and alert generation
- **System Coordination**: Manage multiple sensors and system state

### Patient Module
Realistic patient simulation for testing and demonstration:
- **Markov Chain Simulation**: State-based vital sign generation
- **Multi-Vital Monitoring**: Temperature, heart rate, blood pressure, glucose, oxygen
- **Configurable Health States**: From normal to critical conditions
- **ROS Service Interface**: Provides data to sensor nodes

### System Monitor Module
Comprehensive system monitoring and message handling:
- **Node Monitoring**: Track component status and heartbeats
- **Message Collection**: Centralized gathering of system messages
- **Logging & Persistence**: Convert messages to persistent format
- **Parameter Adaptation**: Route configuration changes to components

## Quick Start

### System Monitoring
```python
from system_monitor.node_monitor import SystemMonitor

# Create and run system monitor
monitor = SystemMonitor()
rclpy.spin(monitor)
```
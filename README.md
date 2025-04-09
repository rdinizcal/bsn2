# BSN2 - Body Sensor Network (ROS 2 Jazzy)

BSN2 is a ROS 2 Jazzy-based application for monitoring and processing sensor data from a body sensor network. It consists of four ROS 2 packages:

- `patient`
- `sensor`
- `central_hub`
- `bsn_interfaces`

This guide explains how to install, build, and run BSN2 using Docker for a streamlined and portable development environment.

## Prerequisites

Before proceeding, ensure you have the following installed on your system:

- [Docker](https://docs.docker.com/get-docker/)

## Installation & Setup

### 1. Clone the Repository

```sh
git clone git@github.com:rdinizcal/bsn2.git
cd bsn2
```

### 2. Build the Docker Image

```sh
docker build -t bsn2:latest .
```

### 3. Run the Container

To start an interactive session within the container:

```sh
docker run -it --rm bsn2:latest
```

This will start a container where ROS 2 is sourced, and you can run ROS 2 commands.

### 4. Running ROS 2 Nodes

Inside the running container, source the environment and run ROS 2 nodes:

```sh
source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
```

Run nodes (in different terminals)

```
ros2 run patient patient
ros2 run sensor thermometer
```

## Development Workflow

### Modifying Code

Changes to source files can be done locally and reflected inside the container if you mount the workspace as a volume:

```sh
docker run -it --rm -v $(pwd):/ros_ws bsn2:latest
```

If MacOS:

```sh
docker run -it --platform linux/amd64 bsn2:latest
```

### Rebuilding the Workspace

After modifying the code, rebuild using:

```sh
colcon build
source /ros_ws/install/setup.bash
```

### Stopping the Container

To exit and remove the container, use:

```sh
exit  # or Ctrl+D
```

### Using devcontainer in vscode

1. download docker extension
2. click on reopen in container
3. if setup.sh was not executed and make sure it is in LF, run in ros_ws directory:

```sh
bash src/.devcontainer/setup.sh
```

4. run the launch file:

```sh
source /ros_ws/install/setup.bash
ros2 launch central_hub emergency_detection_launch.py
```

## Contributing

To collaborate on BSN2 development:

1. Fork the repository.
2. Create a feature branch.
3. Commit changes and push to your branch.
4. Open a pull request.

## License

BSN2 is licensed under [TBD - specify license].

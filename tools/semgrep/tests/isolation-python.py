"""Static fixtures for Semgrep, never imported or executed."""
import rclpy
import time
import io
import os
import threading
import subprocess
from rclpy.node import Node


# ruleid: isolation.python.ros-node-base
class ROSAdapter(Node):
    pass


# ok: isolation.python.ros-node-base
class PureObject:
    pass


def resources(node, client, path, ordinary):
    # ruleid: isolation.python.ros-parameters
    node.get_parameter('frequency')
    # ok: isolation.python.ros-parameters
    node.get_buffer()
    # ruleid: isolation.python.ros-resource
    node.create_client(Service, 'data')
    # ok: isolation.python.ros-resource
    node.create_buffer()
    # ruleid: isolation.python.ros-timer
    node.create_timer(1.0, callback)
    # ruleid: isolation.python.ros-runtime
    rclpy.init()
    # ruleid: isolation.python.service-wait
    client.wait_for_service(timeout_sec=1.0)
    # ruleid: isolation.python.client-call
    client.call(request)
    # ok: isolation.python.client-call
    ordinary.call(request)
    # ruleid: isolation.python.file-io
    open('data.txt')
    # ruleid: isolation.python.file-io
    path.read_text()
    # ruleid: isolation.python.directory-io
    path.mkdir()
    # ruleid: isolation.python.real-clock
    time.time()
    # ruleid: isolation.python.real-sleep
    time.sleep(1)
    # ruleid: isolation.python.thread-creation
    threading.Thread(target=callback)
    # ruleid: isolation.python.process-execution
    subprocess.run(['tool'])
    # ruleid: isolation.python.environment-read
    os.getenv('CONFIG')


class DirectConstructor:
    def __init__(self, node):
        # ruleid: isolation.python.supplied-collaborator
        self.node = node
        # ruleid: isolation.python.constructor-effect, isolation.python.ros-resource
        node.create_client(Service, 'data')


class ConstructorHelper:
    def __init__(self):
        self.setup()

    def setup(self):
        # ruleid: isolation.python.constructor-helper-wait, isolation.python.service-wait
        self.client.wait_for_service(timeout_sec=1.0)


class DeferredSetup:
    def __init__(self, value):
        # ok: isolation.python.supplied-collaborator
        self.value = value

    def setup(self):
        # ok: isolation.python.constructor-helper-wait, isolation.python.constructor-effect
        # ruleid: isolation.python.service-wait
        self.client.wait_for_service(timeout_sec=1.0)


def negative_examples(ordinary):
    # ok: isolation.python.ros-timer, isolation.python.ros-runtime, isolation.python.service-wait
    ordinary.compute()
    # ok: isolation.python.file-io, isolation.python.directory-io
    data = 'open() mkdir()'
    # ok: isolation.python.real-clock, isolation.python.real-sleep
    ordinary.calculate_time()
    # ok: isolation.python.thread-creation, isolation.python.process-execution, isolation.python.environment-read
    ordinary.transform(data)

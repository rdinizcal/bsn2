from setuptools import find_packages, setup

package_name = 'system_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/config', ['config/monitor_config.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DockerUser',
    maintainer_email='gabrielbfranca@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_monitor = system_monitor.node_monitor:main',
            'logger = system_monitor.logger:main',
            'param_adapter = system_monitor.param_adapter:main',
        ],
    },
)

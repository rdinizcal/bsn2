from setuptools import find_packages, setup

package_name = 'sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ricardo.caldas@gssi.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor = sensor.sensor:main',
            'thermometer = sensor.thermometer:main',
            'oximeter = sensor.oximeter:main',
            'heart_rate = sensor.ecg:main',
            'abps = sensor.abps:main',
            'abpd = sensor.abpd:main',
            'glucose = sensor.glucose:main',
        ],
    },
)

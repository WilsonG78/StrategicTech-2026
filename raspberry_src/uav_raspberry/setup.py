from setuptools import find_packages, setup

package_name = 'uav_raspberry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: []},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/uav.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/tests', []),
    ],
    install_requires=['setuptools', 'mavsdk'],
    zip_safe=True,
    maintainer='StrategicTech',
    maintainer_email='strategic.tech.pl@gmail.com',
    description='UAV Raspberry Pi node – telemetry bridge between ArduPilot (MAVSDK) and ROS 2',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'telemetry_node = uav_raspberry.telemetry_node:main',
            'logistic_map_node = uav_raspberry.logistic_map:main',
            'scout_mission_node = uav_raspberry.scout_mission:main'],
    },
)

from setuptools import find_packages, setup

package_name = 'dualtech'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hubert Szolc',
    maintainer_email='szolc@agh.edu.pl',
    description='Package for the DUAL-TECH AGH competition',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detection_publisher = dualtech.detection_publisher:main',
            'detection_subscriber = dualtech.detection_subscriber:main',
        ],
    },
)

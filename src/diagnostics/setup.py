from setuptools import find_packages, setup

package_name = 'diagnostics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdul Shahzeb',
    maintainer_email='z5311131@ad.unsw.edu.au',
    description='ROS latency measurements for the robot and extruder.',
    license='OpenSSL',
    entry_points={
        'console_scripts': [
            'arduino_latency_tester = diagnostics.arduino_latency_tester:main',
            'robot_latency_tester = diagnostics.robot_latency_tester:main',
        ],
    },
)

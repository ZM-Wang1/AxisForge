from setuptools import find_packages, setup

package_name = 'extrusion_control'

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
    description='Filament extrusion control, synchronization and calibration.',
    license='OpenSSL',
    entry_points={
        'console_scripts': [
            'calibrate_extruder = extrusion_control.calibrate_extruder:main',
        ],
    },
)

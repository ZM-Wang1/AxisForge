from glob import glob
from setuptools import find_packages, setup

package_name = 'print_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/gcode', glob('gcode/*.gcode')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdul Shahzeb',
    maintainer_email='z5311131@ad.unsw.edu.au',
    description='G-code interpretation, print paths, execution and keyboard control.',
    license='OpenSSL',
    entry_points={
        'console_scripts': [
            'gcode_interpreter = print_control.gcode_interpreter:main',
            'keyboard_node = print_control.keyboard:main',
            'bed_probe = print_control.bed_probe:main',
        ],
    },
)

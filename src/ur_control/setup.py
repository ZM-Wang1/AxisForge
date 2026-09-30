import os

from setuptools import find_packages, setup

package_name = 'ur_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + [
        (os.path.join('share', package_name, directory),
         [os.path.join(directory, name) for name in files if not name.endswith('.pyc')])
        for directory, subdirs, files in os.walk('models')
        if '__pycache__' not in directory and files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdul Shahzeb',
    maintainer_email='z5311131@ad.unsw.edu.au',
    description='UR robot motion control and custom robot models.',
    license='OpenSSL',
    entry_points={
        'console_scripts': [
            'move_ur = ur_control.move_ur:main',
        ],
    },
)

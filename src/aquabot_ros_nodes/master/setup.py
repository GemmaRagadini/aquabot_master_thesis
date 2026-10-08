from setuptools import setup, find_packages
from glob import glob
import os 

package_name = 'master'

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
    maintainer='eldiez',
    maintainer_email='eldiez@todo.todo',
    description='Master node for Aquabot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_node = master.master_node:main',
            'fake_sensor_node = master.fake_sensor_node:main',
        ],
    },
)
import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'dynamixel_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eldiez',
    maintainer_email='eldiez@todo.todo',
    description='Dynamixel controller node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel_controller_node = dynamixel_controller.dynamixel_controller_node:main',
        ],
    },
)

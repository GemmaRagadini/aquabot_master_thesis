import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aquabot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Indice pacchetti
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # Tutti i launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Tutti i config yaml
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eldiez',
    maintainer_email='eldiez@todo.todo',
    description='Aquabot bringup package (launch + config).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []} # vuoto
)

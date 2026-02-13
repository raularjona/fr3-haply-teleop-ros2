from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fr3_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 1. Instala los archivos de configuración YAML
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 2. Instala los archivos de lanzamiento Launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # ------------------------------
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raul Arjona',
    maintainer_email='raularjonaarmisen@gmail.com',
    description='Control and MoveIt Servo configuration for Franka FR3 teleoperation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

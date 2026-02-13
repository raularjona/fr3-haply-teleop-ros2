from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fr3_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- ESTO ES LO IMPORTANTE: Instala los archivos de Launch y RViz ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raul Arjona',
    maintainer_email='raularjonaarmisen@gmail.com',
    description='Launch and visualization files for Franka FR3 teleoperation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Aquí no necesitamos scripts por ahora, solo archivos de lanzamiento
        ],
    },
)

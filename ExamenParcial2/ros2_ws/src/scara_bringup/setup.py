from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'scara_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sr-gus',
    maintainer_email='lgmarin02@hotmail.com',
    description='Bringup del robot SCARA: control y trayectoria',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scara_tray_line = scara_bringup.scara_tray_line_py:main'
        ],
    },
)

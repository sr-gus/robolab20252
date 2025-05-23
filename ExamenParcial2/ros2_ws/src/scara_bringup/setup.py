from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'scara_bringup'

setup(
    name=package_name,
    version='0.0.0',
    # ahora sí encuentra scara_bringup/
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sr-gus',
    maintainer_email='lgmarin02@hotmail.com',
    description='Bringup del SCARA: URDF + RViz',
    license='MIT',
    entry_points={
        'console_scripts': [
            'initial_joint_state = scara_bringup.initial_joint_state_publisher_py:main',
        ],
    }
)

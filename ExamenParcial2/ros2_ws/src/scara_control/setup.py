from setuptools import setup, find_packages
from glob import glob  

package_name = 'scara_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sr-gus',
    maintainer_email='lgmarin02@hotmail.com',
    description='Algoritmos de control para SCARA',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scara_tray_line = scara_control.scara_tray_line_py:main'
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
)

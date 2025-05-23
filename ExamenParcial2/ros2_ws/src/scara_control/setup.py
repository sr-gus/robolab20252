from setuptools import setup, find_packages

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
    }
)

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'serial_bridge_2026'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
   
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adhikari',
    maintainer_email='080bct059.pratyush@pcampus.edu.np',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            # 'serial_nucleo = scripts.serial_nucleo:main',
            # 'serial_bluepill = scripts.serial_bluepill:main',
            'publisher_node = scripts.publisher:main',
            'serial_bridge_node = scripts.serial_bridge_node:main',
            'subscriber_node = scripts.subscriber:main',
            'laptop_node = scripts.laptop_end:main',
            'gpio_node = scripts.gpio_button:main',
            'teacher_node = scripts.teach_setpoint:main',
            'cmd_multiplexer = scripts.cmd_multiplex:main',
            'process_odom = scripts.process_odom:main'
        ],
    },
)

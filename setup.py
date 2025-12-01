from setuptools import setup
import os
from glob import glob

package_name = 'mes1_system'

setup(
    name=package_name,
    version='2.0.0',
    packages=['ros_can_bridge_native', 'ros_can_bridge_native.utils'],  # Keep module names for compatibility
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/mes1_system']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sadaa Khan',
    maintainer_email='sadaa@example.com',
    description='Production-grade ROS 2 CAN Bridge for gs_usb with native SocketCAN support',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'socketcan_bridge_node = ros_can_bridge_native.socketcan_bridge_node:main',
            'command_client = ros_can_bridge_native.command_client:main',
            'diagnostics_publisher = ros_can_bridge_native.diagnostics_publisher:main',
            'multi_node_bridge = ros_can_bridge_native.multi_node_bridge:main',
            'multi_node_command_client = ros_can_bridge_native.multi_node_command_client:main',
        ],
    },
)

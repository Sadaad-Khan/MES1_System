import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'winch_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wongyichen',
    maintainer_email='wongyichen00@gmail.com',
    description='KEYA Driver DC Winch Motor Hardware and ROS2 Interface',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'winch_interface = winch_control.winch_interface:main',
            'winch_action_client = winch_control.winch_action_client:main',
        ],
    },
)

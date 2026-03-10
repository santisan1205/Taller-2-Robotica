from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'diff_bot_3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='robotica@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    	'console_scripts': [
            'robot_core = diff_bot_3.robot_core:main'
        	'robot_teleop = diff_bot_3.robot_teleop:main',
        	'robot_interface = diff_bot_3.robot_interface:main',
        	'robot_player = diff_bot_3.robot_player:main',
    	],
    },
)

import os
from glob import glob

from setuptools import setup

package_name = 'mrs_roboracer_sysid'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*_launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'pandas'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='andykamin3@gmail.com',
    description='Open-loop motion-noise data collection for the kinematic bicycle model'
                ' (sysid_data_collector)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sysid_data_collector = mrs_roboracer_sysid.sysid_data_collector:main',
            'odom_to_pose_adapter_node = mrs_roboracer_sysid.odom_to_pose_adapter:main',
            'sysid_bag_to_dataset = mrs_roboracer_sysid.bag_to_dataset:main',
        ],
    },
)

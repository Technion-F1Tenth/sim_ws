import os
from glob import glob

from setuptools import setup

package_name = 'mrs_mocap_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'web'),
            glob(os.path.join('web', '*'))),
    ],
    install_requires=['setuptools', 'flask', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='andykamin3@gmail.com',
    description='Build an OccupancyGrid from OptiTrack markers via a web editor'
                ' (mocap_map_node)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocap_map_node = mrs_mocap_mapping.mocap_map_node:main',
        ],
    },
)

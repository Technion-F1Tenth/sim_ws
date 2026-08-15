import os

from setuptools import setup

package_name = 'mrs_roboracer_inference'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'model'),
            ['model/policy.pt', 'model/MODEL_INFO.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='andykamin3@gmail.com',
    description='Learned inference node for RoboRacer',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inference_node = mrs_roboracer_inference.inference_node:main',
        ],
    },
)

from setuptools import setup
from glob import glob
import os

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreskaminker',
    maintainer_email='andreskaminker@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = pure_pursuit.pure_pursuit:main'
        ],
    },
)

from setuptools import setup

package_name = 'mrs_roboracer_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='andykamin3@gmail.com',
    description='Safety arbitration node: clipping and TTC-based AEB before /drive',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = mrs_roboracer_safety.safety_node:main',
        ],
    },
)

from setuptools import setup

package_name = 'mrs_roboracer_plan_playback'

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
    description='Replays plan CSVs as a tracked Path or an open-loop command sequence',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plan_path_publisher = mrs_roboracer_plan_playback.plan_path_publisher_node:main',
            'plan_open_loop_player = mrs_roboracer_plan_playback.plan_open_loop_player_node:main',
        ],
    },
)

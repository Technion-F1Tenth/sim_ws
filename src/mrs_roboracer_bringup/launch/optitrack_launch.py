"""
OptiTrack driver launch: mocap4r2_optitrack_driver, configured and activated.

Included by `mocap_map_launch.py` (the pit-PC side of every `mode:=mocap`
launch), and runnable on its own.
The driver publishes `markers` (mocap4r2_msgs/Markers) and `rigid_bodies`
(mocap4r2_msgs/RigidBodies).

The driver is a lifecycle node. With `auto_activate:=true` (default) it is
configured on launch and activated as soon as it reaches `inactive`, so no
manual `ros2 lifecycle set ... activate` is needed.
"""

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():
    ld = LaunchDescription()

    driver_share = get_package_share_directory('mocap4r2_optitrack_driver')
    default_driver_config = os.path.join(
        driver_share, 'config', 'mocap4r2_optitrack_driver_params.yaml')

    driver_config_arg = DeclareLaunchArgument(
        'driver_config', default_value=default_driver_config,
        description='mocap4r2_optitrack_driver params (server/local address, multicast).')
    auto_activate_arg = DeclareLaunchArgument(
        'auto_activate', default_value='true',
        description='Configure and activate the driver lifecycle node automatically.')

    driver_node = LifecycleNode(
        package='mocap4r2_optitrack_driver',
        executable='mocap4r2_optitrack_driver_main',
        name='mocap4r2_optitrack_driver_node',
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('driver_config')],
    )

    configure_driver = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    activate_driver_when_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=driver_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=launch.events.matchers.matches_action(driver_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                ),
            )],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    for action in (driver_config_arg, auto_activate_arg):
        ld.add_action(action)

    ld.add_action(activate_driver_when_configured)
    ld.add_action(driver_node)
    ld.add_action(configure_driver)

    return ld

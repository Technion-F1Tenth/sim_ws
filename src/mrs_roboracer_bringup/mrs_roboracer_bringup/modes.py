"""
Mode -> topic contract shared by the driving launch files.

Every driving launch takes `mode:=sim|onboard|mocap` and includes
`platform_launch.py`, which brings up whatever produces these topics in that
mode. Controllers resolve their topics here instead of hardcoding sim names.

- scan:  sensor_msgs/LaserScan from the car (or the sim).
- odom:  nav_msgs/Odometry with local velocity. The safety node's TTC AEB
         uses it, so it is always produced on the car itself and never
         crosses WiFi.
- pose:  nav_msgs/Odometry of the car in the `map` frame, for controllers
         that need a global pose (pure pursuit, MPPI, sysid ground truth).
         sim: gym ground truth; onboard: particle_filter (needs
         `localization:=true` and a `map`); mocap: mrs_mocap_mapping's
         /mocap_odom, published by the pit PC.
- drive: the topic the safety node writes and the platform consumes.
"""

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

MODES = ('sim', 'onboard', 'mocap')

TOPICS = {
    'sim': {
        'scan': '/scan',
        'odom': '/ego_racecar/odom',
        'pose': '/ego_racecar/odom',
        'drive': '/drive',
    },
    'onboard': {
        'scan': '/scan',
        'odom': '/odom',
        'pose': '/pf/pose/odom',
        'drive': '/drive',
    },
    'mocap': {
        'scan': '/scan',
        'odom': '/odom',
        'pose': '/mocap_odom',
        'drive': '/drive',
    },
}

MODE_DESCRIPTION = (
    "Platform: 'sim' (f1tenth_gym_ros bridge), 'onboard' (car hardware via "
    "f1tenth_stack, particle_filter when the controller needs a pose), or "
    "'mocap' (car hardware, pose from /mocap_odom; run mocap_map_launch.py "
    'on the pit PC). Required.')


def check_mode(mode, allowed=MODES):
    """Return `mode` if it is one of `allowed`, else raise with the valid choices."""
    if mode not in allowed:
        raise RuntimeError(
            f"mode:='{mode}' is not supported here; choose one of: {', '.join(allowed)}")
    return mode


def topics(mode):
    """Topic dict for `mode` (a copy, safe to modify)."""
    return dict(TOPICS[check_mode(mode)])


def topic_arguments(*keys):
    """`<key>_topic` launch arguments; left empty, the mode's default is used."""
    return [DeclareLaunchArgument(
        f'{key}_topic', default_value='',
        description=f"Override the mode's {key} topic (empty: per-mode default).")
        for key in keys]


def resolve_topics(context, allowed=MODES):
    """Topic dict for the `mode` launch argument, with `<key>_topic` overrides applied."""
    mode = check_mode(LaunchConfiguration('mode').perform(context), allowed)
    resolved = topics(mode)
    for key in resolved:
        override = context.launch_configurations.get(f'{key}_topic', '')
        if override:
            resolved[key] = override
    return mode, resolved


def include_platform(localization='false'):
    """Include platform_launch.py for the current `mode` launch argument.

    `localization` is 'true' for controllers that need the `pose` topic; only
    the onboard mode acts on it (sim and mocap always have a pose source).
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('mrs_roboracer_bringup'), 'launch', 'platform_launch.py'])),
        launch_arguments={
            'mode': LaunchConfiguration('mode'),
            'localization': localization,
        }.items(),
    )

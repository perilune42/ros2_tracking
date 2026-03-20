import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch_ros.actions import Node


def _load_yaml(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('tracker_v2')
    node_config_path = os.path.join(pkg_share, 'config', 'node_config.yaml')
    tracker_params = os.path.join(pkg_share, 'config', 'tracker_params.yaml')
    vesc_params = os.path.join(pkg_share, 'config', 'vesc_params.yaml')
    nav_params = os.path.join(pkg_share, 'config', 'nav_params.yaml')

    cfg = _load_yaml(node_config_path)
    nav_cfg = _load_yaml(nav_params)
    gps_bridge_params = nav_cfg.get('gps_runner_bridge_node', {}).get('ros__parameters', {})
    gps_runner_command = str(
        os.environ.get(
            'TRACKER_V2_GPS_RUNNER_COMMAND',
            gps_bridge_params.get('runner_command', ''),
        )
    ).strip()
    vesc_enabled = cfg.get('vesc_enabled', True)
    gps_enabled = cfg.get('gps_enabled', True)

    nodes = []
    if True:
        nodes.append(
            Node(
                package='tracker_v2',
                executable='gps_publisher',
                name='gps_publisher',
                parameters=[],
                output='screen',
            )
        )
    else:
        nodes.append(LogInfo(msg='[node_config] gps_enabled=false - skipping GPS waypoint node'))

    if vesc_enabled:
        nodes.append(
            Node(
                package='tracker_v2',
                executable='vesc_twist_node',
                name='vesc_twist_node',
                parameters=[vesc_params],
                output='screen',
            )
        )
    else:
        nodes.append(LogInfo(msg='[node_config] vesc_enabled=false - skipping VESC command node'))

    nodes.append(
        Node(
            package='tracker_v2',
            executable='gps_path_follow_node',
            name='gps_path_follow_node',
            parameters=[],
            output='screen',
        )
    )

    nodes.append(
        Node(
            package='tracker_v2',
            executable='yolo_oakd_tracking',
            name='yolo_oakd_tracking',
            parameters=[],
            output='screen',
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])

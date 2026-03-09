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

    cfg = _load_yaml(node_config_path)
    camera_enabled = cfg.get('camera_enabled', True)
    pid_enabled = cfg.get('pid_enabled', True)
    vesc_enabled = cfg.get('vesc_enabled', True)

    nodes = []
    if camera_enabled:
        nodes.append(
            Node(
                package='tracker_v2',
                executable='tracker_camera_node',
                name='tracker_camera_node',
                parameters=[tracker_params],
                output='screen',
                additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')},
            )
        )
    else:
        nodes.append(LogInfo(msg='[node_config] camera_enabled=false - skipping tracker camera node'))

    if pid_enabled:
        nodes.append(
            Node(
                package='tracker_v2',
                executable='tracker_pid_node',
                name='tracker_pid_node',
                parameters=[tracker_params],
                output='screen',
            )
        )
    else:
        nodes.append(LogInfo(msg='[node_config] pid_enabled=false - skipping PID node'))

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

    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])

from setuptools import setup
import os
from glob import glob

package_name = 'tracker_v2'
submodule = package_name + '/vesc_submodule'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, submodule],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'PyYAML',
        'depthai',
        'depthai-nodes',
        'lap',
        'pyserial',
        'pyvesc',
        'ultralytics',
    ],
    zip_safe=True,
    maintainer='team7',
    maintainer_email='team7@ucsd.edu',
    description='DepthAI person tracking, PID following, and VESC integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_camera_node = tracker_v2.tracker_camera_node:main',
            'tracker_pid_node    = tracker_v2.tracker_pid_node:main',
            'vesc_twist_node     = tracker_v2.vesc_twist_node:main',
        ],
    },
)

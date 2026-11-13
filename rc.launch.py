"""ros2 launch /project/controller/rc.launch.py (no new ROS package/build)."""
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    root = Path(__file__).resolve().parent
    config_path = Path(os.environ.get('RC_CONFIG', root / 'config.json'))
    c = json.loads(config_path.read_text())
    from core import validate_config
    validate_config(c)
    lidar = Path(get_package_share_directory('sllidar_ros2')) / 'launch/sllidar_c1_launch.py'
    actions = [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(lidar)), launch_arguments={
        'serial_port': c['ros']['serial_port'], 'serial_baudrate': '460800', 'frame_id': 'laser'}.items()),
        ExecuteProcess(cmd=['python3', str(root / 'ros_bridge.py'), '--config', str(config_path)], output='screen')]
    if c['odom']['enabled'] and c['ros']['laser_xyz_yaw'] is not None:
        slam_config = Path(os.environ.get('RC_SLAM_CONFIG', root.parent / 'config/slam.yaml'))
        if not slam_config.exists():
            raise RuntimeError(f'Existing SLAM config missing: {slam_config}')
        filename = 'localization_launch.py' if os.environ.get('RC_SLAM_LOCALIZATION') == '1' else 'online_async_launch.py'
        launch = Path(get_package_share_directory('slam_toolbox')) / 'launch' / filename
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)), launch_arguments={
            'slam_params_file': str(slam_config), 'use_sim_time': 'false'}.items()))
    else:
        actions.append(LogInfo(msg='SLAM NOT STARTED: calibrated odometry and measured base_link->laser transform required.'))
    return LaunchDescription(actions)

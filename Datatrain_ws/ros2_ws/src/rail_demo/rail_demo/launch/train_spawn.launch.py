from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rail_demo')
   
    model = os.path.join(pkg_share, 'models','base_models', 'gazebo_train', 'model.sdf')

    models_path = os.path.join(pkg_share, 'models')

    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_gz_path:
        gz_path = models_path + os.pathsep + existing_gz_path
    else:
        gz_path = models_path

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_path),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gz_path),

        LogInfo(msg=f'Using model: {model}'),
        LogInfo(msg=f'GZ_SIM_RESOURCE_PATH: {gz_path}'),

        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-world', 'train_world',
                '-name', 'train',
                '-file', model,
                '-x', '0.0', '-y', '-0.018', '-z', '0.1',
                '-R', '0.0', '-P', '0.0', '-Y', '1.571',
            ],
        ),
    ])
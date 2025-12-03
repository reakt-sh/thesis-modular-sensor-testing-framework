from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, LogInfo, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rail_demo')
    world = os.path.join(pkg_share, 'worlds', 'train_world.sdf')

    # Use the real train model instead of 'model.sdf' (e.g. gazebo_train)
    model = os.path.join(pkg_share, 'models', 'gazebo_train', 'model.sdf')

    # Path where all your models live
    models_path = os.path.join(pkg_share, 'models')

    # Preserve any existing resource paths (if set)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    if existing_gz_path:
        gz_path = models_path + os.pathsep + existing_gz_path
    else:
        gz_path = models_path

    return LaunchDescription([
        # Route GUI to the noVNC X server
        SetEnvironmentVariable('DISPLAY', ':1'),

        # Let gz sim find your models
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_path),
        # (optional but harmless, helps some setups)
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gz_path),

        LogInfo(msg=f'Using world: {world}'),
        LogInfo(msg=f'Using model: {model}'),
        LogInfo(msg=f'GZ_SIM_RESOURCE_PATH: {gz_path}'),

        ExecuteProcess(cmd=['gz', 'sim', '-v', '4', world], output='screen'),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/train/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/train/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                #'/world/train_world/model/train/link/base/sensor/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
            ],
            output='screen',
        ),

       TimerAction(
            period=4.0,
            actions=[
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
            ],
        ),
    ])
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, LogInfo, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rail_demo')
    world = os.path.join(pkg_share, 'worlds', 'train_world.sdf')
    model = os.path.join(pkg_share, 'models', 'model.sdf')

    return LaunchDescription([
        # Route GUI to the noVNC X server
        SetEnvironmentVariable('DISPLAY', ':1'),

        # Print the exact resolved paths (debug aid)
        LogInfo(msg=f'Using world: {world}'),
        LogInfo(msg=f'Using model: {model}'),

        # 1) Start Gazebo with GUI
        ExecuteProcess(cmd=['gz', 'sim', '-v', '4', world], output='screen'),

        # 2) Start the bridge using inline mappings (robust)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/train/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/train/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/world/train_world/model/train/link/base/sensor/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
            output='screen',
        ),

        # 3) Spawn the train after a short delay to let the world/server initialize
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    output='screen',
                    arguments=[
                        '-world', 'train_world',
                        '-name', 'train',
                        '-file', model,
                        '-x', '-9.5', '-y', '0', '-z', '0.05', '-Y', '0.0'
                    ],
                ),
            ],
        ),
    ])
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess
import time
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def entity_exists(entity_name):
    cmd = "gz model --list"

    result = subprocess.run(
        cmd, shell=True, capture_output=True, text=True
    )

    return f"- {entity_name}" in result.stdout

    return entity_name in result.stdout
def delete_entity(entity_name, world_name):
    if not entity_exists(entity_name):
        print(f"[train_spawn] Entity '{entity_name}' does not exist, skipping delete")
        return

    print(f"[train_spawn] Deleting entity '{entity_name}'")

    cmd = (
        f"ros2 service call /world/{world_name}/remove "
        f"ros_gz_interfaces/srv/DeleteEntity "
        f"\"{{entity: {{name: '{entity_name}', type: 2}}}}\""
    )

    subprocess.run(cmd, shell=True)
    time.sleep(0.1)

def delete_train_action(context):
    world_name = LaunchConfiguration('world_name').perform(context)
    delete_entity("train", world_name)
    return []

def generate_launch_description():
    pkg_share = get_package_share_directory('rail_demo')
   
    model = os.path.join(pkg_share, 'models','base_models', 'reakt_train', 'model.sdf')

    models_path = os.path.join(pkg_share, 'models')

    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_gz_path:
        gz_path = models_path + os.pathsep + existing_gz_path
    else:
        gz_path = models_path
    
    world_name = LaunchConfiguration('world_name')

    return LaunchDescription([
    DeclareLaunchArgument(
        'world_name',
        default_value='train_world',
        description='Gazebo world name'
    ),

    SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_path),
    SetEnvironmentVariable('GAZEBO_MODEL_PATH', gz_path),

    LogInfo(msg=f'Using model: {model}'),
    LogInfo(msg=f'GZ_SIM_RESOURCE_PATH: {gz_path}'),

    OpaqueFunction(function=delete_train_action),

    Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', world_name,
            '-name', 'train',
            '-file', model,
            '-x', '10.0', '-y', '-0.018', '-z', '0.1',
            '-R', '0.0', '-P', '0.0', '-Y', '1.571',
        ],
    ),
])
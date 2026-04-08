from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, LogInfo, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory

from ros_gz_sim.actions import GzServer
from ros_gz_bridge.actions import RosGzBridge

import os
import yaml


def load_simulation_config():
    pkg_share = get_package_share_directory("rail_demo")
    config_path = os.path.join(pkg_share, "config", "simulation_config.yaml")

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"simulation_config.yaml not found: {config_path}")

    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    simulation = data.get("simulation", {})
    world_name = simulation.get("world")

    if not world_name:
        raise ValueError("Missing 'simulation.world' in simulation_config.yaml")

    if not world_name.endswith(".sdf"):
        world_name = f"{world_name}.sdf"

    return world_name


def generate_launch_description():
    pkg_share = get_package_share_directory("rail_demo")
    bridge_yaml = os.path.join(pkg_share, "config", "bridge.yaml")

    # Read world from config file
    world_name = load_simulation_config()
    world_path = os.path.join(pkg_share, "worlds", world_name)

    if not os.path.exists(world_path):
        raise FileNotFoundError(f"Configured world file not found: {world_path}")

    # --------------------------
    # MODEL PATHS
    # --------------------------
    models_path = os.path.join(pkg_share, "models")
    base_models_path = os.path.join(models_path, "base_models")

    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    merged = models_path + os.pathsep + base_models_path
    if existing:
        merged = merged + os.pathsep + existing

    # --------------------------
    # Launch arguments
    # --------------------------
    declare_world_arg = DeclareLaunchArgument(
        "world_sdf_file",
        default_value=TextSubstitution(text=world_path),
        description="Path to the SDF world file"
    )

    declare_bridge_name = DeclareLaunchArgument(
        "bridge_name",
        default_value="rail_bridge",
        description="Name of the bridge node"
    )

    declare_container_name = DeclareLaunchArgument(
        "container_name",
        default_value="ros_gz_container",
        description="ROS component container name"
    )

    declare_use_composition = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composition"
    )

    # --------------------------
    # Start Gazebo server
    # --------------------------
    gz_server = GzServer(
        world_sdf_file=LaunchConfiguration("world_sdf_file"),
        container_name=LaunchConfiguration("container_name"),
        create_own_container=True,
        use_composition=LaunchConfiguration("use_composition")
    )

    # --------------------------
    # Start Gazebo GUI
    # --------------------------
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-g"],
        output="screen"
    )

    # --------------------------
    # Start ROS <-> Gazebo bridge
    # --------------------------
    ros_gz_bridge = RosGzBridge(
        bridge_name=LaunchConfiguration("bridge_name"),
        config_file=TextSubstitution(text=bridge_yaml),
        container_name=LaunchConfiguration("container_name"),
        create_own_container=False,
        use_composition=LaunchConfiguration("use_composition"),
        use_respawn=False,
        log_level="info"
    )

    # Use configured world name also for service bridges
    world_service_prefix = f"/world/{os.path.splitext(world_name)[0]}"

    spawn_entity_bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            f"{world_service_prefix}/create@ros_gz_interfaces/srv/SpawnEntity"
        ],
        output="screen"
    )

    delete_entity_bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            f"{world_service_prefix}/remove@ros_gz_interfaces/srv/DeleteEntity"
        ],
        output="screen"
    )

    set_pose_bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            f"{world_service_prefix}/set_pose@ros_gz_interfaces/srv/SetEntityPose"
        ],
        output="screen"
    )

    return LaunchDescription([
        SetEnvironmentVariable("DISPLAY", ":1"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", merged),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", merged),

        LogInfo(msg=f"Configured world: {world_name}"),
        LogInfo(msg=f"World file: {world_path}"),
        LogInfo(msg=f"GZ_SIM_RESOURCE_PATH: {merged}"),

        declare_world_arg,
        declare_bridge_name,
        declare_container_name,
        declare_use_composition,

        gz_server,
        gz_gui,
        ros_gz_bridge,
        spawn_entity_bridge,
        delete_entity_bridge,
        set_pose_bridge,
    ])
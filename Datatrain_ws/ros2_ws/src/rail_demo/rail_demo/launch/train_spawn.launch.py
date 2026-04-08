from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, LogInfo, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import subprocess
import time
import yaml


def entity_exists(entity_name):
    cmd = "gz model --list"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    return f"- {entity_name}" in result.stdout


def delete_entity(entity_name, world_name):
    if not entity_exists(entity_name):
        print(f"[train_spawn] Entity '{entity_name}' does not exist, skipping delete")
        return

    print(f"[train_spawn] Deleting entity '{entity_name}' from world '{world_name}'")

    cmd = (
        f"ros2 service call /world/{world_name}/remove "
        f"ros_gz_interfaces/srv/DeleteEntity "
        f"\"{{entity: {{name: '{entity_name}', type: 2}}}}\""
    )

    subprocess.run(cmd, shell=True)
    time.sleep(0.1)


def load_simulation_config(pkg_share):
    config_path = os.path.join(pkg_share, "config", "simulation_config.yaml")

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"simulation_config.yaml not found: {config_path}")

    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    simulation = data.get("simulation", {})
    train = data.get("train", {})
    spawn = train.get("spawn", {})

    model_name = simulation.get("train_model")
    entity_name = simulation.get("train_entity_name", "train")
    world_name = simulation.get("world", "train_world")

    if not model_name:
        raise ValueError("Missing 'simulation.train_model' in simulation_config.yaml")

    if world_name.endswith(".sdf"):
        world_name = os.path.splitext(world_name)[0]

    model_path = os.path.join(
        pkg_share, "models", "base_models", model_name, "model.sdf"
    )

    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Configured train model not found: {model_path}")

    return {
        "model_name": model_name,
        "entity_name": entity_name,
        "world_name": world_name,
        "model_path": model_path,
        "spawn": {
            "x": str(spawn.get("x", 0.0)),
            "y": str(spawn.get("y", 0.0)),
            "z": str(spawn.get("z", 0.0)),
            "R": str(spawn.get("R", spawn.get("roll", 0.0))),
            "P": str(spawn.get("P", spawn.get("pitch", 0.0))),
            "Y": str(spawn.get("Y", spawn.get("yaw", 0.0))),
        },
    }


def delete_train_action(context):
    pkg_share = get_package_share_directory("rail_demo")
    sim_cfg = load_simulation_config(pkg_share)
    delete_entity(sim_cfg["entity_name"], sim_cfg["world_name"])
    return []


def generate_launch_description():
    pkg_share = get_package_share_directory("rail_demo")

    sim_cfg = load_simulation_config(pkg_share)
    model = sim_cfg["model_path"]
    entity_name = sim_cfg["entity_name"]
    world_name = sim_cfg["world_name"]
    spawn = sim_cfg["spawn"]

    models_path = os.path.join(pkg_share, "models")

    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_gz_path:
        gz_path = models_path + os.pathsep + existing_gz_path
    else:
        gz_path = models_path

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_path),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", gz_path),

        LogInfo(msg=f"Using model: {model}"),
        LogInfo(msg=f"Using entity name: {entity_name}"),
        LogInfo(msg=f"Using world name: {world_name}"),
        LogInfo(
            msg=(
                f"Using spawn: "
                f"x={spawn['x']} y={spawn['y']} z={spawn['z']} "
                f"R={spawn['R']} P={spawn['P']} Y={spawn['Y']}"
            )
        ),
        LogInfo(msg=f"GZ_SIM_RESOURCE_PATH: {gz_path}"),

        OpaqueFunction(function=delete_train_action),

        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-world", world_name,
                "-name", entity_name,
                "-file", model,
                "-x", spawn["x"], "-y", spawn["y"], "-z", spawn["z"],
                "-R", spawn["R"], "-P", spawn["P"], "-Y", spawn["Y"],
            ],
        ),
    ])
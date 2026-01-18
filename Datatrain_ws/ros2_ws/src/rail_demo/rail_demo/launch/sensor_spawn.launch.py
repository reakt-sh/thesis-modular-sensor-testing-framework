from launch import LaunchDescription
from launch.actions import OpaqueFunction, LogInfo
from ament_index_python.packages import get_package_share_directory

import os
import yaml
import subprocess
import math
import time


# ----------------------------
# Math utilities (same as AOI)
# ----------------------------

def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return (qx, qy, qz, qw)


# ----------------------------
# Gazebo helpers (same style as AOI)
# ----------------------------

def list_gazebo_models():
    result = subprocess.run(
        ["gz", "model", "--list"],
        capture_output=True,
        text=True
    )

    models = []
    for line in result.stdout.splitlines():
        line = line.strip()
        if line.startswith("- "):
            models.append(line[2:])
    return models


def delete_entity(entity_name):
    cmd = (
        f"ros2 service call /world/train_world/remove "
        f"ros_gz_interfaces/srv/DeleteEntity "
        f"\"{{entity: {{name: '{entity_name}', type: 2}}}}\""
    )
    subprocess.run(cmd, shell=True)
    time.sleep(0.05)


def delete_all_sensor_entities():
    """
    Deletes all Gazebo models whose name contains 'sensor' or 'lidar'
    """
    while True:
        models = list_gazebo_models()
        sensor_models = [
            m for m in models
            if "sensor" in m.lower() or "lidar" in m.lower()
        ]

        if not sensor_models:
            break

        for name in sensor_models:
            delete_entity(name)

        time.sleep(0.1)


def spawn_sensor_entity(entity_name, sdf_path, position, orientation):
    x, y, z = position
    qx, qy, qz, qw = orientation

    cmd = (
        f"ros2 service call /world/train_world/create "
        f"ros_gz_interfaces/srv/SpawnEntity "
        f"\"{{entity_factory: {{"
        f"name: '{entity_name}', "
        f"sdf_filename: '{sdf_path}', "
        f"pose: {{"
        f"position: {{x: {x}, y: {y}, z: {z}}}, "
        f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"
        f"}}, "
        f"relative_to: 'train::base_link'"
        f"}}}}\""
    )

    subprocess.run(cmd, shell=True)
    time.sleep(0.1)


# ----------------------------
# Sensor SDF generation
# ----------------------------

def generate_lidar_sdf(sensor_name, params):
    samples = params.get("horizontal_samples", 1024)
    update_rate = params.get("update_rate", 10.0)
    rmin = params.get("range_min", 0.5)
    rmax = params.get("range_max", 120.0)

    return f"""<sdf version="1.7">
  <model name="{sensor_name}">
    <static>true</static>

    <link name="sensor_link">
    <visual name="lidar_body">
      <geometry>
        <cylinder>
          <radius>0.05</radius>
          <length>0.08</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0.1 0.1 0.1 1</ambient>
        <diffuse>0.1 0.1 0.1 1</diffuse>
      </material>
    </visual>
      <sensor name="gpu_lidar" type="gpu_lidar">
        <update_rate>{update_rate}</update_rate>

        <ray>
          <scan>
            <horizontal>
              <samples>{samples}</samples>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>

          <range>
            <min>{rmin}</min>
            <max>{rmax}</max>
          </range>
        </ray>

      </sensor>

    </link>
  </model>
</sdf>
"""


def write_sensor_sdf(sensor_name, sdf_xml):
    pkg = get_package_share_directory("rail_demo")

    model_dir = os.path.join(
        pkg,
        "models",
        "generated_sensors",
        sensor_name
    )

    os.makedirs(model_dir, exist_ok=True)

    sdf_path = os.path.join(model_dir, "model.sdf")

    with open(sdf_path, "w", encoding="utf-8") as f:
        f.write(sdf_xml)

    return sdf_path


# ----------------------------
# Main launch logic
# ----------------------------

def spawn_sensors(context):
    pkg = get_package_share_directory("rail_demo")

    with open(os.path.join(pkg, "config", "generated_sensor_positions.yaml")) as f:
        slots = yaml.safe_load(f)["slots"]

    with open(os.path.join(pkg, "config", "sensors.yaml")) as f:
        sensors = yaml.safe_load(f)["sensors"]

    with open(os.path.join(pkg, "config", "sensor_assignment.yaml")) as f:
        assignments = yaml.safe_load(f)["assignments"]

    slot_map = {s["name"]: s for s in slots}

    # ---- Clean previous sensors ----
    delete_all_sensor_entities()

    # ---- Spawn sensors ----
    for slot_name, sensor_key in assignments.items():
        slot = slot_map[slot_name]
        sensor_def = sensors[sensor_key]

        # Pose is TRAIN-RELATIVE
        pos = (
            slot["pose"]["x"],
            slot["pose"]["y"],
            slot["pose"]["z"],
        )

        q = quaternion_from_rpy(
            slot["pose"]["roll"],
            slot["pose"]["pitch"],
            slot["pose"]["yaw"],
        )

        entity_name = f"{slot_name}_{sensor_key}"

        sdf_xml = generate_lidar_sdf(entity_name, sensor_def)
        sdf_path = write_sensor_sdf(entity_name, sdf_xml)

        spawn_sensor_entity(
            entity_name,
            sdf_path,
            pos,
            q
        )

    return []


# ----------------------------
# Launch description
# ----------------------------

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Spawning sensors from YAML configuration"),
        OpaqueFunction(function=spawn_sensors),
    ])
from launch import LaunchDescription
from launch.actions import OpaqueFunction, LogInfo, SetEnvironmentVariable

from ament_index_python.packages import get_package_share_directory

import os
import yaml
import subprocess
import time
import math
from lxml import etree

# ==================================================
# Package paths
# ==================================================

PKG_NAME = "rail_demo"
pkg_share = get_package_share_directory(PKG_NAME)

SENSORS_YAML_PATH = os.path.join(pkg_share, "config", "sensors.yaml")
SENSOR_ASSIGNMENT_YAML_PATH = os.path.join(pkg_share, "config", "sensor_assignment.yaml")
GENERATED_SENSOR_POS_YAML_PATH = os.path.join(
    pkg_share, "config", "generated_sensor_positions_relative.yaml"
)
SOURCE_PKG_ROOT = os.path.dirname(os.path.dirname(__file__))
SIM_CFG_SOURCE_PATH = os.path.join(SOURCE_PKG_ROOT, "config", "simulation_config.yaml")
SIM_CFG_INSTALLED_PATH = os.path.join(pkg_share, "config", "simulation_config.yaml")

GENERATED_TRAIN_DIR = os.path.join(
    pkg_share, "models", "generated_train"
)
os.makedirs(GENERATED_TRAIN_DIR, exist_ok=True)

GENERATED_TRAIN_SDF_PATH = os.path.join(
    GENERATED_TRAIN_DIR, "model_with_sensors.sdf"
)

models_path = os.path.join(pkg_share, "models")
existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
if existing_gz_path:
    gz_path = models_path + os.pathsep + existing_gz_path
else:
    gz_path = models_path


# ==================================================
# Simulation config
# ==================================================

def get_simulation_config_path():
    if os.path.exists(SIM_CFG_SOURCE_PATH):
        return SIM_CFG_SOURCE_PATH
    return SIM_CFG_INSTALLED_PATH


def load_simulation_config():
    with open(get_simulation_config_path(), "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    simulation = data.get("simulation", {})
    train = data.get("train", {})
    spawn = train.get("spawn", {})
    sensor_mount = data.get("sensor_mount", {})
    mount_pose = sensor_mount.get("pose", {})

    model_name = simulation.get("train_model")
    entity_name = simulation.get("train_entity_name", model_name or "gazebo_train")
    world_name = simulation.get("world", "train_world")

    if not model_name:
        raise ValueError("Missing 'simulation.train_model' in simulation_config.yaml")

    if world_name.endswith(".sdf"):
        world_name = os.path.splitext(world_name)[0]

    base_train_sdf_path = os.path.join(
        pkg_share, "models", "base_models", model_name, "model.sdf"
    )

    if not os.path.exists(base_train_sdf_path):
        raise FileNotFoundError(f"Configured train model not found: {base_train_sdf_path}")

    sensor_entity_name = f"{entity_name}_with_sensors"

    return {
        "model_name": model_name,
        "world_name": world_name,
        "base_train_sdf_path": base_train_sdf_path,
        "base_entity_name": entity_name,
        "sensor_entity_name": sensor_entity_name,
        "spawn": {
            "x": float(spawn.get("x", 0.0)),
            "y": float(spawn.get("y", 0.0)),
            "z": float(spawn.get("z", 0.0)),
            "roll": float(spawn.get("roll", spawn.get("R", 0.0))),
            "pitch": float(spawn.get("pitch", spawn.get("P", 0.0))),
            "yaw": float(spawn.get("yaw", spawn.get("Y", 0.0))),
        },
        "sensor_mount": {
            "x": float(mount_pose.get("x", 0.0)),
            "y": float(mount_pose.get("y", 0.0)),
            "z": float(mount_pose.get("z", 0.0)),
            "roll": float(mount_pose.get("roll", 0.0)),
            "pitch": float(mount_pose.get("pitch", 0.0)),
            "yaw": float(mount_pose.get("yaw", 0.0)),
        },
    }


# ==================================================
# Gazebo helpers
# ==================================================

def list_gazebo_models():
    result = subprocess.run(
        ["gz", "model", "--list"],
        capture_output=True,
        text=True
    )

    models = []
    for line in result.stdout.splitlines():
        if line.strip().startswith("- "):
            models.append(line.strip()[2:])
    return models


def delete_entity(entity_name):
    sim_cfg = load_simulation_config()
    world_name = sim_cfg["world_name"]

    cmd = (
        f"ros2 service call /world/{world_name}/remove "
        f"ros_gz_interfaces/srv/DeleteEntity "
        f"\"{{entity: {{name: '{entity_name}', type: 2}}}}\""
    )
    subprocess.run(cmd, shell=True)
    time.sleep(0.1)


def delete_train_if_exists():
    sim_cfg = load_simulation_config()
    train_entity_name = sim_cfg["sensor_entity_name"]

    models = list_gazebo_models()
    if train_entity_name in models:
        print(f"[SensorSpawn] Deleting existing train '{train_entity_name}'")
        delete_entity(train_entity_name)
    else:
        print("[SensorSpawn] No existing train to delete")


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

    return qx, qy, qz, qw


def spawn_train_from_sdf(sdf_path):
    sim_cfg = load_simulation_config()
    train_entity_name = sim_cfg["sensor_entity_name"]
    world_name = sim_cfg["world_name"]
    spawn = sim_cfg["spawn"]

    qx, qy, qz, qw = quaternion_from_rpy(
        spawn["roll"],
        spawn["pitch"],
        spawn["yaw"],
    )

    cmd = (
        f"ros2 service call /world/{world_name}/create "
        f"ros_gz_interfaces/srv/SpawnEntity "
        f"\"{{entity_factory: {{"
        f"name: '{train_entity_name}', "
        f"sdf_filename: '{sdf_path}', "
        f"pose: {{"
        f"position: {{x: {spawn['x']}, y: {spawn['y']}, z: {spawn['z']}}}, "
        f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"
        f"}}"
        f"}}}}\""
    )

    subprocess.run(cmd, shell=True)
    time.sleep(0.1)


# ==================================================
# Sensor mount link injection
# ==================================================

def generate_sensor_mount_link_block(sensor_mount_pose):
    pose_str = (
        f"{sensor_mount_pose['x']} {sensor_mount_pose['y']} {sensor_mount_pose['z']} "
        f"{sensor_mount_pose['roll']} {sensor_mount_pose['pitch']} {sensor_mount_pose['yaw']}"
    )

    return f"""
<link name="sensor_mount_link">
  <pose>{pose_str}</pose>
  <gravity>false</gravity>
  <self_collide>false</self_collide>
  <kinematic>true</kinematic>
  <inertial>
    <mass>0.001</mass>
    <inertia>
      <ixx>1e-6</ixx>
      <iyy>1e-6</iyy>
      <izz>1e-6</izz>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyz>0</iyz>
    </inertia>
  </inertial>
</link>

<joint name="sensor_mount_joint" type="fixed">
  <parent>base_link</parent>
  <child>sensor_mount_link</child>
</joint>
"""


# ==================================================
# Step 1 – Resolve sensor info from YAML
# ==================================================

def get_sensor_info():
    with open(SENSORS_YAML_PATH, "r", encoding="utf-8") as f:
        sensors_yaml = yaml.safe_load(f) or {}

    with open(SENSOR_ASSIGNMENT_YAML_PATH, "r", encoding="utf-8") as f:
        assignment_yaml = yaml.safe_load(f) or {}

    with open(GENERATED_SENSOR_POS_YAML_PATH, "r", encoding="utf-8") as f:
        positions_yaml = yaml.safe_load(f) or {}

    sensors_by_name = sensors_yaml.get("sensors", {})
    assignments = assignment_yaml.get("assignments", {})
    slots = positions_yaml.get("slots", [])

    slot_by_name = {
        slot["name"]: slot
        for slot in slots
        if "name" in slot
    }

    resolved = []

    for slot_name, sensor_name in assignments.items():
        if slot_name not in slot_by_name:
            print(f"[SensorSpawn] Missing slot '{slot_name}'")
            continue

        if sensor_name not in sensors_by_name:
            print(f"[SensorSpawn] Missing sensor '{sensor_name}'")
            continue

        slot_pose = slot_by_name[slot_name]["pose"]

        converted_pose = {
            "x": round(slot_pose["x"], 6),
            "y": round(slot_pose["y"], 6),
            "z": round(slot_pose["z"], 6),
            "roll": round(slot_pose.get("roll", 0.0), 6),
            "pitch": round(slot_pose.get("pitch", 0.0), 6),
            "yaw": round(slot_pose.get("yaw", 0.0), 6),
        }

        sensor_cfg = sensors_by_name[sensor_name]
        topic = sensor_cfg.get("topic", sensor_name)

        resolved.append({
            "slot_name": slot_name,
            "sensor_name": sensor_name,
            "pose": converted_pose,
            "sensor_config": sensors_by_name[sensor_name],
            "topic": topic,
        })

    print(f"[SensorSpawn] Resolved {len(resolved)} sensors")
    return resolved


# ==================================================
# Step 2 – Generate sensor SDF blocks
# ==================================================

def _pose_to_sdf(pose):
    return (
        f"{pose.get('x', 0)} "
        f"{pose.get('y', 0)} "
        f"{pose.get('z', 0)} "
        f"{pose.get('roll', 0)} "
        f"{pose.get('pitch', 0)} "
        f"{pose.get('yaw', 0)}"
    )


def generate_lidar_sensor_block(entry):
    sensor_name = entry["sensor_name"]
    slot = entry["slot_name"]
    topic_name = entry["topic"]
    pose = _pose_to_sdf(entry["pose"])
    cfg = entry["sensor_config"]
    params = cfg.get("params", {})

    topic = cfg.get("topic", topic_name)
    update_rate = cfg.get("update_rate", 10.0)

    horizontal_fov = float(params.get("horizontal_fov", 6.283185))
    vertical_fov = float(params.get("vertical_fov", 0.0))

    h_samples = int(params.get("horizontal_samples", 1024))
    h_resolution = float(params.get("horizontal_resolution", 1.0))

    v_samples = int(params.get("vertical_samples", 1))
    v_resolution = float(params.get("vertical_resolution", 1.0))

    range_min = float(params.get("range_min", 0.8))
    range_max = float(params.get("range_max", 120.0))
    range_resolution = float(params.get("range_resolution", 0.01))

    always_on = str(params.get("always_on", True)).lower()
    visualize = str(params.get("visualize", True)).lower()

    h_min = -horizontal_fov / 2.0
    h_max = horizontal_fov / 2.0

    v_min = -vertical_fov / 2.0
    v_max = vertical_fov / 2.0

    return f"""
<link name="{sensor_name}_{slot}_link">
  <pose>{pose}</pose>
  <gravity>false</gravity>
  <mass>0.01</mass>
  <inertia>
    <ixx>1e-4</ixx>
    <iyy>1e-4</iyy>
    <izz>1e-4</izz>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyz>0</iyz>
  </inertia>

  <visual name="lidar_body_visual">
    <geometry>
      <cylinder>
        <radius>0.03</radius>
        <length>0.05</length>
      </cylinder>
    </geometry>
    <material>
      <ambient>0.9 0.1 0.1 1</ambient>
      <diffuse>1.0 0.15 0.15 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
    </material>
  </visual>

  <sensor name="{sensor_name}" type="gpu_lidar">
    <pose>0 0 0 0 0 0</pose>
    <topic>{topic}</topic>
    <update_rate>{update_rate}</update_rate>
    <always_on>{always_on}</always_on>
    <visualize>{visualize}</visualize>

    <ray>
      <scan>
        <horizontal>
          <samples>{h_samples}</samples>
          <resolution>{h_resolution}</resolution>
          <min_angle>{h_min}</min_angle>
          <max_angle>{h_max}</max_angle>
        </horizontal>
        <vertical>
          <samples>{v_samples}</samples>
          <resolution>{v_resolution}</resolution>
          <min_angle>{v_min}</min_angle>
          <max_angle>{v_max}</max_angle>
        </vertical>
      </scan>

      <range>
        <min>{range_min}</min>
        <max>{range_max}</max>
        <resolution>{range_resolution}</resolution>
      </range>
    </ray>
  </sensor>
</link>

<joint name="{sensor_name}_{slot}_joint" type="fixed">
  <parent>sensor_mount_link</parent>
  <child>{sensor_name}_{slot}_link</child>
</joint>
"""


def generate_camera_sensor_block(entry):
    sensor_name = entry["sensor_name"]
    topic_name = entry["topic"]
    slot = entry["slot_name"]
    pose = _pose_to_sdf(entry["pose"])
    cfg = entry["sensor_config"]
    params = cfg.get("params", {})

    topic = cfg.get("topic", topic_name)
    update_rate = cfg.get("update_rate", 30.0)

    width = int(params.get("width", 640))
    height = int(params.get("height", 480))
    horizontal_fov = float(params.get("horizontal_fov", 1.396263))

    near_clip = float(params.get("near_clip", 0.1))
    far_clip = float(params.get("far_clip", 100.0))

    always_on = str(params.get("always_on", True)).lower()
    visualize = str(params.get("visualize", True)).lower()

    return f"""
<link name="{sensor_name}_{slot}_link">
  <pose>{pose}</pose>
  <gravity>false</gravity>
  <mass>0.01</mass>
  <inertia>
    <ixx>1e-4</ixx>
    <iyy>1e-4</iyy>
    <izz>1e-4</izz>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyz>0</iyz>
  </inertia>

  <visual name="camera_body_visual">
    <geometry>
      <cylinder>
        <radius>0.03</radius>
        <length>0.05</length>
      </cylinder>
    </geometry>
    <material>
      <ambient>0.9 0.1 0.1 1</ambient>
      <diffuse>1.0 0.15 0.15 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
    </material>
  </visual>

  <sensor name="{sensor_name}" type="camera">
    <pose>0 0 0 0 0 0</pose>
    <topic>{topic}/image</topic>
    <update_rate>{update_rate}</update_rate>
    <always_on>{always_on}</always_on>
    <visualize>{visualize}</visualize>

    <camera>
      <horizontal_fov>{horizontal_fov}</horizontal_fov>
      <image>
        <width>{width}</width>
        <height>{height}</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>{near_clip}</near>
        <far>{far_clip}</far>
      </clip>
    </camera>
  </sensor>
</link>

<joint name="{sensor_name}_{slot}_joint" type="fixed">
  <parent>sensor_mount_link</parent>
  <child>{sensor_name}_{slot}_link</child>
</joint>
"""


def generate_depth_camera_sensor_block(entry):
    sensor_name = entry["sensor_name"]
    topic_name = entry["topic"]
    slot = entry["slot_name"]
    pose = _pose_to_sdf(entry["pose"])
    cfg = entry["sensor_config"]
    params = cfg.get("params", {})

    topic = cfg.get("topic", topic_name)
    update_rate = cfg.get("update_rate", 30.0)

    width = int(params.get("width", 640))
    height = int(params.get("height", 480))
    hfov = float(params.get("horizontal_fov", 1.396263))
    near_clip = float(params.get("near_clip", 0.1))
    far_clip = float(params.get("far_clip", 50.0))

    return f"""
<link name="{sensor_name}_{slot}_link">
  <pose>{pose}</pose>
  <gravity>false</gravity>
  <always_on>true</always_on>
  <visualize>true</visualize>
  <mass>0.01</mass>
  <inertia>
    <ixx>1e-4</ixx>
    <iyy>1e-4</iyy>
    <izz>1e-4</izz>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyz>0</iyz>
  </inertia>

  <visual name="camera_depth_body_visual">
    <geometry>
      <cylinder>
        <radius>0.03</radius>
        <length>0.05</length>
      </cylinder>
    </geometry>
    <material>
      <ambient>0.9 0.1 0.1 1</ambient>
      <diffuse>1.0 0.15 0.15 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
    </material>
  </visual>

  <sensor name="{sensor_name}" type="depth_camera">
    <update_rate>{update_rate}</update_rate>
    <topic>{topic}/image</topic>
    <camera>
      <horizontal_fov>{hfov}</horizontal_fov>
      <image>
        <width>{width}</width>
        <height>{height}</height>
      </image>
      <clip>
        <near>{near_clip}</near>
        <far>{far_clip}</far>
      </clip>
    </camera>
  </sensor>
</link>

<joint name="{sensor_name}_{slot}_joint" type="fixed">
  <parent>sensor_mount_link</parent>
  <child>{sensor_name}_{slot}_link</child>
</joint>
"""


def generate_rgbd_camera_sensor_block(entry):
    sensor_name = entry["sensor_name"]
    slot = entry["slot_name"]
    pose = _pose_to_sdf(entry["pose"])
    cfg = entry["sensor_config"]
    params = cfg.get("params", {})

    width = int(params.get("width", 640))
    height = int(params.get("height", 480))
    hfov = float(params.get("horizontal_fov", 1.396263))
    topic = entry["topic"]

    return f"""
<link name="{sensor_name}_{slot}_link">
  <pose>{pose}</pose>
  <gravity>false</gravity>
  <mass>0.01</mass>
  <inertia>
    <ixx>1e-4</ixx>
    <iyy>1e-4</iyy>
    <izz>1e-4</izz>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyz>0</iyz>
  </inertia>

  <visual name="camera_rgbd_body_visual">
    <geometry>
      <cylinder>
        <radius>0.03</radius>
        <length>0.05</length>
      </cylinder>
    </geometry>
    <material>
      <ambient>0.9 0.1 0.1 1</ambient>
      <diffuse>1.0 0.15 0.15 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
    </material>
  </visual>

  <sensor name="{sensor_name}_rgb" type="camera">
    <topic>{topic}/image</topic>
    <camera>
      <horizontal_fov>{hfov}</horizontal_fov>
      <image>
        <width>{width}</width>
        <height>{height}</height>
        <format>R8G8B8</format>
      </image>
    </camera>
  </sensor>

  <sensor name="{sensor_name}_depth" type="depth_camera">
    <topic>{sensor_name}/depth</topic>
    <camera>
      <horizontal_fov>{hfov}</horizontal_fov>
      <image>
        <width>{width}</width>
        <height>{height}</height>
      </image>
    </camera>
  </sensor>
</link>

<joint name="{sensor_name}_{slot}_joint" type="fixed">
  <parent>sensor_mount_link</parent>
  <child>{sensor_name}_{slot}_link</child>
</joint>
"""


def generate_imu_sensor_block(entry):
    sensor_name = entry["sensor_name"]
    slot = entry["slot_name"]
    pose = _pose_to_sdf(entry["pose"])

    return f"""
<link name="{sensor_name}_{slot}_link">
  <pose>{pose}</pose>
  <gravity>false</gravity>
  <mass>0.01</mass>
  <inertia>
    <ixx>1e-4</ixx>
    <iyy>1e-4</iyy>
    <izz>1e-4</izz>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyz>0</iyz>
  </inertia>

  <visual name="imu_body_visual">
    <geometry>
      <cylinder>
        <radius>0.03</radius>
        <length>0.05</length>
      </cylinder>
    </geometry>
    <material>
      <ambient>0.9 0.1 0.1 1</ambient>
      <diffuse>1.0 0.15 0.15 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
    </material>
  </visual>

  <sensor name="{sensor_name}" type="imu">
    <topic>imu</topic>
    <imu>
      <angular_velocity>
        <x>true</x>
        <y>true</y>
        <z>true</z>
      </angular_velocity>
      <linear_acceleration>
        <x>true</x>
        <y>true</y>
        <z>true</z>
      </linear_acceleration>
    </imu>
  </sensor>
</link>

<joint name="{sensor_name}_{slot}_joint" type="fixed">
  <parent>sensor_mount_link</parent>
  <child>{sensor_name}_{slot}_link</child>
</joint>
"""


def generate_navsat_sensor_block(entry):
    sensor_name = entry["sensor_name"]
    slot = entry["slot_name"]
    pose = _pose_to_sdf(entry["pose"])

    return f"""
<link name="{sensor_name}_{slot}_link">
  <pose>{pose}</pose>
  <gravity>false</gravity>
  <mass>0.01</mass>
  <inertia>
    <ixx>1e-4</ixx>
    <iyy>1e-4</iyy>
    <izz>1e-4</izz>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyz>0</iyz>
  </inertia>

  <visual name="navsat_body_visual">
    <geometry>
      <cylinder>
        <radius>0.03</radius>
        <length>0.05</length>
      </cylinder>
    </geometry>
    <material>
      <ambient>0.9 0.1 0.1 1</ambient>
      <diffuse>1.0 0.15 0.15 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
    </material>
  </visual>

  <sensor name="{sensor_name}" type="navsat">
    <topic>navsat</topic>
  </sensor>
</link>

<joint name="{sensor_name}_{slot}_joint" type="fixed">
  <parent>sensor_mount_link</parent>
  <child>{sensor_name}_{slot}_link</child>
</joint>
"""


def generate_magnetometer_sensor_block(entry):
    sensor_name = entry["sensor_name"]
    slot = entry["slot_name"]
    pose = _pose_to_sdf(entry["pose"])

    return f"""
<link name="{sensor_name}_{slot}_link">
  <pose>{pose}</pose>
  <gravity>false</gravity>

  <sensor name="{sensor_name}" type="magnetometer">
    <topic>magnetic_field</topic>
  </sensor>
</link>

<joint name="{sensor_name}_{slot}_joint" type="fixed">
  <parent>sensor_mount_link</parent>
  <child>{sensor_name}_{slot}_link</child>
</joint>
"""


def generate_sensor_sdf_blocks(resolved_sensors):
    blocks = []
    for entry in resolved_sensors:
        sensor_type = entry["sensor_config"].get("type")

        if sensor_type == "gpu_lidar":
            blocks.append(generate_lidar_sensor_block(entry))
        elif sensor_type == "camera":
            blocks.append(generate_camera_sensor_block(entry))
        elif sensor_type == "depth_camera":
            blocks.append(generate_depth_camera_sensor_block(entry))
        elif sensor_type == "rgbd_camera":
            blocks.append(generate_rgbd_camera_sensor_block(entry))
        elif sensor_type == "imu":
            blocks.append(generate_imu_sensor_block(entry))
        elif sensor_type == "navsat":
            blocks.append(generate_navsat_sensor_block(entry))
        elif sensor_type == "magnetometer":
            blocks.append(generate_magnetometer_sensor_block(entry))
        else:
            print(f"[SensorSpawn] Unsupported sensor type '{sensor_type}'")

    return "\n".join(blocks)


# ==================================================
# Step 3 – Inject sensors into train SDF
# ==================================================

def inject_sensor_xml_into_sdf(sensor_xml):
    sim_cfg = load_simulation_config()
    base_train_sdf_path = sim_cfg["base_train_sdf_path"]

    parser = etree.XMLParser(remove_comments=False)
    tree = etree.parse(base_train_sdf_path, parser)
    root = tree.getroot()
    model = root.find("model")

    start_marker = None
    end_marker = None

    for node in model.iter():
        if isinstance(node, etree._Comment):
            if "AUTO-GENERATED SENSOR LINKS" in node.text:
                start_marker = node
            if "END AUTO-GENERATED SENSOR LINKS" in node.text:
                end_marker = node

    if start_marker is None or end_marker is None:
        raise RuntimeError("Sensor injection markers not found in train SDF")

    parent = start_marker.getparent()
    start_idx = parent.index(start_marker)
    end_idx = parent.index(end_marker)

    for elem in list(parent)[start_idx + 1:end_idx]:
        parent.remove(elem)

    sensor_mount_xml = generate_sensor_mount_link_block(sim_cfg["sensor_mount"])
    full_generated_xml = sensor_mount_xml + "\n" + sensor_xml

    wrapped = etree.fromstring(f"<wrapper>{full_generated_xml}</wrapper>")
    insert_idx = parent.index(start_marker) + 1

    for elem in wrapped:
        parent.insert(insert_idx, elem)
        insert_idx += 1

    tree.write(
        GENERATED_TRAIN_SDF_PATH,
        pretty_print=True,
        xml_declaration=True,
        encoding="UTF-8",
    )

    print(f"[SensorSpawn] Generated train SDF -> {GENERATED_TRAIN_SDF_PATH}")
    return GENERATED_TRAIN_SDF_PATH


# ==================================================
# Orchestration (delete → generate → spawn)
# ==================================================

def regenerate_and_spawn_train(context):
    delete_train_if_exists()

    resolved = get_sensor_info()
    sensor_xml = generate_sensor_sdf_blocks(resolved)
    sdf_path = inject_sensor_xml_into_sdf(sensor_xml)

    spawn_train_from_sdf(sdf_path)
    return []


# ==================================================
# Launch description
# ==================================================

def generate_launch_description():
    sim_cfg = load_simulation_config()

    return LaunchDescription([
        LogInfo(
            msg=(
                f"Sensor spawn launch: regenerate train with sensors "
                f"from model '{sim_cfg['model_name']}' "
                f"in world '{sim_cfg['world_name']}' "
                f"as '{sim_cfg['sensor_entity_name']}'"
            )
        ),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_path),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", gz_path),
        OpaqueFunction(function=regenerate_and_spawn_train),
    ])
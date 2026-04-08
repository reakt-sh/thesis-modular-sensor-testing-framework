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

AOI_YAML_PATH = os.path.join(pkg_share, "config", "aois.yaml")
SIM_CFG_PATH = os.path.join(pkg_share, "config", "simulation_config.yaml")

GENERATED_SENSOR_POS_WORLD_YAML_PATH = os.path.join(
    pkg_share, "config", "generated_sensor_positions_world.yaml"
)

GENERATED_SENSOR_POS_RELATIVE_YAML_PATH = os.path.join(
    pkg_share, "config", "generated_sensor_positions_relative.yaml"
)

GENERATED_TRAIN_DIR = os.path.join(pkg_share, "models", "generated_train")
os.makedirs(GENERATED_TRAIN_DIR, exist_ok=True)

GENERATED_DEBUG_TRAIN_SDF_PATH = os.path.join(
    GENERATED_TRAIN_DIR, "model_with_debug_aois.sdf"
)

DEBUG_ENTITY_NAME = "gazebo_train_debug_aoi"

models_path = os.path.join(pkg_share, "models")
existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
if existing_gz_path:
    gz_path = models_path + os.pathsep + existing_gz_path
else:
    gz_path = models_path


# ==================================================
# Quaternion / transform helpers
# ==================================================

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


def quaternion_conjugate(q):
    qx, qy, qz, qw = q
    return (-qx, -qy, -qz, qw)


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def rotate_vector_by_quaternion(v, q):
    x, y, z = v
    qx, qy, qz, qw = q

    ix = qw * x + qy * z - qz * y
    iy = qw * y + qz * x - qx * z
    iz = qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z

    rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
    ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
    rz = iz * qw + iw * -qz + ix * -qy - iy * -qx
    return (rx, ry, rz)


def quaternion_to_rpy(q):
    x, y, z, w = q

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


def pose_to_quaternion(pose_dict):
    return quaternion_from_rpy(
        pose_dict["roll"],
        pose_dict["pitch"],
        pose_dict["yaw"],
    )


def invert_pose(pos, quat):
    inv_q = quaternion_conjugate(quat)
    inv_p_rot = rotate_vector_by_quaternion((-pos[0], -pos[1], -pos[2]), inv_q)
    return inv_p_rot, inv_q


def compose_poses(pos1, q1, pos2, q2):
    pos2_rot = rotate_vector_by_quaternion(pos2, q1)
    out_pos = (
        pos1[0] + pos2_rot[0],
        pos1[1] + pos2_rot[1],
        pos1[2] + pos2_rot[2],
    )
    out_q = quaternion_multiply(q1, q2)
    return out_pos, out_q


def relative_pose(child_world_pos, child_world_q, parent_world_pos, parent_world_q):
    inv_parent_pos, inv_parent_q = invert_pose(parent_world_pos, parent_world_q)
    rel_pos, rel_q = compose_poses(inv_parent_pos, inv_parent_q, child_world_pos, child_world_q)
    rel_rpy = quaternion_to_rpy(rel_q)
    return {
        "x": round(rel_pos[0], 6),
        "y": round(rel_pos[1], 6),
        "z": round(rel_pos[2], 6),
        "roll": round(rel_rpy[0], 6),
        "pitch": round(rel_rpy[1], 6),
        "yaw": round(rel_rpy[2], 6),
    }


# ==================================================
# Config / IO helpers
# ==================================================

def load_yaml_file(path):
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_simulation_config():
    data = load_yaml_file(SIM_CFG_PATH)

    simulation = data.get("simulation", {})
    train = data.get("train", {})
    spawn = train.get("spawn", {})
    sensor_mount = data.get("sensor_mount", {})
    mount_pose = sensor_mount.get("pose", {})

    model_name = simulation.get("train_model")
    world_name = simulation.get("world")
    entity_name = simulation.get("train_entity_name", "gazebo_train")

    if not model_name:
        raise ValueError("Missing 'simulation.train_model' in simulation_config.yaml")

    if not world_name:
        raise ValueError("Missing 'simulation.world' in simulation_config.yaml")

    if world_name.endswith(".sdf"):
        world_name = os.path.splitext(world_name)[0]

    base_train_sdf_path = os.path.join(
        pkg_share, "models", "base_models", model_name, "model.sdf"
    )

    if not os.path.exists(base_train_sdf_path):
        raise FileNotFoundError(f"Configured train model not found: {base_train_sdf_path}")

    return {
        "model_name": model_name,
        "world_name": world_name,
        "entity_name": entity_name,
        "base_train_sdf_path": base_train_sdf_path,
        "spawn": {
            "x": float(spawn.get("x", 0.0)),
            "y": float(spawn.get("y", 0.0)),
            "z": float(spawn.get("z", 0.0)),
            "R": float(spawn.get("R", spawn.get("roll", 0.0))),
            "P": float(spawn.get("P", spawn.get("pitch", 0.0))),
            "Y": float(spawn.get("Y", spawn.get("yaw", 0.0))),
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


def load_aoi_config():
    return load_yaml_file(AOI_YAML_PATH)


def load_world_slot_positions():
    data = load_yaml_file(GENERATED_SENSOR_POS_WORLD_YAML_PATH)
    return data.get("slots", [])


def write_relative_sensor_positions_yaml(slots):
    with open(GENERATED_SENSOR_POS_RELATIVE_YAML_PATH, "w", encoding="utf-8") as f:
        yaml.safe_dump({"slots": slots}, f, sort_keys=False)

    print(
        "[DebugAOI] Wrote sensor_mount_link-relative slots -> "
        f"{GENERATED_SENSOR_POS_RELATIVE_YAML_PATH}"
    )


# ==================================================
# Train / frame helpers
# ==================================================

def get_train_spawn_pose():
    sim_cfg = load_simulation_config()
    spawn = sim_cfg["spawn"]

    position = (
        spawn["x"],
        spawn["y"],
        spawn["z"],
    )

    orientation = quaternion_from_rpy(
        spawn["R"],
        spawn["P"],
        spawn["Y"],
    )

    return position, orientation


def get_sensor_mount_local_pose():
    sim_cfg = load_simulation_config()
    sm = sim_cfg["sensor_mount"]

    local_pos = (sm["x"], sm["y"], sm["z"])
    local_q = quaternion_from_rpy(sm["roll"], sm["pitch"], sm["yaw"])

    return local_pos, local_q, sm


def get_sensor_mount_world_pose():
    train_world_pos, train_world_q = get_train_spawn_pose()
    sensor_mount_local_pos, sensor_mount_local_q, _ = get_sensor_mount_local_pose()

    sensor_mount_world_pos, sensor_mount_world_q = compose_poses(
        train_world_pos,
        train_world_q,
        sensor_mount_local_pos,
        sensor_mount_local_q,
    )

    return sensor_mount_world_pos, sensor_mount_world_q


# ==================================================
# Gazebo helpers
# ==================================================

def list_gazebo_models():
    result = subprocess.run(
        ["gz", "model", "--list"],
        capture_output=True,
        text=True,
    )

    models = []
    for line in result.stdout.splitlines():
        if line.strip().startswith("- "):
            models.append(line.strip()[2:])
    return models


def delete_entity(entity_name, world_name):
    cmd = (
        f"ros2 service call /world/{world_name}/remove "
        f"ros_gz_interfaces/srv/DeleteEntity "
        f"\"{{entity: {{name: '{entity_name}', type: 2}}}}\""
    )
    subprocess.run(cmd, shell=True)
    time.sleep(0.1)


def delete_debug_train_if_exists():
    sim_cfg = load_simulation_config()
    world_name = sim_cfg["world_name"]

    models = list_gazebo_models()
    if DEBUG_ENTITY_NAME in models:
        print(f"[DebugAOI] Deleting existing debug train '{DEBUG_ENTITY_NAME}'")
        delete_entity(DEBUG_ENTITY_NAME, world_name)
    else:
        print("[DebugAOI] No existing debug train to delete")


def spawn_train_from_sdf(entity_name, sdf_path):
    sim_cfg = load_simulation_config()
    world_name = sim_cfg["world_name"]

    position, orientation = get_train_spawn_pose()
    x, y, z = position
    qx, qy, qz, qw = orientation

    cmd = (
        f"ros2 service call /world/{world_name}/create "
        f"ros_gz_interfaces/srv/SpawnEntity "
        f"\"{{entity_factory: {{"
        f"name: '{entity_name}', "
        f"sdf_filename: '{sdf_path}', "
        f"pose: {{"
        f"position: {{x: {x}, y: {y}, z: {z}}}, "
        f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"
        f"}}"
        f"}}}}\""
    )

    subprocess.run(cmd, shell=True)
    time.sleep(0.1)


# ==================================================
# SDF generation blocks
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


def generate_aoi_link_block(name, pose, width, depth, color):
    r, g, b, a = color
    pose_str = (
        f"{pose['x']} {pose['y']} {pose['z']} "
        f"{pose['roll']} {pose['pitch']} {pose['yaw']}"
    )
    thickness = 0.01

    return f"""
<link name="{name}_aoi_link">
  <pose relative_to="sensor_mount_link">{pose_str}</pose>
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

  <visual name="{name}_aoi_visual">
    <geometry>
      <box>
        <size>{width} {depth} {thickness}</size>
      </box>
    </geometry>
    <material>
      <ambient>{r} {g} {b} {a}</ambient>
      <diffuse>{r} {g} {b} {a}</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
    </material>
  </visual>
</link>

<joint name="{name}_aoi_joint" type="fixed">
  <parent>sensor_mount_link</parent>
  <child>{name}_aoi_link</child>
</joint>
"""
    r, g, b, a = color
    pose_str = (
        f"{pose['x']} {pose['y']} {pose['z']} "
        f"{pose['roll']} {pose['pitch']} {pose['yaw']}"
    )
    thickness = 0.01

    return f"""
<link name="{name}_aoi_link">
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

  <visual name="{name}_aoi_visual">
    <geometry>
      <box>
        <size>{width} {depth} {thickness}</size>
      </box>
    </geometry>
    <material>
      <ambient>{r} {g} {b} {a}</ambient>
      <diffuse>{r} {g} {b} {a}</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
    </material>
  </visual>
</link>

<joint name="{name}_aoi_joint" type="fixed">
  <parent>sensor_mount_link</parent>
  <child>{name}_aoi_link</child>
</joint>
"""


def generate_slot_marker_block(aoi_name, slot_name, local_pose):
    pose_str = (
        f"{local_pose['x']} {local_pose['y']} {local_pose['z']} "
        f"{local_pose['roll']} {local_pose['pitch']} {local_pose['yaw']}"
    )

    return f"""
<link name="{slot_name}_link">
  <pose relative_to="{aoi_name}_aoi_link">{pose_str}</pose>
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

  <visual name="{slot_name}_visual">
    <geometry>
      <sphere>
        <radius>0.03</radius>
      </sphere>
    </geometry>
    <material>
      <ambient>1.0 0.1 0.1 1</ambient>
      <diffuse>1.0 0.1 0.1 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
    </material>
  </visual>
</link>

<joint name="{slot_name}_joint" type="fixed">
  <parent>{aoi_name}_aoi_link</parent>
  <child>{slot_name}_link</child>
</joint>
"""

    return f"""
<link name="{slot_name}_link">
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

  <visual name="{slot_name}_visual">
    <geometry>
      <sphere>
        <radius>0.03</radius>
      </sphere>
    </geometry>
    <material>
      <ambient>1.0 0.1 0.1 1</ambient>
      <diffuse>1.0 0.1 0.1 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
    </material>
  </visual>
</link>

<joint name="{slot_name}_joint" type="fixed">
  <parent>{aoi_name}_aoi_link</parent>
  <child>{slot_name}_link</child>
</joint>
"""


# ==================================================
# Debug build logic
# ==================================================

def build_debug_aoi_xml_and_slots():
    aoi_cfg = load_aoi_config()
    world_slots = load_world_slot_positions()
    sim_cfg = load_simulation_config()

    sensor_mount_world_pos, sensor_mount_world_q = get_sensor_mount_world_pose()
    _, _, sensor_mount_pose = get_sensor_mount_local_pose()

    aoi_by_name = {}
    for aoi in aoi_cfg.get("aoi", {}).values():
        aoi_by_name[aoi["entity_name"]] = aoi

    debug_xml_blocks = []
    relative_slots = []

    # 1) sensor_mount_link under base_link
    debug_xml_blocks.append(generate_sensor_mount_link_block(sensor_mount_pose))

    # 2) AOI planes: world -> sensor_mount_link relative
    aoi_relative_lookup = {}
    aoi_world_pose_lookup = {}

    for aoi_name, aoi in aoi_by_name.items():
        pose = aoi["pose"]
        aoi_world_pos = (pose["x"], pose["y"], pose["z"])
        aoi_world_q = quaternion_from_rpy(
            pose["roll"], pose["pitch"], pose["yaw"]
        )

        aoi_world_pose_lookup[aoi_name] = {
            "pos": aoi_world_pos,
            "q": aoi_world_q,
        }

        aoi_sensor_pose = relative_pose(
            aoi_world_pos,
            aoi_world_q,
            sensor_mount_world_pos,
            sensor_mount_world_q,
        )

        aoi_relative_lookup[aoi_name] = aoi_sensor_pose

        size = aoi["size"]
        color = aoi.get("visual", {}).get("color", [0.2, 0.8, 0.2, 0.4])

        debug_xml_blocks.append(
            generate_aoi_link_block(
                name=aoi_name,
                pose=aoi_sensor_pose,
                width=size["width"],
                depth=size["depth"],
                color=color,
            )
        )

        print(f"[DebugAOI] AOI {aoi_name} relative to sensor_mount_link: {aoi_sensor_pose}")

    # 3) Slot markers: world slot pose -> AOI-relative for SDF, and world slot pose -> sensor_mount relative for export
    for slot in world_slots:
        slot_name = slot["name"]
        aoi_name = slot["aoi"]

        if aoi_name not in aoi_world_pose_lookup:
            print(f"[DebugAOI][WARN] AOI '{aoi_name}' for slot '{slot_name}' not found in aois.yaml, skipping")
            continue

        slot_world_pose = slot["pose"]
        slot_world_pos = (
            float(slot_world_pose["x"]),
            float(slot_world_pose["y"]),
            float(slot_world_pose["z"]),
        )
        slot_world_q = quaternion_from_rpy(
            float(slot_world_pose.get("roll", 0.0)),
            float(slot_world_pose.get("pitch", 0.0)),
            float(slot_world_pose.get("yaw", 0.0)),
        )

        aoi_world_pos = aoi_world_pose_lookup[aoi_name]["pos"]
        aoi_world_q = aoi_world_pose_lookup[aoi_name]["q"]

        # For SDF slot marker injection: slot pose relative to AOI link
        slot_aoi_pose = relative_pose(
            slot_world_pos,
            slot_world_q,
            aoi_world_pos,
            aoi_world_q,
        )

        debug_xml_blocks.append(
            generate_slot_marker_block(aoi_name, slot_name, slot_aoi_pose)
        )

        # For final export: slot pose relative to sensor_mount_link
        slot_sensor_pose = relative_pose(
            slot_world_pos,
            slot_world_q,
            sensor_mount_world_pos,
            sensor_mount_world_q,
        )

        relative_slots.append({
            "name": slot_name,
            "aoi": aoi_name,
            "grid": slot.get("grid", {}),
            "pose": slot_sensor_pose,
            "frame": "sensor_mount_link",
            "sensor": slot.get("sensor", None),
        })

        print(
            f"[DebugAOI] Slot {slot_name}: "
            f"AOI-relative={slot_aoi_pose} | "
            f"sensor_mount_link-relative={slot_sensor_pose}"
        )

    return "\n".join(debug_xml_blocks), relative_slots


# ==================================================
# SDF manipulation
# ==================================================

def strip_train_visuals(model):
    visuals_to_remove = []
    for visual in model.iter("visual"):
        visuals_to_remove.append(visual)

    for visual in visuals_to_remove:
        parent = visual.getparent()
        if parent is not None:
            parent.remove(visual)

    print(f"[DebugAOI] Removed {len(visuals_to_remove)} existing train visuals")


def inject_debug_xml_into_sdf(debug_xml):
    sim_cfg = load_simulation_config()
    base_train_sdf_path = sim_cfg["base_train_sdf_path"]

    parser = etree.XMLParser(remove_comments=False)
    tree = etree.parse(base_train_sdf_path, parser)
    root = tree.getroot()
    model = root.find("model")

    if model is None:
        raise RuntimeError("No <model> element found in base train SDF")

    strip_train_visuals(model)

    start_marker = None
    end_marker = None

    for node in model.iter():
        if isinstance(node, etree._Comment):
            if "AUTO-GENERATED SENSOR LINKS" in node.text:
                start_marker = node
            if "END AUTO-GENERATED SENSOR LINKS" in node.text:
                end_marker = node

    if start_marker is None or end_marker is None:
        raise RuntimeError(
            "Injection markers not found in train SDF. "
            "Re-use the existing AUTO-GENERATED SENSOR LINKS comment block."
        )

    parent = start_marker.getparent()
    start_idx = parent.index(start_marker)
    end_idx = parent.index(end_marker)

    for elem in list(parent)[start_idx + 1:end_idx]:
        parent.remove(elem)

    wrapped = etree.fromstring(f"<wrapper>{debug_xml}</wrapper>")
    insert_idx = parent.index(start_marker) + 1

    for elem in wrapped:
        parent.insert(insert_idx, elem)
        insert_idx += 1

    tree.write(
        GENERATED_DEBUG_TRAIN_SDF_PATH,
        pretty_print=True,
        xml_declaration=True,
        encoding="UTF-8",
    )

    print(f"[DebugAOI] Generated debug train SDF -> {GENERATED_DEBUG_TRAIN_SDF_PATH}")
    return GENERATED_DEBUG_TRAIN_SDF_PATH


# ==================================================
# Orchestration
# ==================================================

def regenerate_and_spawn_debug_train(context):
    delete_debug_train_if_exists()

    debug_xml, relative_slots = build_debug_aoi_xml_and_slots()
    sdf_path = inject_debug_xml_into_sdf(debug_xml)
    write_relative_sensor_positions_yaml(relative_slots)

    spawn_train_from_sdf(DEBUG_ENTITY_NAME, sdf_path)
    return []


# ==================================================
# Launch description
# ==================================================

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Debug AOI-on-train launch: sensor_mount_link + AOIs + slot markers"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_path),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", gz_path),
        OpaqueFunction(function=regenerate_and_spawn_debug_train),
    ])
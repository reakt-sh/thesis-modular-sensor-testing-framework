from launch import LaunchDescription
from launch.actions import OpaqueFunction, LogInfo
from ament_index_python.packages import get_package_share_directory

import os
import yaml
import subprocess
import math
import time


# ----------------------------
# Math utilities
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


def rotate_vector_by_quaternion(v, q):
    x, y, z = v
    qx, qy, qz, qw = q

    # Quaternion-vector multiplication (q * v * q^-1)
    ix =  qw * x + qy * z - qz * y
    iy =  qw * y + qz * x - qx * z
    iz =  qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z

    rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
    ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
    rz = iz * qw + iw * -qz + ix * -qy - iy * -qx

    return (rx, ry, rz)


# ----------------------------
# Simulation config helpers
# ----------------------------

def load_simulation_config():
    pkg = get_package_share_directory("rail_demo")
    cfg_path = os.path.join(pkg, "config", "simulation_config.yaml")

    if not os.path.exists(cfg_path):
        raise FileNotFoundError(f"simulation_config.yaml not found: {cfg_path}")

    with open(cfg_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    simulation = data.get("simulation", {})
    world_name = simulation.get("world")

    if not world_name:
        raise ValueError("Missing 'simulation.world' in simulation_config.yaml")

    if world_name.endswith(".sdf"):
        world_name = os.path.splitext(world_name)[0]

    return {
        "world_name": world_name,
    }


# ----------------------------
# Gazebo helpers
# ----------------------------

def delete_entity(entity_name, world_name):
    cmd = (
        f"ros2 service call /world/{world_name}/remove "
        f"ros_gz_interfaces/srv/DeleteEntity "
        f"\"{{entity: {{name: '{entity_name}', type: 2}}}}\""
    )
    subprocess.run(cmd, shell=True)
    time.sleep(0.01)


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


def delete_all_aoi_entities(world_name):
    models = list_gazebo_models()
    for m in models:
        if "aoi" in m.lower():
            delete_entity(m, world_name)


def spawn_entity(entity_name, sdf_filename, position, orientation, world_name):
    x, y, z = position
    qx, qy, qz, qw = orientation

    cmd = (
        f"ros2 service call /world/{world_name}/create "
        f"ros_gz_interfaces/srv/SpawnEntity "
        f"\"{{entity_factory: {{"
        f"name: '{entity_name}', "
        f"sdf_filename: '{sdf_filename}', "
        f"pose: {{"
        f"position: {{x: {x}, y: {y}, z: {z}}}, "
        f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"
        f"}}"
        f"}}}}\""
    )

    subprocess.run(cmd, shell=True)
    time.sleep(0.01)


# ----------------------------
# SDF helpers
# ----------------------------

def generate_plane_sdf(entity_name, width, depth, color):
    thickness = 0.01
    r, g, b, a = color

    return f"""<sdf version="1.7">
  <model name="{entity_name}">
    <static>true</static>
    <link name="plane_link">
      <visual name="visual">
        <geometry>
          <box>
            <size>{width} {depth} {thickness}</size>
          </box>
        </geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def write_plane_model(entity_name, sdf_xml):
    pkg = get_package_share_directory("rail_demo")
    model_dir = os.path.join(pkg, "models", "generated_aois", entity_name)
    os.makedirs(model_dir, exist_ok=True)

    sdf_path = os.path.join(model_dir, "model.sdf")
    with open(sdf_path, "w", encoding="utf-8") as f:
        f.write(sdf_xml)

    return sdf_path


def get_grid_point_sdf():
    pkg = get_package_share_directory("rail_demo")
    return os.path.join(
        pkg, "models", "generated_aois", "grid_point", "model.sdf"
    )


# ----------------------------
# Grid logic (LOCAL to AOI)
# ----------------------------

def generate_centered_grid_points(width, depth, rows, cols):
    points = []
    dx = width / cols
    dy = depth / rows

    for r in range(rows):
        for c in range(cols):
            x = -width / 2 + (c + 0.5) * dx
            y = -depth / 2 + (r + 0.5) * dy
            points.append((r, c, (x, y, 0.0)))

    return points


# ----------------------------
# YAML export
# ----------------------------

def write_sensor_positions_yaml(slots):
    pkg = get_package_share_directory("rail_demo")
    path = os.path.join(pkg, "config", "generated_sensor_positions_world.yaml")

    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump({"slots": slots}, f, sort_keys=False)

    print(f"[AOI] Wrote sensor slots -> {path}")


# ----------------------------
# Main logic
# ----------------------------

def spawn_aois(context):
    pkg = get_package_share_directory("rail_demo")
    cfg_path = os.path.join(pkg, "config", "aois.yaml")

    with open(cfg_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    sim_cfg = load_simulation_config()
    world_name = sim_cfg["world_name"]

    delete_all_aoi_entities(world_name)

    grid_sdf = get_grid_point_sdf()
    sensor_slots = []

    for aoi in cfg["aoi"].values():
        name = aoi["entity_name"]

        pose = aoi["pose"]
        pos_world = (pose["x"], pose["y"], pose["z"])
        q_world = quaternion_from_rpy(
            pose["roll"], pose["pitch"], pose["yaw"]
        )

        size = aoi["size"]
        color = aoi.get("visual", {}).get("color", [0.2, 0.8, 0.2, 0.4])

        sdf_xml = generate_plane_sdf(
            name, size["width"], size["depth"], color
        )
        sdf_path = write_plane_model(name, sdf_xml)

        spawn_entity(name, sdf_path, pos_world, q_world, world_name)

        if "grid" in aoi:
            grid = aoi["grid"]
            local_pts = generate_centered_grid_points(
                size["width"], size["depth"],
                grid["rows"], grid["cols"]
            )
            for r, c, p_local in local_pts:
                dot_name = f"{name}_r{r}_c{c}"

                # 1. Rotate local grid point by AOI orientation
                p_rot = rotate_vector_by_quaternion(p_local, q_world)

                # 2. Translate into world space (relative to AOI center)
                p_world = (
                    pos_world[0] + p_rot[0],
                    pos_world[1] + p_rot[1],
                    pos_world[2] + p_rot[2],
                )

                # 3. Spawn grid dot at unique world pose
                spawn_entity(
                    dot_name,
                    grid_sdf,
                    p_world,
                    q_world,
                    world_name
                )

                sensor_slots.append({
                    "name": f"{name}_r{r}_c{c}",
                    "aoi": name,
                    "grid": {"row": r, "col": c},
                    "pose": {
                        "x": round(p_world[0], 6),
                        "y": round(p_world[1], 6),
                        "z": round(p_world[2], 6),
                        "roll": round(pose["roll"], 6),
                        "pitch": round(pose["pitch"], 6),
                        "yaw": round(pose["yaw"], 6),
                    },
                    "sensor": None
                })

    write_sensor_positions_yaml(sensor_slots)
    return []


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="AOI spawn v2: world-pose-based AOI placement"),
        OpaqueFunction(function=spawn_aois),
    ])
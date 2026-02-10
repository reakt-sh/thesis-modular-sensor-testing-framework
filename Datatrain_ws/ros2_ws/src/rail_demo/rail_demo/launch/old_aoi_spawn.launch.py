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

    ix =  qw * x + qy * z - qz * y
    iy =  qw * y + qz * x - qx * z
    iz =  qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z

    rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
    ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
    rz = iz * qw + iw * -qz + ix * -qy - iy * -qx

    return (rx, ry, rz)


def multiply_quaternions(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2

    return (x, y, z, w)


# ----------------------------
# Gazebo helpers
# ----------------------------

def delete_entity(entity_name):
    cmd = (
        f"ros2 service call /world/train_world/remove "
        f"ros_gz_interfaces/srv/DeleteEntity "
        f"\"{{entity: {{name: '{entity_name}', type: 2}}}}\""
    )
    subprocess.run(cmd, shell=True)
    time.sleep(0.001)


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


def delete_all_aoi_entities():
    while True:
        models = list_gazebo_models()
        aoi_models = [m for m in models if "aoi" in m.lower()]

        if not aoi_models:
            break

        for m in aoi_models:
            delete_entity(m)


def spawn_entity(entity_name, sdf_filename, position, orientation):
    x, y, z = position
    qx, qy, qz, qw = orientation

    cmd = (
        f"ros2 service call /world/train_world/create "
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
    time.sleep(0.001)


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


def write_plane_model(sdf_xml):
    pkg = get_package_share_directory("rail_demo")
    model_dir = os.path.join(pkg, "models", "generated_aois", "top")
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
# Grid logic
# ----------------------------

def generate_centered_grid_points(width, depth, rows, cols):
    points = []
    dx = width / cols
    dy = depth / rows

    for r in range(rows):
        for c in range(cols):
            x = -width/2 + (c + 0.5) * dx
            y = -depth/2 + (r + 0.5) * dy
            points.append((r, c, (x, y, 0.0)))

    return points


def transform_grid_points(
    grid_points, aoi_q, aoi_offset, train_q, train_pos
):
    out = []

    for r, c, p in grid_points:
        p1 = rotate_vector_by_quaternion(p, aoi_q)
        p2 = (
            p1[0] + aoi_offset[0],
            p1[1] + aoi_offset[1],
            p1[2] + aoi_offset[2],
        )
        p3 = rotate_vector_by_quaternion(p2, train_q)
        pw = (
            p3[0] + train_pos[0],
            p3[1] + train_pos[1],
            p3[2] + train_pos[2],
        )

        out.append((r, c, p2, pw))

    return out


# ----------------------------
# YAML export
# ----------------------------

def write_sensor_positions_yaml(slots):
    pkg = get_package_share_directory("rail_demo")
    path = os.path.join(pkg, "config", "generated_sensor_positions.yaml")

    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump({"slots": slots}, f, sort_keys=False)

    print(f"[AOI] Wrote sensor slots → {path}")


# ----------------------------
# Main logic
# ----------------------------

def spawn_aois(context):
    pkg = get_package_share_directory("rail_demo")
    cfg_path = os.path.join(pkg, "config", "aois.yaml")

    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)

    delete_all_aoi_entities()

    train = cfg["train_spawn"]
    train_pos = (train["x"], train["y"], train["z"])
    train_q = quaternion_from_rpy(train["R"], train["P"], train["Y"])

    grid_sdf = get_grid_point_sdf()
    sensor_slots = []

    for aoi in cfg["aoi"].values():
        name = aoi["entity_name"]

        offset = aoi["offset"]
        offset_local = (offset["x"], offset["y"], offset["z"])

        rpy = aoi["orientation_rpy"]
        aoi_q = quaternion_from_rpy(rpy["roll"], rpy["pitch"], rpy["yaw"])

        offset_world = rotate_vector_by_quaternion(offset_local, train_q)
        pos_world = (
            train_pos[0] + offset_world[0],
            train_pos[1] + offset_world[1],
            train_pos[2] + offset_world[2],
        )
        q_world = multiply_quaternions(train_q, aoi_q)

        size = aoi["size"]
        color = aoi.get("visual", {}).get("color", [0.2, 0.8, 0.2, 0.4])

        # --- Spawn AOI plane FIRST ---
        sdf_xml = generate_plane_sdf(name, size["width"], size["depth"], color)
        sdf_path = write_plane_model(sdf_xml)

        spawn_entity(name, sdf_path, pos_world, q_world)

        # --- Grid dots AFTER ---
        if "grid" in aoi:
            grid = aoi["grid"]
            local_pts = generate_centered_grid_points(
                size["width"], size["depth"],
                grid["rows"], grid["cols"]
            )

            world_pts = transform_grid_points(
                local_pts, aoi_q, offset_local, train_q, train_pos
            )

            for r, c, p_train, p_world in world_pts:
                dot_name = f"{name}_grid_r{r}_c{c}"

                spawn_entity(
                    dot_name,
                    grid_sdf,
                    p_world,
                    (0, 0, 0, 1)
                )

                sensor_slots.append({
                    "name": f"{name}_r{r}_c{c}",
                    "aoi": name,
                    "grid": {"row": r, "col": c},
                    "pose": {
                        "x": round(p_train[0], 6),
                        "y": round(p_train[1], 6),
                        "z": round(p_train[2], 6),
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                    },
                    "sensor": None
                })

    write_sensor_positions_yaml(sensor_slots)
    return []


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="AOI spawn launch: clean AOI + grid spawn"),
        OpaqueFunction(function=spawn_aois),
    ])
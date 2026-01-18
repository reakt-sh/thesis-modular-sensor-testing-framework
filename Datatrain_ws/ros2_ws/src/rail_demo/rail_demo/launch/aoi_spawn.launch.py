from launch import LaunchDescription
from launch.actions import OpaqueFunction, LogInfo
from ament_index_python.packages import get_package_share_directory

import os
import yaml
import subprocess
import math
import time

# Math utilities
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



# Gazebo service helpers

def delete_entity(entity_name):
    cmd = (
        f"ros2 service call /world/train_world/remove "
        f"ros_gz_interfaces/srv/DeleteEntity "
        f"\"{{entity: {{name: '{entity_name}', type: 2}}}}\""
    )
    subprocess.run(cmd, shell=True)
    time.sleep(0.1)

def list_gazebo_models():
    """
    Returns a list of model names currently in the Gazebo world.
    """
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

def delete_all_aoi_entities():
    """
    Deletes all Gazebo models whose name contains 'aoi'
    (case-insensitive), repeating until none remain.
    """
    while True:
        models = list_gazebo_models()

        aoi_models = [
            m for m in models
            if "aoi" in m.lower()
        ]

        if not aoi_models:
            break  # fully clean

        for name in aoi_models:
            delete_entity(name)

        time.sleep(0.1)


def spawn_plane_entity(entity_name, sdf_filename, position, orientation):
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
    time.sleep(0.1)


# AOI SDF generator

def get_grid_point_sdf_path():
    pkg_share = get_package_share_directory("rail_demo")
    return os.path.join(
        pkg_share,
        "models",
        "generated_aois",
        "grid_point",
        "model.sdf"
    )


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
</sdf>"""


def write_sdf_to_model_folder(sdf_xml):
    pkg_share = get_package_share_directory("rail_demo")

    model_dir = os.path.join(
        pkg_share,
        "rail_demo",
        "models",
        "generated_aois",
        "top"
    )

    os.makedirs(model_dir, exist_ok=True)

    sdf_path = os.path.join(model_dir, "model.sdf")

    with open(sdf_path, "w", encoding="utf-8") as f:
        f.write(sdf_xml)

    return sdf_path


# centered grid generation 
def generate_centered_grid_points(width, depth, rows, cols):
    """
    Generate centered grid points on an AOI plane in the AOI-local frame.

    Plane-local frame:
      x ∈ [-width/2, +width/2]
      y ∈ [-depth/2, +depth/2]
      z = 0

    Returns:
      List of (row, col, (x, y, z))
    """
    points = []

    dx = width / cols
    dy = depth / rows

    for r in range(rows):
        for c in range(cols):
            x = -width / 2.0 + (c + 0.5) * dx
            y = -depth / 2.0 + (r + 0.5) * dy
            z = 0.0
            points.append((r, c, (x, y, z)))

    return points

# Logic for Grid Points generted on Aoi
def transform_grid_points_to_world(
    grid_points,
    aoi_q_local,
    aoi_offset_local,
    train_q,
    train_pos
):
    """
    Transform AOI-local grid points into train and world frames.

    Returns:
      List of dicts with:
        - row, col
        - position_train
        - position_world
    """
    transformed = []

    for r, c, p_local in grid_points:
        # Rotate by AOI orientation (still in train frame)
        p_aoi_rot = rotate_vector_by_quaternion(p_local, aoi_q_local)

        # Translate by AOI offset (train frame)
        p_train = (
            p_aoi_rot[0] + aoi_offset_local[0],
            p_aoi_rot[1] + aoi_offset_local[1],
            p_aoi_rot[2] + aoi_offset_local[2],
        )

        # Rotate by train orientation (world frame)
        p_world_rot = rotate_vector_by_quaternion(p_train, train_q)

        # Translate by train position
        p_world = (
            p_world_rot[0] + train_pos[0],
            p_world_rot[1] + train_pos[1],
            p_world_rot[2] + train_pos[2],
        )

        transformed.append({
            "row": r,
            "col": c,
            "position_train": p_train,
            "position_world": p_world,
        })

    return transformed

# Main launch logic

def spawn_aois(context):
    pkg_share = get_package_share_directory("rail_demo")
    yaml_path = os.path.join(pkg_share, "config", "aois.yaml")

    with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f)

    # ---- Train pose (trusted from YAML) ----
    train = cfg["train_spawn"]

    train_pos = (train["x"], train["y"], train["z"])
    train_q = quaternion_from_rpy(
        train["R"], train["P"], train["Y"]
    )

    # Pre-resolve grid point model path (static, reused)
    grid_point_sdf = get_grid_point_sdf_path()

    # ---- Iterate AOIs ----
    for _, aoi in cfg.get("aoi", {}).items():
        entity_name = aoi["entity_name"]

        # ---- Delete existing AOI plane ----
        delete_entity(entity_name)

        # ---- AOI offset and orientation (train frame) ----
        offset = aoi["offset"]
        offset_local = (offset["x"], offset["y"], offset["z"])

        rpy = aoi["orientation_rpy"]
        aoi_q_local = quaternion_from_rpy(
            rpy["roll"], rpy["pitch"], rpy["yaw"]
        )

        # ---- AOI pose in world frame ----
        offset_world = rotate_vector_by_quaternion(offset_local, train_q)
        aoi_pos_world = (
            train_pos[0] + offset_world[0],
            train_pos[1] + offset_world[1],
            train_pos[2] + offset_world[2],
        )

        aoi_q_world = multiply_quaternions(train_q, aoi_q_local)

        # ---- AOI parameters ----
        size = aoi["size"]
        color = aoi.get("visual", {}).get("color", [0.2, 0.8, 0.2, 0.4])

       
        # AOI grid computation + visualization
    
        grid_cfg = aoi.get("grid", None)
        if grid_cfg:
            # Generate centered grid points (AOI-local)
            grid_points_local = generate_centered_grid_points(
                width=size["width"],
                depth=size["depth"],
                rows=grid_cfg["rows"],
                cols=grid_cfg["cols"],
            )

            # Transform grid points into train/world frames
            grid_points_world = transform_grid_points_to_world(
                grid_points=grid_points_local,
                aoi_q_local=aoi_q_local,
                aoi_offset_local=offset_local,
                train_q=train_q,
                train_pos=train_pos,
            )


            # Spawn grid point visualization markers
            for gp in grid_points_world:
                r = gp["row"]
                c = gp["col"]
                pos = gp["position_world"]

                grid_name = f"{entity_name}_grid_r{r}_c{c}"

                # Spawn static grid marker
                spawn_plane_entity(
                    grid_name,
                    grid_point_sdf,
                    pos,
                    (0.0, 0.0, 0.0, 1.0)
                )

      
        # AOI plane spawning
       
        sdf_xml = generate_plane_sdf(
            entity_name,
            size["width"],
            size["depth"],
            color
        )

        sdf_path = write_sdf_to_model_folder(sdf_xml)

        delete_all_aoi_entities()
        
        spawn_plane_entity(
            entity_name,
            sdf_path,
            aoi_pos_world,
            aoi_q_world
        )

    return []


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="AOI spawn launch: deleting and respawning AOIs from YAML"),
        OpaqueFunction(function=spawn_aois),
    ])
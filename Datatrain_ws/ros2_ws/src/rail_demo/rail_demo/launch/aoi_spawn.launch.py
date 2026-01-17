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
    # v' = q * (v,0) * q^-1
    x, y, z = v
    qx, qy, qz, qw = q

    # Quaternion-vector multiplication
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
# Gazebo service helpers
# ----------------------------

def delete_entity(entity_name):
    cmd = (
        f"ros2 service call /world/train_world/remove "
        f"ros_gz_interfaces/srv/DeleteEntity "
        f"\"{{entity: {{name: '{entity_name}', type: 2}}}}\""
    )
    subprocess.run(cmd, shell=True)
    time.sleep(0.1)


def spawn_plane_entity(entity_name, sdf, position, orientation):
    x, y, z = position
    qx, qy, qz, qw = orientation

    # Escape quotes for shell
    sdf_escaped = sdf.replace('"', "'")

    cmd = (
        f"ros2 service call /world/train_world/create "
        f"ros_gz_interfaces/srv/SpawnEntity "
        f"\"{{entity_factory: {{"
        f"name: '{entity_name}', "
        f"sdf: \"{sdf_escaped}\", "
        f"pose: {{"
        f"position: {{x: {x}, y: {y}, z: {z}}}, "
        f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"
        f"}}"
        f"}}}}\""
    )

    subprocess.run(cmd, shell=True)
    time.sleep(0.1)


# ----------------------------
# AOI SDF generator
# ----------------------------

def generate_plane_sdf(entity_name, width, depth, color):
    thickness = 0.01
    r, g, b, a = color

    return f"""
<sdf version="1.7">
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


# ----------------------------
# Main launch logic
# ----------------------------

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

    # ---- Iterate AOIs ----
    for aoi_name, aoi in cfg.get("aoi", {}).items():
        entity_name = aoi["entity_name"]

        # Delete existing entity
        delete_entity(entity_name)

        # AOI local offset and orientation
        offset = aoi["offset"]
        offset_local = (offset["x"], offset["y"], offset["z"])

        rpy = aoi["orientation_rpy"]
        aoi_q_local = quaternion_from_rpy(
            rpy["roll"], rpy["pitch"], rpy["yaw"]
        )

        # Transform offset into world frame
        offset_world = rotate_vector_by_quaternion(offset_local, train_q)
        aoi_pos_world = (
            train_pos[0] + offset_world[0],
            train_pos[1] + offset_world[1],
            train_pos[2] + offset_world[2],
        )

        # Combine orientations
        aoi_q_world = multiply_quaternions(train_q, aoi_q_local)

        # Generate SDF
        size = aoi["size"]
        color = aoi.get("visual", {}).get("color", [0.2, 0.8, 0.2, 0.4])

        sdf = generate_plane_sdf(
            entity_name,
            size["width"],
            size["depth"],
            color
        )

        # Spawn entity
        spawn_plane_entity(
            entity_name,
            sdf,
            aoi_pos_world,
            aoi_q_world
        )

    return []


# ----------------------------
# Launch description
# ----------------------------

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="AOI spawn launch: deleting and respawning AOIs from YAML"),
        OpaqueFunction(function=spawn_aois),
    ])
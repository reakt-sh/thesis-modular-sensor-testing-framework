#!/usr/bin/env python3
import os
import yaml
import math
from pathlib import Path
import xml.etree.ElementTree as ET

PKG_ROOT = Path(__file__).resolve().parent.parent
AOI_YAML_PATH = PKG_ROOT / "config" / "aois.yaml"
OUTPUT_DIR = PKG_ROOT / "models" / "generated_aois"


# --------------------------------------------
# Helpers
# --------------------------------------------

def rotate_point(px, py, yaw):
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    x = cos_y * px - sin_y * py
    y = sin_y * px + cos_y * py
    return x, y


def make_xml_sdf_model(name, width, height, pose_xyzrpy, colors):
    """
    Create a 2-sided AOI SDF model using ElementTree.
    colors = { "front": [r,g,b,a], "back": [r,g,b,a] }
    """
    x, y, z, R, P, Y = pose_xyzrpy

    sdf = ET.Element("sdf", {"version": "1.7"})
    model = ET.SubElement(sdf, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"

    link = ET.SubElement(model, "link", {"name": "aoi_plane"})
    ET.SubElement(link, "pose").text = f"{x} {y} {z} {R} {P} {Y}"

    # --------------------------
    # FRONT FACE
    # --------------------------
    visual_f = ET.SubElement(link, "visual", {"name": "front_face"})
    geom_f = ET.SubElement(visual_f, "geometry")
    box_f = ET.SubElement(geom_f, "box")
    ET.SubElement(box_f, "size").text = f"{width} 0.01 {height}"

    mat_f = ET.SubElement(visual_f, "material")
    f_r, f_g, f_b, f_a = colors["front"]
    ET.SubElement(mat_f, "ambient").text = f"{f_r} {f_g} {f_b} {f_a}"
    ET.SubElement(mat_f, "diffuse").text = f"{f_r} {f_g} {f_b} {f_a}"

    # --------------------------
    # BACK FACE (slightly offset)
    # --------------------------
    visual_b = ET.SubElement(link, "visual", {"name": "back_face"})

    # slight offset to avoid z-fighting
    ET.SubElement(visual_b, "pose").text = "0 -0.02 0 0 0 0"

    geom_b = ET.SubElement(visual_b, "geometry")
    box_b = ET.SubElement(geom_b, "box")
    ET.SubElement(box_b, "size").text = f"{width} 0.01 {height}"

    mat_b = ET.SubElement(visual_b, "material")
    b_r, b_g, b_b, b_a = colors["back"]
    ET.SubElement(mat_b, "ambient").text = f"{b_r} {b_g} {b_b} {b_a}"
    ET.SubElement(mat_b, "diffuse").text = f"{b_r} {b_g} {b_b} {b_a}"

    return ET.tostring(sdf, encoding="unicode")

def make_xml_model_config(name, description):
    root = ET.Element("model")
    ET.SubElement(root, "name").text = name
    ET.SubElement(root, "version").text = "1.0"
    sdf_tag = ET.SubElement(root, "sdf", {"version": "1.7"})
    sdf_tag.text = "model.sdf"
    ET.SubElement(root, "description").text = description
    return ET.tostring(root, encoding="unicode")


# --------------------------------------------
# MAIN
# --------------------------------------------

def main():
    print("Loading:", AOI_YAML_PATH)
    with open(AOI_YAML_PATH) as f:
        data = yaml.safe_load(f)

    train = data["train_spawn"]
    aois = data["aoi"]

    train_x = float(train["x"])
    train_y = float(train["y"])
    train_z = float(train["z"])
    train_yaw = float(train["yaw"])

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print("Generating AOI SDF models...")

    for name, cfg in aois.items():

        cx, cy, cz = cfg["center"]
        width, height = cfg["size"]
        ar, ap, ay = cfg["orientation_rpy"]
        desc = cfg.get("description", "")

        # Rotate AOI center around train yaw
        rot_x, rot_y = rotate_point(cx, cy, train_yaw)

        world_x = train_x + rot_x
        world_y = train_y + rot_y
        world_z = train_z + cz

        # Global orientation (train yaw + AOI yaw)
        R = ar
        P = ap
        Y = train_yaw + ay

        pose = (world_x, world_y, world_z, R, P, Y)

        colors = {
            "front": cfg["colors"]["front"],
            "back":  cfg["colors"]["back"]
        }

        sdf_xml = make_xml_sdf_model(
            name=name,
            width=width,
            height=height,
            pose_xyzrpy=pose,
            colors=colors
        )

        config_xml = make_xml_model_config(name, desc)

        model_dir = OUTPUT_DIR / name
        model_dir.mkdir(exist_ok=True)

        with open(model_dir / "model.sdf", "w") as f:
            f.write(sdf_xml)

        with open(model_dir / "model.config", "w") as f:
            f.write(config_xml)

        print(f" ✔ AOI model created: {name}")

    print("\nAll AOI models generated.")
    print("Output directory:", OUTPUT_DIR)


if __name__ == "__main__":
    main()
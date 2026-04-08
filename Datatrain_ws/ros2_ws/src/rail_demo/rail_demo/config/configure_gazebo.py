#!/usr/bin/env python3

import os
import yaml
import logging
from textual.app import App, ComposeResult
from textual.widgets import Header, Static, Select, Button, Input
from textual.containers import Vertical, Horizontal
from ament_index_python.packages import get_package_share_directory


class GazeboConfigApp(App):
    CSS_PATH = None
    TITLE = "Gazebo Configuration Selector"
    SUB_TITLE = "Modular Sensor Testing Framework"

    def __init__(self):
        super().__init__()

        package_root = os.path.dirname(os.path.dirname(__file__))
        self.config_path = os.path.join(
            package_root, "config", "simulation_config.yaml"
        )

        pkg_share = get_package_share_directory("rail_demo")
        self.worlds_path = os.path.join(pkg_share, "worlds")
        self.models_path = os.path.join(pkg_share, "models", "base_models")

        self.world_files = sorted(
            [f for f in os.listdir(self.worlds_path) if f.endswith(".sdf")]
        ) if os.path.isdir(self.worlds_path) else []

        self.model_dirs = sorted(
            [
                d for d in os.listdir(self.models_path)
                if os.path.isdir(os.path.join(self.models_path, d))
            ]
        ) if os.path.isdir(self.models_path) else []

        self.default_entity_name = "gazebo_train"
        self.default_spawn_pose = "(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)"
        self.default_sensor_mount_pose = "(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)"

        self.load_defaults_from_yaml()

    def load_defaults_from_yaml(self):
        if not os.path.exists(self.config_path):
            return

        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}

            simulation = data.get("simulation", {})
            train = data.get("train", {})
            sensor_mount = data.get("sensor_mount", {})

            self.default_entity_name = simulation.get(
                "train_entity_name", self.default_entity_name
            )

            spawn = train.get("spawn", {})
            self.default_spawn_pose = (
                f"({spawn.get('x', 0.0)}, {spawn.get('y', 0.0)}, {spawn.get('z', 0.0)}, "
                f"{spawn.get('roll', 0.0)}, {spawn.get('pitch', 0.0)}, {spawn.get('yaw', 0.0)})"
            )

            mount_pose = sensor_mount.get("pose", {})
            self.default_sensor_mount_pose = (
                f"({mount_pose.get('x', 0.0)}, {mount_pose.get('y', 0.0)}, {mount_pose.get('z', 0.0)}, "
                f"{mount_pose.get('roll', 0.0)}, {mount_pose.get('pitch', 0.0)}, {mount_pose.get('yaw', 0.0)})"
            )

        except Exception as e:
            logging.error(f"Error reading config file: {e}")

    def parse_spawn_pose(self, pose_str: str):
        """
        Expected format:
            (x, y, z, roll, pitch, yaw)
        Returns tuple of 6 floats or raises ValueError.
        """
        if not pose_str:
            raise ValueError("Pose is empty")

        cleaned = pose_str.strip()

        if cleaned.startswith("(") and cleaned.endswith(")"):
            cleaned = cleaned[1:-1]

        parts = [p.strip() for p in cleaned.split(",")]

        if len(parts) != 6:
            raise ValueError(
                "Pose must contain exactly 6 values: "
                "(x, y, z, roll, pitch, yaw)"
            )

        try:
            values = tuple(float(p) for p in parts)
        except ValueError:
            raise ValueError("Pose must contain only numeric values")

        return values

    def save_yaml(
        self,
        world: str,
        model: str,
        entity_name: str,
        spawn_pose: str,
        sensor_mount_pose: str,
    ):
        x, y, z, roll, pitch, yaw = self.parse_spawn_pose(spawn_pose)
        sm_x, sm_y, sm_z, sm_roll, sm_pitch, sm_yaw = self.parse_spawn_pose(sensor_mount_pose)

        data = {
            "simulation": {
                "world": world,
                "train_model": model,
                "train_entity_name": entity_name,
            },
            "train": {
                "spawn": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw,
                },
            },
            "sensor_mount": {
                "pose": {
                    "x": sm_x,
                    "y": sm_y,
                    "z": sm_z,
                    "roll": sm_roll,
                    "pitch": sm_pitch,
                    "yaw": sm_yaw,
                },
            },
        }

        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)

        with open(self.config_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)

    def compose(self) -> ComposeResult:
        yield Header()

        world_options = [(w, w) for w in self.world_files]
        model_options = [(m, m) for m in self.model_dirs]

        yield Vertical(
            Static("Select World File:", classes="label"),
            Select(
                id="world_select",
                options=world_options,
                prompt="Choose a world",
                allow_blank=True,
            ),

            Static("Select Train Model Folder:", classes="label"),
            Select(
                id="model_select",
                options=model_options,
                prompt="Choose a train model",
                allow_blank=True,
            ),

            Static("Train Entity Name:", classes="label"),
            Input(
                value=self.default_entity_name,
                placeholder="gazebo_train",
                id="entity_name_input",
            ),

            Static("Train Spawn Pose (x, y, z, roll, pitch, yaw):", classes="label"),
            Input(
                value=self.default_spawn_pose,
                placeholder="(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)",
                id="spawn_pose_input",
            ),

            Static(
                "Sensor Mount Injection Pose relative to the model's base_link (x, y, z, roll, pitch, yaw):",
                classes="label",
            ),
            Input(
                value=self.default_sensor_mount_pose,
                placeholder="(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)",
                id="sensor_mount_pose_input",
            ),

            Static(
                f"World files found: {len(self.world_files)} | Model folders found: {len(self.model_dirs)}",
                classes="label",
            ),

            Horizontal(
                Button(label="Save Configuration", id="save_button"),
                Button(label="Close", id="close_button", classes="danger"),
            ),
        )

    def on_button_pressed(self, event: Button.Pressed) -> None:
        button_id = event.button.id

        if button_id == "save_button":
            world = self.query_one("#world_select", Select).value
            model = self.query_one("#model_select", Select).value
            entity_name = self.query_one("#entity_name_input", Input).value.strip()
            spawn_pose = self.query_one("#spawn_pose_input", Input).value.strip()
            sensor_mount_pose = self.query_one("#sensor_mount_pose_input", Input).value.strip()

            if not world or not model:
                self.notify(
                    "Please explicitly select both a world and a train model.",
                    severity="error",
                )
                return

            if not entity_name:
                self.notify("Train entity name cannot be empty.", severity="error")
                return

            try:
                self.save_yaml(world, model, entity_name, spawn_pose, sensor_mount_pose)
                self.notify(
                    f"Saved configuration to {self.config_path}",
                    severity="information",
                )
            except ValueError as e:
                self.notify(str(e), severity="error")
            except Exception as e:
                self.notify(f"Failed to save config: {e}", severity="error")

        elif button_id == "close_button":
            self.exit()


if __name__ == "__main__":
    GazeboConfigApp().run()
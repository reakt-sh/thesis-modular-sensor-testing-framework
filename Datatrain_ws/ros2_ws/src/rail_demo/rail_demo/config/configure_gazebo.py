#!/usr/bin/env python3

import os
import yaml
import logging
from textual.app import App, ComposeResult
from textual.widgets import Header, Footer, Static, Select, Button
from textual.containers import Vertical, Horizontal
from ament_index_python.packages import get_package_share_directory

# ------------------------------------------
# Logging configuration – writes to this file because debugging 
# ------------------------------------------
# logging.basicConfig(
#     filename="/tmp/textual.log",
#     level=logging.DEBUG,
#     format="%(asctime)s [%(levelname)s] %(message)s"
# )

# in container tail -f /tmp/textual.log


class GazeboConfigApp(App):
    CSS_PATH = None
    TITLE = "Gazebo Configuration Selector"
    SUB_TITLE = "Modular Sensor Testing Framework"

    def __init__(self):
        super().__init__()

        #Path to config in src not in install to be able to see local changes easier
        package_root = os.path.dirname(os.path.dirname(__file__))
        self.config_path = os.path.join(package_root, "config", "simulation.yaml")
        logging.info(f"Using SOURCE config path: {self.config_path}")

        
        # Installed package share directory for models/worlds in installed folder (not src)
       
        pkg_share = get_package_share_directory("rail_demo")
        self.worlds_path = os.path.join(pkg_share, "worlds")
        self.models_path = os.path.join(pkg_share, "models")
        logging.info(f"Using INSTALLED pkg_share: {pkg_share}")

        self.world_files = [
            f for f in os.listdir(self.worlds_path)
            if f.endswith(".sdf")
        ]
        logging.info(f"Found worlds: {self.world_files}")

        self.model_dirs = [
            d for d in os.listdir(self.models_path)
            if os.path.isdir(os.path.join(self.models_path, d))
        ]
        logging.info(f"Found models: {self.model_dirs}")

         # Load defaults
        self.default_world = None
        self.default_model = None

        if os.path.exists(self.config_path):
            try:
                with open(self.config_path) as f:
                    sim = yaml.safe_load(f) or {}

                self.default_world = sim.get("world")
                self.default_model = sim.get("model")

                logging.info(f"Loaded defaults: {sim}")



            except Exception as e:
                logging.error(f"Error reading YAML: {e}")

    #function to overwrite yaml
    def save_yaml(self, world: str, model: str):
        data = {"world": world, "model": model}

        # Ensure directory exists
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)

        try:
            with open(self.config_path, "w") as f:
                yaml.dump(data, f)

            logging.info(f"YAML saved to {self.config_path}: {data}")

        except Exception as e:
            logging.error(f"Failed writing YAML: {e}")

    #create UI to pick world model exit and save
    def compose(self) -> ComposeResult:
        yield Header()

        yield Vertical(
            Static("Select World File:", classes="label"),
            Select(
                id="world_select",
                options=[(w, w) for w in self.world_files],
                value=self.default_world
            ),

            Static("Select Train Model Folder:", classes="label"),
            Select(
                id="model_select",
                options=[(m, m) for m in self.model_dirs],
                value=self.default_model
            ),
            Horizontal(
                Button(label="Save Configuration", id="save_button"),
                Button(label="X", id="close_button", classes="danger"),
            ),

            
        )


    # Buttons to close app and to save picked world and train
    def on_button_pressed(self, event: Button.Pressed) -> None:
        button_id = event.button.id
        logging.info(f"Button pressed: {button_id}")

        if button_id == "save_button":
            world = self.query_one("#world_select").value
            model = self.query_one("#model_select").value

            logging.info(f"Selections: world={world}, model={model}")

            if not world or not model:
                logging.error("Missing world or model selection")
                return

            self.save_yaml(world, model)

        elif button_id == "close_button":
            logging.info("Closing Textual app...")
            self.exit()


if __name__ == "__main__":
    GazeboConfigApp().run()
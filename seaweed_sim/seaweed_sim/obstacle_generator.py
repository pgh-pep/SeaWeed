#!/usr/bin/env python3
# type: ignore

import yaml
import argparse
import os
from typing import List, Dict, Any
import xml.etree.ElementTree as ET

# NOTE: Specifically made for the OpenRobotics buoys, will need refactoring to work with other models, both in fuel and custom


class ObstacleGenerator:
    def __init__(self):
        self.model_uris = {
            "mb_round_buoy": "https://fuel.gazebosim.org/1.0/openrobotics/models/mb_round_buoy_{color}",
            "mb_marker_buoy": "https://fuel.gazebosim.org/1.0/openrobotics/models/mb_marker_buoy_{color}",
            # "polyform_a3_buoy": "https://fuel.gazebosim.org/1.0/openrobotics/models/polyform_a3_{color}",
        }
        self.buoy_configs = {
            "mb_marker_buoy": {
                "linear_drag": "25.0",
                "angular_drag": "2.0",
                "buoyancy_pose": "0 0 -0.3 0 0 0",
                "geometry_type": "cylinder",
                "radius": 0.35,
            },
            "mb_round_buoy": {
                "linear_drag": "75.0",
                "angular_drag": "2.0",
                "buoyancy_pose": "0 0 0 0 0 0",
                "geometry_type": "sphere",
                "radius": 0.25,
            },
        }

    def read_yaml(self, filepath: str) -> List[Dict[str, Any]]:
        with open(filepath, "r") as file:
            data: Dict[str, Any] = yaml.safe_load(file)
        return data.get("obstacles", [])

    def read_obstacles(self, filepath: str) -> List[Dict[str, Any]]:
        ext = os.path.splitext(filepath)[1].lower()
        if ext == ".yaml":
            return self.read_yaml(filepath)
        else:
            raise ValueError(f"Unsupported file format: {ext}")

    def create_geometry_element(
        self, geometry_parent: ET.Element, config: Dict[str, Any], obstacle: Dict[str, Any]
    ) -> None:
        geometry_type = config["geometry_type"]

        if geometry_type == "cylinder":
            self.create_cylinder_geo(geometry_parent, config, obstacle)
        elif geometry_type == "sphere":
            self.create_sphere_geo(geometry_parent, config, obstacle)
        else:
            raise ValueError(f"Unsupported geo type: {geometry_type}")

    def create_cylinder_geo(
        self, geometry_parent: ET.Element, config: Dict[str, Any], obstacle: Dict[str, Any]
    ) -> None:
        cylinder = ET.SubElement(geometry_parent, "cylinder")

        radius = obstacle.get("radius", config["radius"])
        radius_elem = ET.SubElement(cylinder, "radius")
        radius_elem.text = str(radius)

        length = radius * 0.325  # constant used from existing vrx worlds
        length_elem = ET.SubElement(cylinder, "length")
        length_elem.text = str(length)

    def create_sphere_geo(self, geometry_parent: ET.Element, config: Dict[str, Any], obstacle: Dict[str, Any]) -> None:
        """Create sphere geometry element."""
        sphere = ET.SubElement(geometry_parent, "sphere")

        radius = obstacle.get("radius", config["radius"])
        radius_elem = ET.SubElement(sphere, "radius")
        radius_elem.text = str(radius)

    def validate_obstacle(self, model_type, color) -> bool:
        valid_colors = {
            "mb_round_buoy": ["black", "orange"],
            "mb_marker_buoy": ["red", "black", "green", "white"],
        }

        if model_type not in self.model_uris:
            print(f"Unknown model type: {model_type}")
            raise Exception(f"Model {model_type} does not exist")

        if color not in valid_colors[model_type]:
            print(valid_colors)
            print(f"Model {model_type} does not exist in color {color}")
            raise Exception(f"Model {model_type} does not exist in color {color}")

    def create_obstacle(self, obstacle: Dict[str, Any], parent: ET.Element) -> ET.Element:
        model_type = obstacle["type"]
        color = obstacle["color"]

        self.validate_obstacle(model_type, color)

        pose = " ".join(map(str, obstacle["pose"]))

        include = ET.SubElement(parent, "include")

        name = ET.SubElement(include, "name")
        name.text = obstacle.get("name", f"{model_type}_{color}")

        # Pose
        pose_elem = ET.SubElement(include, "pose")
        pose_elem.text = pose

        # Model URI
        uri_elem = ET.SubElement(include, "uri")
        uri = self.model_uris[model_type].format(color=color)
        uri_elem.text = uri

        # Physics plugins
        plugin = ET.SubElement(include, "plugin")
        plugin.set("name", "vrx::PolyhedraBuoyancyDrag")
        plugin.set("filename", "libPolyhedraBuoyancyDrag.so")

        fluid_density = ET.SubElement(plugin, "fluid_density")
        fluid_density.text = "1000"

        fluid_level = ET.SubElement(plugin, "fluid_level")
        fluid_level.text = "0.0"

        config = self.buoy_configs.get(model_type, self.buoy_configs["mb_round_buoy"])

        # Drag
        linear_drag = ET.SubElement(plugin, "linear_drag")
        linear_drag.text = config["linear_drag"]
        angular_drag = ET.SubElement(plugin, "angular_drag")
        angular_drag.text = config["angular_drag"]

        # Buoyancy
        buoyancy = ET.SubElement(plugin, "buoyancy")
        buoyancy.set("name", "collision_outer")
        link_name = ET.SubElement(buoyancy, "link_name")
        link_name.text = "link"
        buoyancy_pose = ET.SubElement(buoyancy, "pose")
        buoyancy_pose.text = config["buoyancy_pose"]

        # Geometry (radius, length)
        geometry = ET.SubElement(buoyancy, "geometry")
        self.create_geometry_element(geometry, config, obstacle)

        wavefield = ET.SubElement(plugin, "wavefield")
        topic = ET.SubElement(wavefield, "topic")
        topic.text = "/vrx/wavefield/parameters"

        return include

    def add_obstacles(self, sdf_file: str, obstacles_file: str, output_file: str):
        try:
            obstacles = self.read_obstacles(obstacles_file)
            print(f"Read {len(obstacles)} obstacles from {obstacles_file}")

            tree = ET.parse(sdf_file)
            root = tree.getroot()

            world = root.find("world")

            world_name = os.path.splitext(os.path.basename(output_file))[0]
            world.set("name", world_name)
            print(f"Set world name to: {world_name}")

            for obstacle in obstacles:
                print(
                    f"Adding obstacle: {obstacle['name']} ({obstacle['color']} {obstacle['type']}, r = {obstacle.get('radius', 'default')})"
                )
                self.create_obstacle(obstacle, world)

            self.write_to_sdf(tree, output_file)

        except Exception as e:
            print(f"Error adding obstacles: {e}")
            raise e

    def write_to_sdf(self, tree: ET.ElementTree, output_file: str):
        ET.indent(tree, space="\t", level=0)

        with open(output_file, "w", encoding="utf-8") as f:
            f.write('<?xml version="1.0"?>\n')
            tree.write(f, encoding="unicode", xml_declaration=False)

        print(f"Updated SDF written to: {output_file}")


def main():
    parser = argparse.ArgumentParser(description="Add obstacles to SDF world file")
    parser.add_argument("sdf_file", help="Input SDF file path")
    parser.add_argument("obstacles_yaml", help="Obstacles definition file (YAML)")
    parser.add_argument("-o", "--output", help="Output SDF file path")

    args = parser.parse_args()

    obstacle_generator = ObstacleGenerator()
    obstacle_generator.add_obstacles(args.sdf_file, args.obstacles_yaml, args.output)


if __name__ == "__main__":
    main()


# round: black and orange
# markers: red, black, green, white

# red marker?

#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "table_collision.yaml"
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "ur3_table_collision.launch"
PACKAGE_XML = PACKAGE_ROOT / "package.xml"


class TableCollisionLaunchTest(unittest.TestCase):
    def test_package_declares_table_collision_runtime_dependencies(self):
        root = ET.fromstring(PACKAGE_XML.read_text(encoding="utf-8"))
        dependencies = {
            item.text
            for tag in ("depend", "exec_depend")
            for item in root.findall(tag)
        }
        self.assertTrue(
            {
                "anygrasp_ros",
                "diagnostic_msgs",
                "geometry_msgs",
                "moveit_commander",
                "moveit_msgs",
                "python3-numpy",
                "tf",
            }.issubset(dependencies)
        )

    def test_config_has_safe_bounded_defaults(self):
        self.assertTrue(CONFIG_PATH.is_file(), "table collision config is missing")
        config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))

        self.assertFalse(config["table_collision_enabled"])
        self.assertEqual(config["table_object_id"], "table_surface")
        self.assertEqual(config["table_collision_thickness"], 0.05)
        self.assertEqual(config["table_xy_margin"], 0.025)
        self.assertEqual(config["stable_plane_frames"], 5)
        self.assertEqual(config["max_height_variation"], 0.009)
        self.assertEqual(config["max_normal_angle_deg"], 4.5)
        self.assertEqual(config["update_height_threshold"], 0.005)
        self.assertEqual(config["update_angle_threshold_deg"], 2.0)
        self.assertEqual(config["max_table_plane_age"], 1.0)
        self.assertEqual(config["status_publish_period"], 0.5)
        self.assertEqual(
            config["stable_table_pose_topic"], "/table_collision/stable_surface_pose"
        )
        self.assertEqual(config["table_status_topic"], "/table_collision/status")

    def test_launch_is_disabled_by_default_and_reuses_anygrasp_roi_file(self):
        self.assertTrue(LAUNCH_PATH.is_file(), "table collision launch is missing")
        root = ET.fromstring(LAUNCH_PATH.read_text(encoding="utf-8"))
        arguments = {item.attrib["name"]: item for item in root.findall("arg")}

        self.assertEqual(arguments["table_collision_enabled"].attrib["default"], "false")
        self.assertIn("anygrasp_ros", arguments["geometry_config_file"].attrib["default"])
        self.assertIn("anygrasp_d405.yaml", arguments["geometry_config_file"].attrib["default"])
        node = root.find("node")
        self.assertEqual(node.attrib["pkg"], "tracer_bringup")
        self.assertEqual(node.attrib["type"], "table_collision_updater.py")
        self.assertEqual(node.attrib["if"], "$(arg table_collision_enabled)")
        loaded_files = [item.attrib["file"] for item in node.findall("rosparam")]
        self.assertEqual(
            loaded_files,
            ["$(arg geometry_config_file)", "$(arg config_file)"],
        )


if __name__ == "__main__":
    unittest.main()

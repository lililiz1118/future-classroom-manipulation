import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "yolo_target_cloud.yaml"
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "yolo_target_cloud.launch"


class YoloTargetCloudConfigurationTest(unittest.TestCase):
    def test_config_has_timestamp_safe_detection_driven_defaults(self):
        config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))

        self.assertEqual(config["cloud_topic"], "/d405/depth/color/points")
        self.assertEqual(config["camera_info_topic"], "/d405/color/camera_info")
        self.assertEqual(
            config["detection_topic"], "/yolo_world/target_detection"
        )
        self.assertEqual(
            config["target_cloud_topic"], "/yolo_world/target_cloud"
        )
        self.assertIn("table_surface_pose_topic", config)
        self.assertEqual(
            config["table_surface_pose_topic"], "/yolo_world/table_surface_pose"
        )
        self.assertEqual(config["color_frame"], "d405_color_optical_frame")
        self.assertEqual(config["cloud_cache_duration_sec"], 1.0)
        self.assertEqual(config["max_stamp_delta_sec"], 0.02)
        self.assertEqual(config["max_detection_age_sec"], 0.5)
        self.assertEqual(config["stale_check_period_sec"], 0.1)
        self.assertEqual(config["table_preprocess_rate_hz"], 5.0)
        self.assertTrue(config["require_ransac_success"])

    def test_launch_reuses_anygrasp_geometry_config_without_control_nodes(self):
        root = ET.fromstring(LAUNCH_PATH.read_text(encoding="utf-8"))

        self.assertEqual(root.findall("include"), [])
        nodes = root.findall("node")
        self.assertEqual(len(nodes), 1)
        node = nodes[0]
        self.assertEqual(node.attrib["pkg"], "yolo_world_ros")
        self.assertEqual(node.attrib["type"], "yolo_target_cloud_node.py")
        self.assertEqual(node.attrib["name"], "yolo_target_cloud_node")
        self.assertIn("python_executable", node.attrib["launch-prefix"])
        arguments = {item.attrib["name"]: item for item in root.findall("arg")}
        self.assertIn(
            "anygrasp_ros", arguments["geometry_config_file"].attrib["default"]
        )
        self.assertIn(
            "anygrasp_d405.yaml",
            arguments["geometry_config_file"].attrib["default"],
        )
        self.assertIn(
            "yolo_target_cloud.yaml", arguments["config_file"].attrib["default"]
        )
        rosparams = node.findall("rosparam")
        self.assertEqual(len(rosparams), 2)
        self.assertEqual(rosparams[0].attrib["file"], "$(arg geometry_config_file)")
        self.assertEqual(rosparams[1].attrib["file"], "$(arg config_file)")


if __name__ == "__main__":
    unittest.main()

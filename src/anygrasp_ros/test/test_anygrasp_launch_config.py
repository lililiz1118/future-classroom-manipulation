import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "anygrasp_d405.yaml"
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "anygrasp_d405.launch"
RESOURCE_CONFIG_PATH = PACKAGE_ROOT / "config" / "anygrasp_resources.yaml"


class AnyGraspConfigurationTest(unittest.TestCase):
    def test_config_matches_verified_d405_and_sdk_settings(self):
        self.assertTrue(CONFIG_PATH.is_file(), f"missing config: {CONFIG_PATH}")
        config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))

        self.assertEqual(config["cloud_topic"], "/d405/depth/color/points")
        self.assertEqual(config["best_grasp_topic"], "/anygrasp/best_grasp")
        self.assertEqual(
            config["best_grasp_base_topic"], "/anygrasp/best_grasp_base"
        )
        self.assertEqual(
            config["workspace_cloud_topic"], "/anygrasp/workspace_cloud"
        )
        self.assertEqual(config["sdk_dir"], "/home/jt001/anygrasp_sdk/grasp_detection")
        self.assertEqual(
            config["checkpoint_path"],
            "/home/jt001/anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar",
        )
        self.assertEqual(config["top_n"], 10)
        self.assertEqual(config["inference_rate"], 0.2)
        self.assertTrue(config["publish_input_cloud"])
        self.assertTrue(config["collision_detection"])
        self.assertTrue(config["top_down_grasp"])
        self.assertFalse(config["dense_grasp"])
        self.assertEqual(config["voxel_size"], 0.005)
        self.assertEqual(config["min_workspace_points"], 1000)
        self.assertEqual(config["dynamic_lims_margin"], 0.01)
        self.assertEqual(config["tf_timeout"], 0.2)
        self.assertEqual(
            config["statistical_outlier_filter"],
            {
                "enabled": True,
                "nb_neighbors": 20,
                "std_ratio": 2.0,
            },
        )
        self.assertEqual(
            config["workspace"],
            {
                "frame_id": "ur_arm_base_link",
                "x_min": 0.0,
                "x_max": 0.0,
                "y_min": 0.0,
                "y_max": 0.0,
                "z_min": 0.0,
                "z_max": 0.0,
            },
        )

    def test_launch_uses_conda_python_and_includes_no_control_launch(self):
        self.assertTrue(LAUNCH_PATH.is_file(), f"missing launch: {LAUNCH_PATH}")
        root = ET.fromstring(LAUNCH_PATH.read_text(encoding="utf-8"))

        self.assertEqual(root.findall("include"), [])
        nodes = root.findall("node")
        self.assertEqual(len(nodes), 1)
        node = nodes[0]
        self.assertEqual(node.attrib["pkg"], "anygrasp_ros")
        self.assertEqual(node.attrib["type"], "anygrasp_d405_node.py")
        self.assertEqual(node.attrib["name"], "anygrasp_d405_node")
        self.assertIn("python_executable", node.attrib["launch-prefix"])
        rosparams = node.findall("rosparam")
        self.assertEqual(len(rosparams), 1)
        self.assertEqual(rosparams[0].attrib["command"], "load")
        resource_argument = next(
            item for item in root.findall("arg") if item.attrib["name"] == "resource_config_file"
        )
        self.assertIn("anygrasp_resources.yaml", resource_argument.attrib["default"])
        resource_environment = node.find("env")
        self.assertEqual(resource_environment.attrib["name"], "ANYGRASP_RESOURCE_CONFIG")
        self.assertEqual(resource_environment.attrib["value"], "$(arg resource_config_file)")


if __name__ == "__main__":
    unittest.main()

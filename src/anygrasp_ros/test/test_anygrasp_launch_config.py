import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "anygrasp_d405.yaml"
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "anygrasp_d405.launch"


class AnyGraspConfigurationTest(unittest.TestCase):
    def test_config_matches_verified_d405_and_sdk_defaults(self):
        self.assertTrue(CONFIG_PATH.is_file(), f"missing config: {CONFIG_PATH}")
        config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))

        self.assertEqual(config["cloud_topic"], "/d405/depth/color/points")
        self.assertEqual(config["sdk_dir"], "/home/jt001/anygrasp_sdk/grasp_detection")
        self.assertEqual(
            config["checkpoint_path"],
            "/home/jt001/anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar",
        )
        self.assertEqual(config["top_n"], 10)
        self.assertEqual(config["inference_rate"], 1.0)
        self.assertTrue(config["collision_detection"])
        self.assertTrue(config["top_down_grasp"])
        self.assertFalse(config["dense_grasp"])
        self.assertEqual(config["voxel_size"], 0.005)
        self.assertEqual(
            config["workspace"],
            {
                "x_min": -0.5,
                "x_max": 0.5,
                "y_min": -0.5,
                "y_max": 0.5,
                "z_min": 0.1,
                "z_max": 1.5,
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


if __name__ == "__main__":
    unittest.main()

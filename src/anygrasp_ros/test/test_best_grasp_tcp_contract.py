import ast
import os
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "scripts" / "best_grasp_tcp_node.py"
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "best_grasp_tcp.launch"


class BestGraspTcpContractTest(unittest.TestCase):
    def test_node_has_no_motion_or_robot_control_imports(self):
        tree = ast.parse(NODE_PATH.read_text(encoding="utf-8"))
        imported = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                imported.append(node.module or "")

        forbidden = (
            "moveit",
            "trajectory",
            "controller_manager",
            "ur_dashboard",
            "dh_gripper",
            "actionlib",
        )
        violations = [
            name for name in imported if any(token in name.lower() for token in forbidden)
        ]
        self.assertEqual(violations, [], f"motion/control imports found: {violations}")

    def test_launch_starts_only_the_debug_pose_converter(self):
        self.assertTrue(LAUNCH_PATH.is_file(), f"missing launch: {LAUNCH_PATH}")
        root = ET.fromstring(LAUNCH_PATH.read_text(encoding="utf-8"))

        self.assertEqual(root.findall("include"), [])
        nodes = root.findall("node")
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].attrib["pkg"], "anygrasp_ros")
        self.assertEqual(nodes[0].attrib["type"], "best_grasp_tcp_node.py")
        self.assertEqual(nodes[0].attrib["name"], "best_grasp_tcp_node")
        self.assertEqual(
            {
                param.attrib["name"]: param.attrib["value"]
                for param in nodes[0].findall("param")
            },
            {
                "input_topic": "$(arg input_topic)",
                "output_topic": "$(arg output_topic)",
                "marker_topic": "$(arg marker_topic)",
                "base_frame": "$(arg base_frame)",
                "pregrasp_topic": "$(arg pregrasp_topic)",
                "pregrasp_marker_topic": "$(arg pregrasp_marker_topic)",
                "pregrasp_distance": "$(arg pregrasp_distance)",
            },
        )

    def test_node_imports_in_a_clean_ros_python_process_without_test_path_setup(self):
        environment = os.environ.copy()
        ros_python = "/opt/ros/noetic/lib/python3/dist-packages"
        existing = environment.get("PYTHONPATH", "")
        environment["PYTHONPATH"] = ros_python + (
            os.pathsep + existing if existing else ""
        )
        probe = (
            "import runpy; "
            f"runpy.run_path({str(NODE_PATH)!r}, run_name='best_grasp_tcp_import_probe')"
        )

        result = subprocess.run(
            [sys.executable, "-c", probe],
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()

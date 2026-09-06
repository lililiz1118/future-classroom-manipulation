import ast
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "scripts" / "tcp_to_wrist_goal_node.py"
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "tcp_to_wrist_goal.launch"


class TcpToWristGoalContractTest(unittest.TestCase):
    def test_node_has_no_motion_or_execution_imports(self):
        tree = ast.parse(NODE_PATH.read_text(encoding="utf-8"))
        imported = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                imported.append(node.module or "")
        forbidden = ("moveit", "trajectory", "controller_manager", "actionlib", "dh_gripper")
        self.assertEqual(
            [name for name in imported if any(word in name.lower() for word in forbidden)],
            [],
        )

    def test_launch_starts_only_the_tf_geometry_bridge(self):
        root = ET.fromstring(LAUNCH_PATH.read_text(encoding="utf-8"))
        self.assertEqual(root.findall("include"), [])
        nodes = root.findall("node")
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].attrib["pkg"], "anygrasp_ros")
        self.assertEqual(nodes[0].attrib["type"], "tcp_to_wrist_goal_node.py")


if __name__ == "__main__":
    unittest.main()

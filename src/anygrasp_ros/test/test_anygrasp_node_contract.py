import ast
import os
import subprocess
import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "scripts" / "anygrasp_d405_node.py"


class AnyGraspNodeContractTest(unittest.TestCase):
    def test_resource_setup_precedes_numpy_and_torch_setup_precedes_gsnet(self):
        tree = ast.parse(NODE_PATH.read_text(encoding="utf-8"))
        numpy_import = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
            and any(alias.name == "numpy" for alias in node.names)
        )
        process_setup = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "initialize_resource_policy"
        )
        gsnet_import = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module == "gsnet"
        )
        torch_setup = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "configure_torch"
        )

        self.assertLess(process_setup.lineno, numpy_import.lineno)
        self.assertLess(torch_setup.lineno, gsnet_import.lineno)

    def test_node_declares_latest_message_buffer(self):
        self.assertTrue(NODE_PATH.is_file(), f"missing node: {NODE_PATH}")
        tree = ast.parse(NODE_PATH.read_text(encoding="utf-8"))
        classes = {
            node.name: node
            for node in tree.body
            if isinstance(node, ast.ClassDef)
        }
        self.assertIn("LatestMessageBuffer", classes)
        methods = {
            node.name
            for node in classes["LatestMessageBuffer"].body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        self.assertTrue({"update", "take_latest"}.issubset(methods))

    def test_node_has_no_motion_control_imports(self):
        self.assertTrue(NODE_PATH.is_file(), f"missing node: {NODE_PATH}")
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
        violations = [name for name in imported if any(token in name.lower() for token in forbidden)]
        self.assertEqual(violations, [], f"motion/control imports found: {violations}")

    def test_system_python_path_is_removed_after_ros_imports(self):
        self.assertTrue(NODE_PATH.is_file(), f"missing node: {NODE_PATH}")
        environment = os.environ.copy()
        ros_python = "/opt/ros/noetic/lib/python3/dist-packages"
        existing = environment.get("PYTHONPATH", "")
        environment["PYTHONPATH"] = ros_python + (os.pathsep + existing if existing else "")
        probe = (
            "import runpy,sys; "
            f"runpy.run_path({str(NODE_PATH)!r}, run_name='anygrasp_import_probe'); "
            "print('/usr/lib/python3/dist-packages' in sys.path)"
        )

        result = subprocess.run(
            [sys.executable, "-c", probe],
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip().splitlines()[-1], "False")


if __name__ == "__main__":
    unittest.main()

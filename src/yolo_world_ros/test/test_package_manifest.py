import sys
import unittest
from pathlib import Path


SYSTEM_DIST_PACKAGES = "/usr/lib/python3/dist-packages"
if SYSTEM_DIST_PACKAGES not in sys.path:
    sys.path.append(SYSTEM_DIST_PACKAGES)

from catkin_pkg.package import InvalidPackage, parse_package  # noqa: E402


PACKAGE_XML = Path(__file__).resolve().parents[1] / "package.xml"


class PackageManifestTest(unittest.TestCase):
    def test_manifest_declares_only_perception_runtime_dependencies(self):
        try:
            package = parse_package(str(PACKAGE_XML))
        except InvalidPackage as exc:
            self.fail(str(exc))

        self.assertEqual(package.name, "yolo_world_ros")
        dependencies = {item.name for item in package.exec_depends}
        self.assertTrue(
            {
                "anygrasp_ros",
                "rospy",
                "sensor_msgs",
                "std_msgs",
                "tf2_ros",
            }.issubset(dependencies)
        )
        self.assertTrue(
            dependencies.isdisjoint(
                {"moveit_ros_planning_interface", "ur_robot_driver", "ur_msgs"}
            )
        )


if __name__ == "__main__":
    unittest.main()

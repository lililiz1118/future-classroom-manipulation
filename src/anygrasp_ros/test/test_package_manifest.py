import sys
import unittest
from pathlib import Path


SYSTEM_DIST_PACKAGES = "/usr/lib/python3/dist-packages"
if SYSTEM_DIST_PACKAGES not in sys.path:
    sys.path.append(SYSTEM_DIST_PACKAGES)

from catkin_pkg.package import InvalidPackage, parse_package  # noqa: E402


PACKAGE_XML = Path(__file__).resolve().parents[1] / "package.xml"


class PackageManifestTest(unittest.TestCase):
    def test_manifest_passes_catkin_validation(self):
        try:
            package = parse_package(str(PACKAGE_XML))
        except InvalidPackage as exc:
            self.fail(str(exc))
        self.assertEqual(package.name, "anygrasp_ros")
        runtime_dependencies = {dependency.name for dependency in package.exec_depends}
        self.assertTrue(
            {"rospy", "sensor_msgs", "geometry_msgs", "std_msgs", "visualization_msgs"}
            .issubset(runtime_dependencies)
        )


if __name__ == "__main__":
    unittest.main()

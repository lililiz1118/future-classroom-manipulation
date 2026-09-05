#!/usr/bin/env python3
import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


SYSTEM_PATHS = (
    "/usr/lib/python3/dist-packages",
    "/opt/ros/noetic/lib/python3/dist-packages",
)
for path in SYSTEM_PATHS:
    if path not in sys.path:
        sys.path.append(path)

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parents[2]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))
NODE_PATH = PACKAGE_ROOT / "scripts" / "table_collision_updater.py"

NODE_MODULE = None
if NODE_PATH.is_file():
    try:
        spec = importlib.util.spec_from_file_location("table_collision_updater", NODE_PATH)
        NODE_MODULE = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(NODE_MODULE)
    except ImportError:
        NODE_MODULE = None


class TableCollisionPlanningSceneTest(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(NODE_MODULE, "table collision updater node is missing")

    def test_builds_one_atomic_add_diff_with_fixed_object_id(self):
        box = NODE_MODULE.build_collision_box(
            normal=(0.0, 0.0, 1.0),
            height_at_roi_center=0.3,
            roi_xy=(-0.37, 0.21, 0.23, 0.54),
            thickness=0.05,
            xy_margin=0.025,
        )

        scene = NODE_MODULE.build_planning_scene_message(
            object_id="table_surface",
            frame_id="ur_arm_base_link",
            stamp=NODE_MODULE.rospy.Time.from_sec(10.0),
            box=box,
        )

        self.assertTrue(scene.is_diff)
        self.assertTrue(scene.robot_state.is_diff)
        self.assertEqual(len(scene.world.collision_objects), 1)
        collision = scene.world.collision_objects[0]
        self.assertEqual(collision.id, "table_surface")
        self.assertEqual(collision.operation, collision.ADD)
        self.assertEqual(collision.header.frame_id, "ur_arm_base_link")
        self.assertEqual(len(collision.primitives), 1)
        np.testing.assert_allclose(collision.primitives[0].dimensions, box.size)
        np.testing.assert_allclose(
            [collision.pose.position.x, collision.pose.position.y, collision.pose.position.z],
            box.center,
        )

    def test_repeated_updates_keep_the_same_single_object_id(self):
        ids = []
        for height in (0.3, 0.31):
            box = NODE_MODULE.build_collision_box(
                normal=(0.0, 0.0, 1.0),
                height_at_roi_center=height,
                roi_xy=(-0.37, 0.21, 0.23, 0.54),
                thickness=0.05,
                xy_margin=0.025,
            )
            scene = NODE_MODULE.build_planning_scene_message(
                "table_surface",
                "ur_arm_base_link",
                NODE_MODULE.rospy.Time.from_sec(10.0),
                box,
            )
            self.assertEqual(len(scene.world.collision_objects), 1)
            ids.append(scene.world.collision_objects[0].id)

        self.assertEqual(ids, ["table_surface", "table_surface"])

    def test_rejects_an_object_id_other_than_the_fixed_table_surface_id(self):
        box = NODE_MODULE.build_collision_box(
            normal=(0.0, 0.0, 1.0),
            height_at_roi_center=0.3,
            roi_xy=(-0.37, 0.21, 0.23, 0.54),
            thickness=0.05,
            xy_margin=0.025,
        )

        with self.assertRaisesRegex(ValueError, "table_surface"):
            NODE_MODULE.build_planning_scene_message(
                object_id="table_2",
                frame_id="ur_arm_base_link",
                stamp=NODE_MODULE.rospy.Time.from_sec(10.0),
                box=box,
            )

    def test_status_message_exposes_stale_age_without_removing_collision(self):
        status = NODE_MODULE.build_status_message(
            object_id="table_surface",
            frame_id="ur_arm_base_link",
            stamp=NODE_MODULE.rospy.Time.from_sec(12.0),
            state="stale",
            age_sec=2.5,
            collision_present=True,
        )

        self.assertEqual(status.header.frame_id, "ur_arm_base_link")
        self.assertEqual(len(status.status), 1)
        entry = status.status[0]
        self.assertEqual(entry.level, entry.STALE)
        values = {item.key: item.value for item in entry.values}
        self.assertEqual(values["state"], "stale")
        self.assertEqual(values["model_age_sec"], "2.500000")
        self.assertEqual(values["collision_present"], "true")


if __name__ == "__main__":
    unittest.main()

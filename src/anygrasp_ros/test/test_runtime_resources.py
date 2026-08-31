import dataclasses
import os
import tempfile
import unittest
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT / "src"
if str(SOURCE_ROOT) not in os.sys.path:
    os.sys.path.insert(0, str(SOURCE_ROOT))

from anygrasp_ros.runtime_resources import (  # noqa: E402
    ResourcePolicyError,
    apply_process_environment,
    configure_torch,
    load_resource_policy,
)


class AnyGraspResourcePolicyTest(unittest.TestCase):
    def _write_policy(self, document):
        handle = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
        yaml.safe_dump(document, handle)
        handle.close()
        self.addCleanup(lambda: os.unlink(handle.name))
        return handle.name

    def test_checked_in_policy_uses_conservative_configurable_limits(self):
        policy = load_resource_policy(
            str(PACKAGE_ROOT / "config" / "anygrasp_resources.yaml")
        )

        self.assertEqual(policy.torch_intra_op_threads, 2)
        self.assertEqual(policy.torch_inter_op_threads, 1)
        self.assertEqual(policy.omp_threads, 2)
        self.assertEqual(policy.mkl_threads, 2)
        self.assertEqual(policy.openblas_threads, 2)
        self.assertEqual(policy.nice_increment, 10)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            policy.omp_threads = 8

    def test_process_environment_and_nice_are_applied_from_one_policy(self):
        policy = load_resource_policy(
            str(PACKAGE_ROOT / "config" / "anygrasp_resources.yaml")
        )
        environment = {"OMP_NUM_THREADS": "99", "UNRELATED": "keep"}
        nice_calls = []

        report = apply_process_environment(
            policy,
            environment=environment,
            nice=lambda increment: nice_calls.append(increment) or 12,
        )

        self.assertEqual(environment["OMP_NUM_THREADS"], "2")
        self.assertEqual(environment["MKL_NUM_THREADS"], "2")
        self.assertEqual(environment["OPENBLAS_NUM_THREADS"], "2")
        self.assertEqual(environment["UNRELATED"], "keep")
        self.assertEqual(nice_calls, [10])
        self.assertEqual(report.effective_nice, 12)

    def test_torch_thread_limits_are_applied_and_reported(self):
        policy = load_resource_policy(
            str(PACKAGE_ROOT / "config" / "anygrasp_resources.yaml")
        )

        class FakeTorch:
            def __init__(self):
                self.intra = 16
                self.inter = 8

            def set_num_threads(self, value):
                self.intra = value

            def set_num_interop_threads(self, value):
                self.inter = value

            def get_num_threads(self):
                return self.intra

            def get_num_interop_threads(self):
                return self.inter

        torch = FakeTorch()

        report = configure_torch(torch, policy)

        self.assertEqual(torch.intra, 2)
        self.assertEqual(torch.inter, 1)
        self.assertEqual(report.effective_intra_op_threads, 2)
        self.assertEqual(report.effective_inter_op_threads, 1)

    def test_invalid_values_unknown_keys_and_nice_failure_are_fatal(self):
        valid = {
            "pytorch": {"intra_op_threads": 2, "inter_op_threads": 1},
            "environment": {
                "omp_threads": 2,
                "mkl_threads": 2,
                "openblas_threads": 2,
            },
            "process": {"nice_increment": 10},
        }
        invalid_documents = [
            {},
            dict(valid, unknown=True),
            dict(valid, pytorch={"intra_op_threads": 0, "inter_op_threads": 1}),
            dict(valid, process={"nice_increment": 20}),
        ]
        for document in invalid_documents:
            with self.subTest(document=document):
                with self.assertRaises(ResourcePolicyError):
                    load_resource_policy(self._write_policy(document))

        policy = load_resource_policy(self._write_policy(valid))
        with self.assertRaisesRegex(ResourcePolicyError, "nice"):
            apply_process_environment(
                policy,
                environment={},
                nice=lambda _increment: (_ for _ in ()).throw(OSError("denied")),
            )


if __name__ == "__main__":
    unittest.main()

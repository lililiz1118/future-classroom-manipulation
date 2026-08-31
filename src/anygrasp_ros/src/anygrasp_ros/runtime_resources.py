"""Central CPU-resource policy for the independently launched AnyGrasp node."""

from dataclasses import dataclass
import os
from typing import Any, Dict

import yaml


RESOURCE_CONFIG_ENV = "ANYGRASP_RESOURCE_CONFIG"


class ResourcePolicyError(RuntimeError):
    """AnyGrasp resource configuration could not be applied safely."""


@dataclass(frozen=True)
class AnyGraspResourcePolicy:
    torch_intra_op_threads: int
    torch_inter_op_threads: int
    omp_threads: int
    mkl_threads: int
    openblas_threads: int
    nice_increment: int


@dataclass(frozen=True)
class ProcessResourceReport:
    omp_threads: int
    mkl_threads: int
    openblas_threads: int
    nice_increment: int
    effective_nice: int


@dataclass(frozen=True)
class TorchResourceReport:
    requested_intra_op_threads: int
    requested_inter_op_threads: int
    effective_intra_op_threads: int
    effective_inter_op_threads: int


def _exact_map(value: Any, keys, context: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ResourcePolicyError("%s must be a map" % context)
    if set(value) != set(keys):
        raise ResourcePolicyError(
            "%s keys must be exactly %s; got %s"
            % (context, sorted(keys), sorted(value))
        )
    return value


def _positive_thread_count(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ResourcePolicyError("%s must be a positive integer" % context)
    return value


def load_resource_policy(path: str) -> AnyGraspResourcePolicy:
    try:
        with open(path, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise ResourcePolicyError(
            "Cannot read AnyGrasp resource policy %s: %s" % (path, exc)
        ) from exc
    root = _exact_map(
        document, {"pytorch", "environment", "process"}, "resource policy"
    )
    pytorch = _exact_map(
        root["pytorch"],
        {"intra_op_threads", "inter_op_threads"},
        "resource policy pytorch",
    )
    environment = _exact_map(
        root["environment"],
        {"omp_threads", "mkl_threads", "openblas_threads"},
        "resource policy environment",
    )
    process = _exact_map(
        root["process"], {"nice_increment"}, "resource policy process"
    )
    nice_increment = process["nice_increment"]
    if (
        isinstance(nice_increment, bool)
        or not isinstance(nice_increment, int)
        or not 0 <= nice_increment <= 19
    ):
        raise ResourcePolicyError("process.nice_increment must be in [0, 19]")
    return AnyGraspResourcePolicy(
        torch_intra_op_threads=_positive_thread_count(
            pytorch["intra_op_threads"], "pytorch.intra_op_threads"
        ),
        torch_inter_op_threads=_positive_thread_count(
            pytorch["inter_op_threads"], "pytorch.inter_op_threads"
        ),
        omp_threads=_positive_thread_count(
            environment["omp_threads"], "environment.omp_threads"
        ),
        mkl_threads=_positive_thread_count(
            environment["mkl_threads"], "environment.mkl_threads"
        ),
        openblas_threads=_positive_thread_count(
            environment["openblas_threads"], "environment.openblas_threads"
        ),
        nice_increment=nice_increment,
    )


def apply_process_environment(
    policy: AnyGraspResourcePolicy,
    environment=None,
    nice=os.nice,
) -> ProcessResourceReport:
    target = os.environ if environment is None else environment
    target["OMP_NUM_THREADS"] = str(policy.omp_threads)
    target["MKL_NUM_THREADS"] = str(policy.mkl_threads)
    target["OPENBLAS_NUM_THREADS"] = str(policy.openblas_threads)
    try:
        effective_nice = nice(policy.nice_increment)
    except OSError as exc:
        raise ResourcePolicyError(
            "Cannot apply AnyGrasp nice increment %d: %s"
            % (policy.nice_increment, exc)
        ) from exc
    return ProcessResourceReport(
        omp_threads=policy.omp_threads,
        mkl_threads=policy.mkl_threads,
        openblas_threads=policy.openblas_threads,
        nice_increment=policy.nice_increment,
        effective_nice=int(effective_nice),
    )


def configure_torch(torch_module: Any, policy: AnyGraspResourcePolicy) -> TorchResourceReport:
    torch_module.set_num_threads(policy.torch_intra_op_threads)
    torch_module.set_num_interop_threads(policy.torch_inter_op_threads)
    return TorchResourceReport(
        requested_intra_op_threads=policy.torch_intra_op_threads,
        requested_inter_op_threads=policy.torch_inter_op_threads,
        effective_intra_op_threads=int(torch_module.get_num_threads()),
        effective_inter_op_threads=int(torch_module.get_num_interop_threads()),
    )


_INITIALIZED = None


def initialize_resource_policy(default_path: str):
    global _INITIALIZED
    selected_path = os.environ.get(RESOURCE_CONFIG_ENV, default_path)
    selected_path = os.path.realpath(selected_path)
    if _INITIALIZED is not None:
        initialized_path, policy, report = _INITIALIZED
        if selected_path != initialized_path:
            raise ResourcePolicyError(
                "AnyGrasp resources already initialized from %s, cannot switch to %s"
                % (initialized_path, selected_path)
            )
        return policy, report
    policy = load_resource_policy(selected_path)
    report = apply_process_environment(policy)
    _INITIALIZED = (selected_path, policy, report)
    return policy, report


def format_process_report(report: ProcessResourceReport) -> str:
    return (
        "[AnyGrasp resources] OMP=%d MKL=%d OPENBLAS=%d "
        "nice_increment=%d effective_nice=%d"
        % (
            report.omp_threads,
            report.mkl_threads,
            report.openblas_threads,
            report.nice_increment,
            report.effective_nice,
        )
    )


def format_torch_report(report: TorchResourceReport) -> str:
    return (
        "[AnyGrasp resources] PyTorch intra-op=%d (effective=%d) "
        "inter-op=%d (effective=%d)"
        % (
            report.requested_intra_op_threads,
            report.effective_intra_op_threads,
            report.requested_inter_op_threads,
            report.effective_inter_op_threads,
        )
    )

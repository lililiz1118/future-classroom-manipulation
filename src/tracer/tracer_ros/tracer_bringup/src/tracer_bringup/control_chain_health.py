"""Pure, latched health model for the UR3 motion-control chain."""

from dataclasses import dataclass, replace
from enum import Enum
import threading
from typing import List, Optional

from .runtime_config import UrRuntimePolicy


class ControlChainState(Enum):
    STARTING = "STARTING"
    READY = "READY"
    FAULT = "FAULT"


@dataclass(frozen=True)
class ControlChainSnapshot:
    robot_mode: Optional[str] = None
    safety_mode: Optional[str] = None
    robot_program_running: Optional[bool] = None
    controller_healthy: Optional[bool] = None
    controller_reason: str = "not observed"
    joint_state_complete: bool = False
    joint_state_valid: bool = False
    joint_received_at: Optional[float] = None
    joint_header_stamp: Optional[float] = None
    joint_header_age: Optional[float] = None
    advancing_joint_samples: int = 0


class ControlChainHealth:
    """Own STARTING -> READY -> FAULT and never leave FAULT."""

    def __init__(self, policy: UrRuntimePolicy):
        self._policy = policy
        self._state = ControlChainState.STARTING
        self._fault_reason = None
        self._snapshot = ControlChainSnapshot()
        self._lock = threading.RLock()

    @property
    def state(self) -> ControlChainState:
        with self._lock:
            return self._state

    @property
    def fault_reason(self) -> Optional[str]:
        with self._lock:
            return self._fault_reason

    @property
    def snapshot(self) -> ControlChainSnapshot:
        with self._lock:
            return self._snapshot

    def _fault(self, reason: str) -> None:
        if self._state is ControlChainState.FAULT:
            return
        self._state = ControlChainState.FAULT
        self._fault_reason = reason

    def observe_robot_mode(self, mode: str) -> None:
        normalized = str(mode).strip().upper()
        with self._lock:
            self._snapshot = replace(self._snapshot, robot_mode=normalized)
            if self._state is ControlChainState.READY and normalized != "RUNNING":
                self._fault("robot mode=%s (expected RUNNING)" % normalized)

    def observe_safety_mode(self, mode: str) -> None:
        normalized = str(mode).strip().upper()
        with self._lock:
            self._snapshot = replace(self._snapshot, safety_mode=normalized)
            if self._state is ControlChainState.READY and normalized != "NORMAL":
                self._fault("safety mode=%s (expected NORMAL)" % normalized)

    def observe_program_running(self, running: bool) -> None:
        value = bool(running)
        with self._lock:
            self._snapshot = replace(
                self._snapshot, robot_program_running=value
            )
            if self._state is ControlChainState.READY and not value:
                self._fault("robot_program_running=False")

    def observe_controller(self, healthy: bool, reason: str = "") -> None:
        value = bool(healthy)
        description = reason.strip() or ("running" if value else "not running")
        with self._lock:
            self._snapshot = replace(
                self._snapshot,
                controller_healthy=value,
                controller_reason=description,
            )
            if self._state is ControlChainState.READY and not value:
                self._fault("trajectory controller unhealthy: %s" % description)

    def observe_joint_state(
        self,
        received_at: float,
        header_stamp: float,
        header_age: float,
        complete: bool,
    ) -> None:
        with self._lock:
            previous_stamp = self._snapshot.joint_header_stamp
            samples = self._snapshot.advancing_joint_samples
            reason = None
            if not complete:
                reason = "joint_states missing required UR joints"
            elif header_age < 0.0:
                reason = "joint_states timestamp is in the future"
            elif header_age > self._policy.joint_state_timeout:
                reason = "joint_states header is stale by %.3fs" % header_age
            elif previous_stamp is not None and header_stamp <= previous_stamp:
                reason = "joint_states timestamp did not advance"

            valid = reason is None
            if valid:
                samples += 1
            self._snapshot = replace(
                self._snapshot,
                joint_state_complete=bool(complete),
                joint_state_valid=valid,
                joint_received_at=float(received_at),
                joint_header_stamp=(
                    float(header_stamp) if valid else previous_stamp
                ),
                joint_header_age=float(header_age),
                advancing_joint_samples=samples,
            )
            if self._state is ControlChainState.READY and reason is not None:
                self._fault(reason)

    def _readiness_blockers(self, now: float) -> List[str]:
        snapshot = self._snapshot
        blockers = []
        if snapshot.robot_mode != "RUNNING":
            blockers.append("robot mode=%s" % (snapshot.robot_mode or "unavailable"))
        if snapshot.safety_mode != "NORMAL":
            blockers.append("safety mode=%s" % (snapshot.safety_mode or "unavailable"))
        if snapshot.robot_program_running is not True:
            blockers.append(
                "robot_program_running=%s" % snapshot.robot_program_running
            )
        if snapshot.controller_healthy is not True:
            blockers.append("trajectory controller: %s" % snapshot.controller_reason)
        if not snapshot.joint_state_complete:
            blockers.append("joint_states missing required UR joints")
        elif not snapshot.joint_state_valid:
            blockers.append("joint_states invalid")
        if snapshot.advancing_joint_samples < self._policy.ready_joint_samples:
            blockers.append(
                "joint_states advancing samples=%d/%d"
                % (
                    snapshot.advancing_joint_samples,
                    self._policy.ready_joint_samples,
                )
            )
        if snapshot.joint_received_at is None:
            blockers.append("joint_states unavailable")
        else:
            age = now - snapshot.joint_received_at
            if age > self._policy.joint_state_timeout:
                blockers.append(
                    "joint_states stale: %.3fs > %.3fs"
                    % (age, self._policy.joint_state_timeout)
                )
        return blockers

    def readiness_blockers(self, now: float) -> List[str]:
        with self._lock:
            if self._state is ControlChainState.FAULT:
                return [self._fault_reason]
            return self._readiness_blockers(now)

    def evaluate(self, now: float) -> ControlChainState:
        with self._lock:
            if self._state is ControlChainState.FAULT:
                return self._state
            if self._snapshot.joint_received_at is not None:
                age = now - self._snapshot.joint_received_at
                if (
                    self._state is ControlChainState.READY
                    and age > self._policy.joint_state_timeout
                ):
                    self._fault(
                        "joint_states stale: %.3fs > %.3fs"
                        % (age, self._policy.joint_state_timeout)
                    )
                    return self._state
            if self._state is ControlChainState.STARTING and not self._readiness_blockers(now):
                self._state = ControlChainState.READY
            return self._state

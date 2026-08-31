"""ROS adapter for the pure UR3 control-chain health model."""

from types import SimpleNamespace
import threading
import time
from typing import Any, Iterable

from .control_chain_health import ControlChainHealth, ControlChainState
from .headless_startup import assert_exclusive_controller
from .runtime_config import UrRuntimePolicy


ROBOT_MODE_TOPIC = "/ur/ur_hardware_interface/robot_mode"
SAFETY_MODE_TOPIC = "/ur/ur_hardware_interface/safety_mode"
PROGRAM_RUNNING_TOPIC = "/ur/ur_hardware_interface/robot_program_running"
JOINT_STATE_TOPIC = "/ur/joint_states"
LIST_CONTROLLERS_SERVICE = "/ur/controller_manager/list_controllers"


class ControlChainFault(RuntimeError):
    """A previously READY control chain entered terminal FAULT."""


class ControlChainNotReady(RuntimeError):
    """The control chain did not satisfy every READY condition in time."""


def controller_snapshot(response: Any):
    return [
        {
            "name": controller.name,
            "state": controller.state,
            "claimed_resources": [
                {
                    "hardware_interface": claim.hardware_interface,
                    "resources": list(claim.resources),
                }
                for claim in controller.claimed_resources
            ],
        }
        for controller in response.controller
    ]


def _default_message_types():
    from controller_manager_msgs.srv import ListControllers
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from ur_dashboard_msgs.msg import RobotMode, SafetyMode

    return SimpleNamespace(
        RobotMode=RobotMode,
        SafetyMode=SafetyMode,
        Bool=Bool,
        JointState=JointState,
        ListControllers=ListControllers,
    )


def _mode_names(message_type: Any):
    return {
        value: name
        for name, value in vars(message_type).items()
        if name.isupper() and isinstance(value, int)
    }


class RosControlChainMonitor:
    def __init__(
        self,
        rospy_module: Any,
        policy: UrRuntimePolicy,
        required_joints: Iterable[str],
        target_controller: str,
        diagnostic_output=print,
        message_types=None,
        monotonic=time.monotonic,
    ):
        self._rospy = rospy_module
        self._policy = policy
        self._required_joints = set(required_joints)
        self._target_controller = target_controller
        self._diagnostic_output = diagnostic_output
        self._types = message_types or _default_message_types()
        self._monotonic = monotonic
        self._health = ControlChainHealth(policy)
        self._robot_mode_names = _mode_names(self._types.RobotMode)
        self._safety_mode_names = _mode_names(self._types.SafetyMode)
        self._subscriptions = []
        self._controller_service = None
        self._stop_event = threading.Event()
        self._changed = threading.Event()
        self._worker = None
        self._fault_reported = False
        self._ready_reported = False

    @property
    def state(self):
        return self._health.state

    @property
    def fault_reason(self):
        return self._health.fault_reason

    @property
    def snapshot(self):
        return self._health.snapshot

    def _emit(self, message: str) -> None:
        try:
            self._diagnostic_output(message)
        except Exception:
            pass

    def _notify(self) -> None:
        self._changed.set()
        if self.state is ControlChainState.FAULT and not self._fault_reported:
            self._fault_reported = True
            self._emit(
                "❌ CONTROL_CHAIN_STATE=FAULT｜原因: %s｜必须完整重启控制链"
                % self.fault_reason
            )

    @staticmethod
    def _normalize_mode(names, value):
        return names.get(value, "UNKNOWN(%s)" % value)

    def _observe_robot_mode(self, message: Any) -> None:
        self._health.observe_robot_mode(
            self._normalize_mode(self._robot_mode_names, message.mode)
        )
        self._notify()

    def _observe_safety_mode(self, message: Any) -> None:
        self._health.observe_safety_mode(
            self._normalize_mode(self._safety_mode_names, message.mode)
        )
        self._notify()

    def _observe_program_running(self, message: Any) -> None:
        self._health.observe_program_running(message.data)
        self._notify()

    def _observe_joint_state(self, message: Any) -> None:
        try:
            header_age = (self._rospy.Time.now() - message.header.stamp).to_sec()
            header_stamp = message.header.stamp.to_sec()
        except (AttributeError, TypeError) as exc:
            self._health.observe_joint_state(
                self._monotonic(), 0.0, 0.0, False
            )
            self._notify()
            self._emit("⚠️ Invalid /ur/joint_states header: %s" % exc)
            return
        self._health.observe_joint_state(
            self._monotonic(),
            header_stamp,
            header_age,
            self._required_joints.issubset(message.name),
        )
        self._notify()

    def _poll_controller(self) -> None:
        try:
            response = self._controller_service()
            assert_exclusive_controller(
                controller_snapshot(response),
                self._target_controller,
                self._required_joints,
            )
        except Exception as exc:
            self._health.observe_controller(False, str(exc))
        else:
            self._health.observe_controller(True, "running exclusively")
        self._notify()

    def _run(self) -> None:
        next_controller_poll = 0.0
        while not self._stop_event.is_set() and not self._rospy.is_shutdown():
            now = self._monotonic()
            if now >= next_controller_poll:
                self._poll_controller()
                next_controller_poll = now + self._policy.controller_poll_period
            self._health.evaluate(now)
            self._notify()
            self._stop_event.wait(self._policy.health_evaluation_period)

    def start(self) -> None:
        if self._worker is not None:
            return
        self._subscriptions = [
            self._rospy.Subscriber(
                ROBOT_MODE_TOPIC,
                self._types.RobotMode,
                self._observe_robot_mode,
                queue_size=1,
            ),
            self._rospy.Subscriber(
                SAFETY_MODE_TOPIC,
                self._types.SafetyMode,
                self._observe_safety_mode,
                queue_size=1,
            ),
            self._rospy.Subscriber(
                PROGRAM_RUNNING_TOPIC,
                self._types.Bool,
                self._observe_program_running,
                queue_size=1,
            ),
            self._rospy.Subscriber(
                JOINT_STATE_TOPIC,
                self._types.JointState,
                self._observe_joint_state,
                queue_size=10,
            ),
        ]
        self._controller_service = self._rospy.ServiceProxy(
            LIST_CONTROLLERS_SERVICE, self._types.ListControllers
        )
        self._emit("🔄 CONTROL_CHAIN_STATE=STARTING")
        self._worker = threading.Thread(
            target=self._run,
            name="ur3-control-chain-health",
            daemon=True,
        )
        self._worker.start()

    def wait_until_ready(self, timeout: float):
        deadline = self._monotonic() + timeout
        while self._monotonic() < deadline:
            self.raise_if_fault()
            state = self._health.evaluate(self._monotonic())
            if state is ControlChainState.READY:
                if not self._ready_reported:
                    self._ready_reported = True
                    self._emit("✅ CONTROL_CHAIN_STATE=READY")
                return state
            remaining = deadline - self._monotonic()
            self._changed.wait(
                max(0.0, min(self._policy.health_evaluation_period, remaining))
            )
            self._changed.clear()
        blockers = self._health.readiness_blockers(self._monotonic())
        raise ControlChainNotReady(
            "Control chain remained STARTING: %s" % "; ".join(blockers)
        )

    def raise_if_fault(self) -> None:
        if self.state is ControlChainState.FAULT:
            raise ControlChainFault(self.fault_reason)

    def stop(self) -> None:
        self._stop_event.set()
        for subscription in self._subscriptions:
            try:
                subscription.unregister()
            except Exception:
                pass
        self._subscriptions = []
        if self._worker is not None:
            self._worker.join(timeout=1.0)
            self._worker = None

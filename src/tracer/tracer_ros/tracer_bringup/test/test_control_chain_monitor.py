#!/usr/bin/env python3
from types import SimpleNamespace
import os
import sys
import time
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from tracer_bringup.control_chain_health import ControlChainState  # noqa: E402
from tracer_bringup.control_chain_monitor import (  # noqa: E402
    ControlChainFault,
    ControlChainNotReady,
    RosControlChainMonitor,
)
from tracer_bringup.headless_runtime import REQUIRED_JOINTS, TARGET_CONTROLLER  # noqa: E402
from tracer_bringup.runtime_config import UrRuntimePolicy  # noqa: E402


POLICY = UrRuntimePolicy(
    robot_receive_timeout=0.10,
    health_evaluation_period=0.01,
    controller_poll_period=0.01,
    joint_state_timeout=0.50,
    ready_joint_samples=2,
)


class Duration:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds


class Stamp:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds

    def __sub__(self, other):
        return Duration(self.seconds - other.seconds)


class FakeSubscription:
    def __init__(self, topic, callback):
        self.topic = topic
        self.callback = callback
        self.unregistered = False

    def unregister(self):
        self.unregistered = True


class FakeRospy:
    class ROSException(Exception):
        pass

    class ServiceException(Exception):
        pass

    def __init__(self):
        self.subscriptions = {}
        self.now = Stamp(100.02)
        self.Time = SimpleNamespace(now=lambda: self.now)
        self.controller_response = controller_response(True)

    def Subscriber(self, topic, _message_type, callback, queue_size=1):
        subscription = FakeSubscription(topic, callback)
        self.subscriptions[topic] = subscription
        return subscription

    def ServiceProxy(self, _name, _service_type):
        return lambda: self.controller_response

    def is_shutdown(self):
        return False

    def publish(self, topic, message):
        self.subscriptions[topic].callback(message)


class RobotMode:
    RUNNING = 7
    IDLE = 5


class SafetyMode:
    NORMAL = 1
    REDUCED = 2


class Bool:
    pass


class JointState:
    pass


class ListControllers:
    pass


MESSAGE_TYPES = SimpleNamespace(
    RobotMode=RobotMode,
    SafetyMode=SafetyMode,
    Bool=Bool,
    JointState=JointState,
    ListControllers=ListControllers,
)


def controller_response(running):
    claim = SimpleNamespace(
        hardware_interface="hardware_interface::PositionJointInterface",
        resources=list(REQUIRED_JOINTS),
    )
    controller = SimpleNamespace(
        name=TARGET_CONTROLLER,
        state="running" if running else "stopped",
        claimed_resources=[claim],
    )
    return SimpleNamespace(controller=[controller])


def publish_ready_inputs(rospy):
    rospy.publish("/ur/ur_hardware_interface/robot_mode", SimpleNamespace(mode=7))
    rospy.publish("/ur/ur_hardware_interface/safety_mode", SimpleNamespace(mode=1))
    rospy.publish(
        "/ur/ur_hardware_interface/robot_program_running",
        SimpleNamespace(data=True),
    )
    rospy.publish(
        "/ur/joint_states",
        SimpleNamespace(
            name=list(REQUIRED_JOINTS),
            header=SimpleNamespace(stamp=Stamp(100.00)),
        ),
    )
    rospy.publish(
        "/ur/joint_states",
        SimpleNamespace(
            name=list(REQUIRED_JOINTS),
            header=SimpleNamespace(stamp=Stamp(100.01)),
        ),
    )


class RosControlChainMonitorTest(unittest.TestCase):
    def monitor(self, rospy, diagnostics=None):
        return RosControlChainMonitor(
            rospy,
            POLICY,
            required_joints=REQUIRED_JOINTS,
            target_controller=TARGET_CONTROLLER,
            message_types=MESSAGE_TYPES,
            diagnostic_output=(diagnostics or []).append,
        )

    def test_topics_and_controller_poll_form_real_ready_state(self):
        rospy = FakeRospy()
        monitor = self.monitor(rospy)
        monitor.start()
        self.addCleanup(monitor.stop)

        publish_ready_inputs(rospy)

        self.assertIs(monitor.wait_until_ready(1.0), ControlChainState.READY)
        self.assertEqual(monitor.state, ControlChainState.READY)
        self.assertEqual(monitor.snapshot.robot_mode, "RUNNING")
        self.assertEqual(monitor.snapshot.safety_mode, "NORMAL")

    def test_readiness_timeout_reports_every_missing_condition(self):
        monitor = self.monitor(FakeRospy())
        monitor.start()
        self.addCleanup(monitor.stop)

        with self.assertRaises(ControlChainNotReady) as raised:
            monitor.wait_until_ready(0.03)

        message = str(raised.exception)
        self.assertIn("robot mode=unavailable", message)
        self.assertIn("safety mode=unavailable", message)
        self.assertIn("robot_program_running=None", message)
        self.assertIn("joint_states unavailable", message)

    def test_program_false_after_ready_latches_fault(self):
        rospy = FakeRospy()
        monitor = self.monitor(rospy)
        monitor.start()
        self.addCleanup(monitor.stop)
        publish_ready_inputs(rospy)
        monitor.wait_until_ready(1.0)

        rospy.publish(
            "/ur/ur_hardware_interface/robot_program_running",
            SimpleNamespace(data=False),
        )
        rospy.publish(
            "/ur/ur_hardware_interface/robot_program_running",
            SimpleNamespace(data=True),
        )

        with self.assertRaisesRegex(
            ControlChainFault, "robot_program_running=False"
        ):
            monitor.raise_if_fault()

    def test_controller_poll_faults_when_target_stops(self):
        rospy = FakeRospy()
        monitor = self.monitor(rospy)
        monitor.start()
        self.addCleanup(monitor.stop)
        publish_ready_inputs(rospy)
        monitor.wait_until_ready(1.0)

        rospy.controller_response = controller_response(False)
        deadline = time.monotonic() + 1.0
        while monitor.state is not ControlChainState.FAULT and time.monotonic() < deadline:
            time.sleep(0.01)

        with self.assertRaisesRegex(
            ControlChainFault, "Target controller is not running"
        ):
            monitor.raise_if_fault()

    def test_stop_is_idempotent_and_unregisters_all_topics(self):
        rospy = FakeRospy()
        monitor = self.monitor(rospy)
        monitor.start()

        monitor.stop()
        monitor.stop()

        self.assertEqual(len(rospy.subscriptions), 4)
        self.assertTrue(
            all(item.unregistered for item in rospy.subscriptions.values())
        )


if __name__ == "__main__":
    unittest.main()

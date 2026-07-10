#!/usr/bin/env python3
"""Publish deterministic step or sine actions for the UR5e execution node."""

import math
import threading

import rospy
from std_msgs.msg import Float64MultiArray


class RLActionGenerator:
    JOINT_COUNT = 6

    def __init__(self):
        self._signal_type = rospy.get_param("~signal_type", "step").lower()
        self._action_mode = rospy.get_param("~action_mode", "position").lower()
        self._topic = rospy.get_param("~rl_action_topic", "/rl_action")
        self._rl_state_topic = rospy.get_param("~rl_state_topic", "/rl_state")
        self._action_scale = float(rospy.get_param("~action_scale", 1.0))
        self._max_delta_action = float(
            rospy.get_param("~generator_max_delta_action", 1.0)
        )
        self._publish_rate = float(rospy.get_param("~generator_publish_rate", 50.0))
        self._start_delay = float(rospy.get_param("~generator_start_delay", 2.0))
        self._frequency = float(rospy.get_param("~sine_frequency", 0.2))
        self._baseline = self._read_vector("generator_baseline")
        self._amplitude = self._read_vector("generator_amplitude")

        if self._signal_type not in ("step", "sine"):
            raise rospy.ROSInitException("signal_type must be 'step' or 'sine'")
        if self._action_mode not in ("position", "delta"):
            raise rospy.ROSInitException("action_mode must be 'position' or 'delta'")
        if not math.isfinite(self._action_scale) or self._action_scale <= 0.0:
            raise rospy.ROSInitException("action_scale must be finite and positive")
        if self._max_delta_action < 0.0:
            raise rospy.ROSInitException("generator_max_delta_action cannot be negative")
        if self._publish_rate <= 0.0:
            raise rospy.ROSInitException("generator_publish_rate must be positive")
        if self._start_delay < 0.0:
            raise rospy.ROSInitException("generator_start_delay cannot be negative")
        if self._signal_type == "sine" and self._frequency <= 0.0:
            raise rospy.ROSInitException("sine_frequency must be positive")

        self._state_lock = threading.Lock()
        self._joint_positions = None
        self._state_subscriber = rospy.Subscriber(
            self._rl_state_topic,
            Float64MultiArray,
            self._rl_state_callback,
            queue_size=1,
        )
        self._publisher = rospy.Publisher(
            self._topic, Float64MultiArray, queue_size=1, latch=True
        )
        self._start_time = rospy.Time.now()
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self._publish_rate), self._timer_callback
        )
        rospy.loginfo(
            "RL action generator: %s signal in %s mode -> %s at %.1f Hz",
            self._signal_type,
            self._action_mode,
            self._topic,
            self._publish_rate,
        )
        rospy.logwarn(
            "Test signal amplitude is %s rad; keep controller output disabled "
            "until the trajectory is verified in RViz",
            self._amplitude,
        )

    def _read_vector(self, parameter):
        values = rospy.get_param("~" + parameter)
        if len(values) != self.JOINT_COUNT:
            raise rospy.ROSInitException(
                "%s must contain exactly six values" % parameter
            )
        result = [float(value) for value in values]
        if not all(math.isfinite(value) for value in result):
            raise rospy.ROSInitException("%s contains NaN or infinity" % parameter)
        return result

    def _rl_state_callback(self, message):
        """Store q[0:6] from [q(6), dq(6), tau(6)] as numeric values."""
        if len(message.data) < self.JOINT_COUNT:
            rospy.logwarn_throttle(
                2.0, "Ignoring /rl_state with fewer than six values"
            )
            return
        positions = [float(value) for value in message.data[: self.JOINT_COUNT]]
        if not all(math.isfinite(value) for value in positions):
            rospy.logwarn_throttle(2.0, "Ignoring non-finite /rl_state positions")
            return
        with self._state_lock:
            self._joint_positions = positions

    def _desired_to_action(self, desired_positions):
        if self._action_mode == "position":
            return [value / self._action_scale for value in desired_positions]

        with self._state_lock:
            actual_positions = (
                None
                if self._joint_positions is None
                else list(self._joint_positions)
            )
        if actual_positions is None:
            rospy.logwarn_throttle(
                2.0, "Waiting for /rl_state before publishing delta actions"
            )
            return None

        actions = [
            (desired - actual) / self._action_scale
            for desired, actual in zip(desired_positions, actual_positions)
        ]
        if self._max_delta_action > 0.0:
            limit = self._max_delta_action
            actions = [max(-limit, min(limit, value)) for value in actions]
        return actions

    def _timer_callback(self, _event):
        elapsed = max(0.0, (rospy.Time.now() - self._start_time).to_sec())
        active_time = elapsed - self._start_delay

        if active_time < 0.0:
            desired_positions = list(self._baseline)
        elif self._signal_type == "step":
            desired_positions = [
                baseline + amplitude
                for baseline, amplitude in zip(self._baseline, self._amplitude)
            ]
        else:
            wave = math.sin(2.0 * math.pi * self._frequency * active_time)
            desired_positions = [
                baseline + amplitude * wave
                for baseline, amplitude in zip(self._baseline, self._amplitude)
            ]

        actions = self._desired_to_action(desired_positions)
        if actions is None:
            return
        message = Float64MultiArray()
        message.data = actions
        self._publisher.publish(message)


if __name__ == "__main__":
    rospy.init_node("rl_action_generator")
    RLActionGenerator()
    rospy.spin()

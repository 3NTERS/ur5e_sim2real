#!/usr/bin/env python3
"""Publish deterministic step or sine actions for the UR5e execution node."""

import math

import rospy
from std_msgs.msg import Float64MultiArray


class RLActionGenerator:
    JOINT_COUNT = 6

    def __init__(self):
        self._signal_type = rospy.get_param("~signal_type", "step").lower()
        self._topic = rospy.get_param("~rl_action_topic", "/rl_action")
        self._publish_rate = float(rospy.get_param("~generator_publish_rate", 50.0))
        self._start_delay = float(rospy.get_param("~generator_start_delay", 2.0))
        self._frequency = float(rospy.get_param("~sine_frequency", 0.2))
        self._baseline = self._read_vector("generator_baseline")
        self._amplitude = self._read_vector("generator_amplitude")

        if self._signal_type not in ("step", "sine"):
            raise rospy.ROSInitException("signal_type must be 'step' or 'sine'")
        if self._publish_rate <= 0.0:
            raise rospy.ROSInitException("generator_publish_rate must be positive")
        if self._start_delay < 0.0:
            raise rospy.ROSInitException("generator_start_delay cannot be negative")
        if self._signal_type == "sine" and self._frequency <= 0.0:
            raise rospy.ROSInitException("sine_frequency must be positive")

        self._publisher = rospy.Publisher(
            self._topic, Float64MultiArray, queue_size=1, latch=True
        )
        self._start_time = rospy.Time.now()
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self._publish_rate), self._timer_callback
        )
        rospy.loginfo(
            "RL action generator: %s signal -> %s at %.1f Hz",
            self._signal_type,
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

    def _timer_callback(self, _event):
        elapsed = max(0.0, (rospy.Time.now() - self._start_time).to_sec())
        active_time = elapsed - self._start_delay

        if active_time < 0.0:
            positions = list(self._baseline)
        elif self._signal_type == "step":
            positions = [
                baseline + amplitude
                for baseline, amplitude in zip(self._baseline, self._amplitude)
            ]
        else:
            wave = math.sin(2.0 * math.pi * self._frequency * active_time)
            positions = [
                baseline + amplitude * wave
                for baseline, amplitude in zip(self._baseline, self._amplitude)
            ]

        message = Float64MultiArray()
        message.data = positions
        self._publisher.publish(message)


if __name__ == "__main__":
    rospy.init_node("rl_action_generator")
    RLActionGenerator()
    rospy.spin()

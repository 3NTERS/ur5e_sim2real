#!/usr/bin/env python3
"""Normalize UR5e joint commands and publish them for robot_state_publisher."""

import math
import threading

import rospy
from sensor_msgs.msg import JointState


class JointStateVisualizer:
    def __init__(self):
        self._joint_names = rospy.get_param("~joint_names")
        initial = rospy.get_param("~initial_positions", [0.0] * len(self._joint_names))
        self._rate = float(rospy.get_param("~publish_rate", 50.0))
        self._timeout = float(rospy.get_param("~command_timeout", 0.0))
        input_topic = rospy.get_param("~input_topic", "/ur5e/joint_command")
        output_topic = rospy.get_param("~output_topic", "/joint_states")

        if len(self._joint_names) != 6:
            raise rospy.ROSInitException("joint_names must contain the six UR5e joints")
        if len(initial) != len(self._joint_names):
            raise rospy.ROSInitException("initial_positions must match joint_names")
        if input_topic == output_topic:
            raise rospy.ROSInitException("input_topic and output_topic must be different")
        if self._rate <= 0.0:
            raise rospy.ROSInitException("publish_rate must be greater than zero")

        self._lock = threading.Lock()
        self._positions = [float(value) for value in initial]
        self._velocities = []
        self._efforts = []
        self._last_command_time = None
        self._timeout_reported = False

        self._publisher = rospy.Publisher(output_topic, JointState, queue_size=10)
        self._subscriber = rospy.Subscriber(
            input_topic, JointState, self._command_callback, queue_size=1
        )
        self._timer = rospy.Timer(rospy.Duration(1.0 / self._rate), self._publish)

        rospy.loginfo(
            "UR5e visualization bridge: %s -> %s at %.1f Hz",
            input_topic,
            output_topic,
            self._rate,
        )

    def _command_callback(self, message):
        if not message.position:
            rospy.logwarn_throttle(2.0, "Ignoring JointState without positions")
            return

        try:
            if message.name:
                positions = self._ordered_values(message.name, message.position, "position")
                velocities = self._optional_ordered_values(
                    message.name, message.velocity, "velocity"
                )
                efforts = self._optional_ordered_values(message.name, message.effort, "effort")
            else:
                if len(message.position) != len(self._joint_names):
                    raise ValueError("unnamed command must contain exactly six positions")
                positions = list(message.position)
                velocities = self._optional_sequential_values(message.velocity, "velocity")
                efforts = self._optional_sequential_values(message.effort, "effort")

            if not all(math.isfinite(value) for value in positions):
                raise ValueError("positions contain NaN or infinity")
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "Ignoring invalid joint command: %s", error)
            return

        with self._lock:
            self._positions = positions
            self._velocities = velocities
            self._efforts = efforts
            self._last_command_time = rospy.Time.now()
            self._timeout_reported = False

    def _ordered_values(self, names, values, field):
        if len(names) != len(values):
            raise ValueError("name and %s arrays have different lengths" % field)
        lookup = dict(zip(names, values))
        missing = [name for name in self._joint_names if name not in lookup]
        if missing:
            raise ValueError("missing joints: %s" % ", ".join(missing))
        return [lookup[name] for name in self._joint_names]

    def _optional_ordered_values(self, names, values, field):
        if not values:
            return []
        return self._ordered_values(names, values, field)

    def _optional_sequential_values(self, values, field):
        if not values:
            return []
        if len(values) != len(self._joint_names):
            raise ValueError("%s must be empty or contain exactly six values" % field)
        return list(values)

    def _publish(self, _event):
        now = rospy.Time.now()
        with self._lock:
            positions = list(self._positions)
            velocities = list(self._velocities)
            efforts = list(self._efforts)
            last_command_time = self._last_command_time

            if (
                self._timeout > 0.0
                and last_command_time is not None
                and (now - last_command_time).to_sec() > self._timeout
                and not self._timeout_reported
            ):
                rospy.logwarn("Joint command timed out; holding the last pose")
                self._timeout_reported = True

        output = JointState()
        output.header.stamp = now
        output.name = list(self._joint_names)
        output.position = positions
        output.velocity = velocities
        output.effort = efforts
        self._publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("ur5e_joint_state_bridge")
    JointStateVisualizer()
    rospy.spin()

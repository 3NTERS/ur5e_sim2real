#include "ur5e_rl_control/trajectory_sender.h"

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <boost/bind.hpp>

#include <algorithm>
#include <cmath>

namespace ur5e_rl_control {

TrajectorySender::TrajectorySender(
    const std::string& action_name,
    const std::vector<std::string>& joint_names,
    const double start_delay)
    : client_(action_name, true),
      joint_names_(joint_names),
      start_delay_(std::max(0.0, start_delay)) {}

bool TrajectorySender::waitForServer(const ros::Duration& timeout) {
  return client_.waitForServer(timeout);
}

bool TrajectorySender::isServerConnected() {
  return client_.isServerConnected();
}

bool TrajectorySender::sendPositions(
    const std::vector<double>& positions,
    const ros::Duration& movement_duration) {
  if (positions.size() != joint_names_.size()) {
    ROS_ERROR("Trajectory has %zu positions, expected %zu", positions.size(),
              joint_names_.size());
    return false;
  }
  if (movement_duration.toSec() <= 0.0) {
    ROS_ERROR("Trajectory duration must be greater than zero");
    return false;
  }
  if (!std::all_of(positions.begin(), positions.end(),
                   [](double value) { return std::isfinite(value); })) {
    ROS_ERROR("Trajectory contains NaN or infinity");
    return false;
  }
  if (!client_.isServerConnected()) {
    ROS_ERROR_THROTTLE(2.0, "FollowJointTrajectory action server is unavailable");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(start_delay_);
  goal.trajectory.joint_names = joint_names_;

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = positions;
  point.velocities.assign(joint_names_.size(), 0.0);
  point.time_from_start = movement_duration;
  goal.trajectory.points.push_back(point);
  goal.goal_time_tolerance = ros::Duration(0.25);

  client_.sendGoal(
      goal,
      boost::bind(&TrajectorySender::doneCallback, this, _1, _2));
  return true;
}

void TrajectorySender::cancel() {
  client_.cancelAllGoals();
}

void TrajectorySender::doneCallback(
    const actionlib::SimpleClientGoalState& state,
    const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_DEBUG("UR5e trajectory completed");
    return;
  }
  if (state == actionlib::SimpleClientGoalState::PREEMPTED ||
      state == actionlib::SimpleClientGoalState::RECALLED) {
    ROS_DEBUG("UR5e trajectory was replaced by a newer command");
    return;
  }

  const int error_code = result ? result->error_code : 0;
  const std::string error_string = result ? result->error_string : "no result";
  ROS_WARN("UR5e trajectory ended with state %s (code %d): %s",
           state.toString().c_str(), error_code, error_string.c_str());
}

}  // namespace ur5e_rl_control

#include "ur5e_rl_control/ur5e_execution_node.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace ur5e_rl_control {

UR5eExecutionNode::UR5eExecutionNode(ros::NodeHandle nh,
                                     ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh)) {
  if (!readAndValidateParameters()) {
    throw std::runtime_error("Invalid UR5e execution parameters");
  }

  std::string rl_action_topic;
  std::string joint_state_topic;
  std::string visualization_topic;
  std::string controller_action;
  double controller_wait_timeout;
  double trajectory_start_delay;
  private_nh_.param<std::string>("rl_action_topic", rl_action_topic,
                                 "/rl_action");
  private_nh_.param<std::string>("joint_state_topic", joint_state_topic,
                                 "/joint_states");
  private_nh_.param<std::string>("visualization_topic", visualization_topic,
                                 "/ur5e/joint_command");
  private_nh_.param<std::string>(
      "controller_action", controller_action,
      "/scaled_pos_joint_traj_controller/follow_joint_trajectory");
  private_nh_.param("controller_wait_timeout", controller_wait_timeout, 5.0);
  private_nh_.param("trajectory_start_delay", trajectory_start_delay, 0.05);

  visualization_publisher_ =
      nh_.advertise<sensor_msgs::JointState>(visualization_topic, 10);
  action_subscriber_ = nh_.subscribe(rl_action_topic, 1,
                                     &UR5eExecutionNode::actionCallback, this);
  joint_state_subscriber_ =
      nh_.subscribe(joint_state_topic, 1,
                    &UR5eExecutionNode::jointStateCallback, this);

  if (send_to_controller_) {
    trajectory_sender_.reset(new TrajectorySender(
        controller_action, joint_names_, trajectory_start_delay));
    ROS_INFO("Waiting %.1f seconds for controller action %s",
             controller_wait_timeout, controller_action.c_str());
    if (!trajectory_sender_->waitForServer(
            ros::Duration(controller_wait_timeout))) {
      ROS_WARN("Controller action is not available yet; incoming actions will "
               "still be shown in RViz");
    }
  }

  ROS_INFO("UR5e execution node listening on %s (%s mode, scale %.3f)",
           rl_action_topic.c_str(), action_mode_.c_str(), action_scale_);
  ROS_INFO("Controller output is %s",
           send_to_controller_ ? "ENABLED" : "DISABLED (RViz only)");
}

bool UR5eExecutionNode::readAndValidateParameters() {
  if (!private_nh_.getParam("joint_names", joint_names_)) {
    ROS_ERROR("Missing required parameter: joint_names");
    return false;
  }
  private_nh_.param<std::string>("action_mode", action_mode_, "position");
  private_nh_.param("action_scale", action_scale_, 1.0);
  private_nh_.param("trajectory_duration", trajectory_duration_, 0.2);
  private_nh_.param("clamp_to_limits", clamp_to_limits_, true);
  private_nh_.param("send_to_controller", send_to_controller_, false);

  private_nh_.getParam("initial_positions", current_positions_);
  private_nh_.getParam("lower_limits", lower_limits_);
  private_nh_.getParam("upper_limits", upper_limits_);

  if (joint_names_.size() != 6) {
    ROS_ERROR("joint_names must contain exactly six names");
    return false;
  }
  if (current_positions_.size() != joint_names_.size()) {
    ROS_ERROR("initial_positions must contain exactly six values");
    return false;
  }
  if (action_mode_ != "position" && action_mode_ != "delta") {
    ROS_ERROR("action_mode must be 'position' or 'delta'");
    return false;
  }
  if (!std::isfinite(action_scale_) || trajectory_duration_ <= 0.0) {
    ROS_ERROR("action_scale must be finite and trajectory_duration positive");
    return false;
  }
  if (clamp_to_limits_ &&
      (lower_limits_.size() != joint_names_.size() ||
       upper_limits_.size() != joint_names_.size())) {
    ROS_ERROR("lower_limits and upper_limits must contain six values");
    return false;
  }
  for (std::size_t i = 0; clamp_to_limits_ && i < joint_names_.size(); ++i) {
    if (lower_limits_[i] > upper_limits_[i]) {
      ROS_ERROR("Lower limit exceeds upper limit for %s",
                joint_names_[i].c_str());
      return false;
    }
  }
  return true;
}

void UR5eExecutionNode::actionCallback(
    const std_msgs::Float64MultiArrayConstPtr& message) {
  std::vector<double> target;
  if (!makeTarget(message->data, &target)) {
    return;
  }

  publishVisualization(target);
  current_positions_ = target;

  if (send_to_controller_ && trajectory_sender_) {
    trajectory_sender_->sendPositions(target,
                                      ros::Duration(trajectory_duration_));
  }
}

bool UR5eExecutionNode::makeTarget(const std::vector<double>& action,
                                   std::vector<double>* target) const {
  if (action.size() != joint_names_.size()) {
    ROS_WARN_THROTTLE(2.0, "Ignoring RL action with %zu values; expected six",
                      action.size());
    return false;
  }
  if (!std::all_of(action.begin(), action.end(),
                   [](double value) { return std::isfinite(value); })) {
    ROS_WARN_THROTTLE(2.0, "Ignoring RL action containing NaN or infinity");
    return false;
  }

  target->resize(joint_names_.size());
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const double scaled_action = action[i] * action_scale_;
    (*target)[i] = action_mode_ == "delta"
                       ? current_positions_[i] + scaled_action
                       : scaled_action;
    if (clamp_to_limits_) {
      const double unclamped = (*target)[i];
      (*target)[i] =
          std::max(lower_limits_[i], std::min(upper_limits_[i], unclamped));
      if ((*target)[i] != unclamped) {
        ROS_WARN_THROTTLE(2.0, "Clamped %s to its configured joint limit",
                          joint_names_[i].c_str());
      }
    }
  }
  return true;
}

void UR5eExecutionNode::jointStateCallback(
    const sensor_msgs::JointStateConstPtr& message) {
  if (message->name.size() != message->position.size()) {
    return;
  }
  std::unordered_map<std::string, double> positions;
  for (std::size_t i = 0; i < message->name.size(); ++i) {
    positions[message->name[i]] = message->position[i];
  }
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const auto found = positions.find(joint_names_[i]);
    if (found == positions.end()) {
      return;
    }
    current_positions_[i] = found->second;
  }
}

void UR5eExecutionNode::publishVisualization(
    const std::vector<double>& target) {
  sensor_msgs::JointState message;
  message.header.stamp = ros::Time::now();
  message.name = joint_names_;
  message.position = target;
  visualization_publisher_.publish(message);
}

}  // namespace ur5e_rl_control

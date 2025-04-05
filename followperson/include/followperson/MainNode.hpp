// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MAIN_NODE_HPP_
#define MAIN_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "followperson/PerceptionNode.hpp"
#include "followperson/ControlNode.hpp"

namespace followperson
{

class MainNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MainNode)

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  MainNode();
  void controlNodeStateCallback(const lifecycle_msgs::msg::State::SharedPtr msg);
  void checkPersonDetection();

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state);

private:
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr control_activation_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr perception_activation_client_;

  std::shared_ptr<PerceptionNode> perception_node_;
  std::shared_ptr<ControlNode> control_node_;
  rclcpp::Subscription<lifecycle_msgs::msg::State>::SharedPtr control_node_state_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace followperson

#endif // MAIN_NODE_HPP_

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

#include "followperson/MainNode.hpp"

using namespace std::chrono_literals;

namespace followperson
{

MainNode::MainNode() // Constructor definition for MainNode class
: rclcpp_lifecycle::LifecycleNode("main_node") // Initialize the node with the name "main_node"
{
  perception_node_ = std::make_shared<PerceptionNode>(); // Create an instance of PerceptionNode and store it in a shared pointer
  control_node_ = std::make_shared<ControlNode>(); // Create an instance of ControlNode and store it in a shared pointer

  timer_ = create_wall_timer(
    // Create a timer that invokes the checkPersonDetection method
    100ms, std::bind(&MainNode::checkPersonDetection, this));

  control_node_state_subscription_ = this->create_subscription<lifecycle_msgs::msg::State>(
    // Subscribe to changes in the state of the control node
    "control_node",
    10,
    std::bind(&MainNode::controlNodeStateCallback, this, std::placeholders::_1));
    
  control_activation_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>("control_node/change_state");
}

void MainNode::controlNodeStateCallback(const lifecycle_msgs::msg::State::SharedPtr msg)
{
  if (msg->label == "inactive" && perception_node_->isPersonDetected()) {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    control_activation_client_->async_send_request(request);
  }
}

void MainNode::checkPersonDetection()
{
  if (control_node_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
      !perception_node_->isPersonDetected())
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    control_activation_client_->async_send_request(request);
  }
}

// Lifecycle methods
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MainNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Configuring...");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MainNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Activating...");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MainNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MainNode::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up...");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MainNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down...");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MainNode::on_error(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Error State");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace followperson

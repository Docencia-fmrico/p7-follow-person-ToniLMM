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

#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // Necesario para convertir transformaciones a mensajes
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <cmath>

namespace followperson
{

class ControlNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  ControlNode();
  const rclcpp_lifecycle::State get_current_state() const;

private:
  void controlLoop();
  double calculateError(const geometry_msgs::msg::TransformStamped & person_tf);
  geometry_msgs::msg::Twist calculateControlCommand(
    double error,
    const geometry_msgs::msg::TransformStamped & person_tf);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  // PID parameters
  double kp_;
  double ki_;
  double kd_;
  double prev_error_;
  double integral_;
};

} // namespace followperson

#endif // CONTROL_NODE_HPP_

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


#include "followperson/ControlNode.hpp"

using namespace std::chrono_literals;

namespace followperson
{

ControlNode::ControlNode()
: rclcpp_lifecycle::LifecycleNode("control_node"),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  kp_ = 0.2;
  ki_ = 0.01;
  kd_ = 0.1;
  prev_error_ = 0.0;
  integral_ = 0.0;

  timer_ = create_wall_timer(
    100ms, std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::controlLoop()
{
  try {
    auto person_tf = tf_buffer_->lookupTransform("odom", "person", tf2::TimePointZero);

    RCLCPP_INFO(this->get_logger(), "Person found at: x=%f, y=%f, z=%f",
      person_tf.transform.translation.x,
      person_tf.transform.translation.y,
      person_tf.transform.translation.z);

    double error = calculateError(person_tf);

    auto control_command = calculateControlCommand(error, person_tf);

    cmd_publisher_->publish(control_command);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Error getting person transform: %s", ex.what());
    geometry_msgs::msg::Twist search_command;
    search_command.angular.z = 0.3;
    cmd_publisher_->publish(search_command);
  }
}

double ControlNode::calculateError(const geometry_msgs::msg::TransformStamped & person_tf)
{
  double distance = std::sqrt(
    std::pow(person_tf.transform.translation.x, 2) +
    std::pow(person_tf.transform.translation.y, 2)
  );
  return distance - 1.0;
}

geometry_msgs::msg::Twist ControlNode::calculateControlCommand(
  double error,
  const geometry_msgs::msg::TransformStamped & person_tf)
{
  integral_ += error;
  double derivative = error - prev_error_;

  geometry_msgs::msg::Twist control_command;
  control_command.linear.x = kp_ * error + ki_ * integral_ + kd_ * derivative;

  // Limitar velocidad lineal (por ejemplo, entre 0.0 y 0.6 m/s)
  if (control_command.linear.x > 0.6) {control_command.linear.x = 0.6;}
  if (control_command.linear.x < 0.0) {control_command.linear.x = 0.0;}

  // Ángulo hacia la persona
  double angle = std::atan2(person_tf.transform.translation.y, person_tf.transform.translation.x);

  // Limitar velocidad angular (por ejemplo, entre -0.5 y 0.5 rad/s)
  control_command.angular.z = std::clamp(angle, -0.5, 0.5);

  prev_error_ = error;
  return control_command;
}

const rclcpp_lifecycle::State ControlNode::get_current_state() const
{
  return rclcpp_lifecycle::LifecycleNode::get_current_state();
}

} // namespace followperson

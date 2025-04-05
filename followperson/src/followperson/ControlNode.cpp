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

ControlNode::ControlNode() // Constructor definition for ControlNode class
: rclcpp_lifecycle::LifecycleNode("control_node"), // Initialize the node with the name "control_node"
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())), // Create a buffer for transforming coordinates
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) // Create a listener for TF transformations
{
  cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // Create a publisher for Twist messages

  // Set PID parameters
  kp_ = 0.41;
  ki_ = 0.06;
  kd_ = 0.53;
  prev_error_ = 0.0;
  integral_ = 0.0;

  timer_ = create_wall_timer(
    // Create a timer that invokes the controlLoop method
    100ms, std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::controlLoop() // Definition of the control loop method
{
  try {
    auto person_tf = tf_buffer_->lookupTransform("odom", "person", tf2::TimePointZero); // Look up the transformation from base_link to person frame

    double error = calculateError(person_tf); // Calculate the error based on the person's position

    auto control_command = calculateControlCommand(error, person_tf); // Calculate the control command based on the error and person's position

    cmd_publisher_->publish(control_command); // Publish the control command

  } catch (const tf2::TransformException & ex) { // Handle exceptions related to TF transformations
    RCLCPP_ERROR(this->get_logger(), "Error getting person transform: %s", ex.what()); // Log the error message
    // Handle the case when the person is lost or not detected
    geometry_msgs::msg::Twist search_command; // Create a Twist message for searching
    search_command.angular.z = 0.3; // Set the angular velocity to make the robot turn in place
    cmd_publisher_->publish(search_command); // Publish the search command
  }
}

double ControlNode::calculateError(const geometry_msgs::msg::TransformStamped & person_tf) // Definition of the error calculation method
{
  double distance = std::sqrt(
    // Calculate the distance from the robot to the person
    std::pow(person_tf.transform.translation.x, 2) +
    std::pow(person_tf.transform.translation.y, 2)
  );
  return distance - 1.0; // Return the error, which is the deviation from the desired distance
}

geometry_msgs::msg::Twist ControlNode::calculateControlCommand(
  double error,
  const geometry_msgs::msg::TransformStamped & person_tf) // Definition of the control command calculation method
{
  integral_ += error; // Update the integral term of the PID controller
  double derivative = error - prev_error_; // Calculate the derivative term of the PID controller
  geometry_msgs::msg::Twist control_command; // Create a Twist message for the control command
  control_command.linear.x = kp_ * error + ki_ * integral_ + kd_ * derivative; // Calculate the linear velocity based on PID control

  double angle = std::atan2(person_tf.transform.translation.y, person_tf.transform.translation.x); // Calculate the angle to turn towards the person
  control_command.angular.z = angle; // Set the angular velocity to turn towards the person

  prev_error_ = error; // Update the previous error for the next iteration
  return control_command; // Return the control command
}

const rclcpp_lifecycle::State ControlNode::get_current_state() const // Definition of the method to get the current state
{
  return this->get_current_state(); // Return the current state of the node
}

} // namespace followperson

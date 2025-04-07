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

#ifndef FOLLOWPERSON__PERCEPTIONNODE_HPP_
#define FOLLOWPERSON__PERCEPTIONNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/bool.hpp"

namespace followperson
{

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode();

  // Function to check if a person is detected
  bool isPersonDetected() const;

private:
  void detectionCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);

  rclcpp::Logger logger_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detection_state_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool is_person_detected_; // Flag to track if a person is detected
};

} // namespace followperson

#endif // FOLLOWPERSON__PERCEPTIONNODE_HPP_

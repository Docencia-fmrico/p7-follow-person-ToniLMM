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


#ifndef FOLLOWPERSON__DETECTION3DTOTFNODE_HPP_
#define FOLLOWPERSON__DETECTION3DTOTFNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace followperson
{

class Detection3DToTFNode : public rclcpp::Node
{
public:
  Detection3DToTFNode();

private:
  void detectionCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace followperson

#endif  // FOLLOWPERSON__DETECTION3DTOTFNODE_HPP_

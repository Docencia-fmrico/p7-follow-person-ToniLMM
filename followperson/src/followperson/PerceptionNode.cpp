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

#include "followperson/PerceptionNode.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "yolo_msgs/msg/detection_array.hpp"

namespace followperson
{

PerceptionNode::PerceptionNode()
: Node("perception_node"),
  is_person_detected_(false),
  logger_(rclcpp::get_logger("PerceptionNode")),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock()))
{
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); // Create a TF listener with the TF buffer
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this); // Create a TF broadcaster passing this node to the constructor

  detection_subscription_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    // Create a subscription to the detection topic
    "output_detection_3d",
    10,
    std::bind(&PerceptionNode::detectionCallback, this, std::placeholders::_1)); // Set the callback function for incoming detection messages

    detection_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("person_detected", 10);
}

void PerceptionNode::detectionCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  bool found_person = false;

  for (const auto & detection : msg->detections) {
    for (const auto & result : detection.results) {
      if (result.hypothesis.class_id != "person") continue;

      geometry_msgs::msg::TransformStamped odom2person_tf;
      odom2person_tf.header = detection.header;
      odom2person_tf.child_frame_id = "person";
      odom2person_tf.transform.translation.x = detection.bbox.center.position.x;
      odom2person_tf.transform.translation.y = detection.bbox.center.position.y;
      odom2person_tf.transform.translation.z = detection.bbox.center.position.z;


      tf2::Quaternion quaternion;
      quaternion.setRPY(0, 0, atan2(
        detection.bbox.center.position.y,
        detection.bbox.center.position.x));

      odom2person_tf.transform.rotation.x = quaternion.getX();
      odom2person_tf.transform.rotation.y = quaternion.getY();
      odom2person_tf.transform.rotation.z = quaternion.getZ();
      odom2person_tf.transform.rotation.w = quaternion.getW();

      tf_broadcaster_->sendTransform(odom2person_tf);

      found_person = true;
    }
  }

  is_person_detected_ = found_person;

  std_msgs::msg::Bool detection_msg;
  detection_msg.data = found_person;
  detection_state_pub_->publish(detection_msg);

  RCLCPP_DEBUG(logger_, "Person detected: %s", found_person ? "true" : "false");
}

bool PerceptionNode::isPersonDetected() const // Definition of the method to check if a person is detected
{
  return is_person_detected_; // Return the flag indicating if a person is detected
}

}  // namespace followperson

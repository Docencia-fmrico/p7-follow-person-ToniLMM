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
#include "yolov8_msgs/msg/detection_array.hpp"

namespace followperson
{

PerceptionNode::PerceptionNode() // Constructor definition for PerceptionNode class
: Node("perception_node"), tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())), // Initialize the node with the name "perception_node" and create a TF buffer
  is_person_detected_(false) // Initialize the flag for person detection to false
{
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); // Create a TF listener with the TF buffer
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this); // Create a TF broadcaster passing this node to the constructor

  detection_subscription_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    // Create a subscription to the detection topic
    "output_detection_3d",
    10,
    std::bind(&PerceptionNode::detectionCallback, this, std::placeholders::_1)); // Set the callback function for incoming detection messages
}

void PerceptionNode::detectionCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg) // Definition of the detection callback method
{
  bool found_person = false; // Initialize a flag to track if a person is found in the detection results

  for (const auto & detection : msg->detections) { // Iterate through each detection in the message
    for (const auto & result : detection.results) { // Iterate through each result in the detection
      // Assuming detection provides position in the frame of the robot
      geometry_msgs::msg::TransformStamped odom2person_tf; // Create a TransformStamped message for the person's position in the robot's frame
      odom2person_tf.header = detection.header; // Set the header of the TF message
      odom2person_tf.child_frame_id = "person"; // Set the child frame ID as "person"
      odom2person_tf.transform.translation.x = detection.bbox.center.position.x; // Set the translation in x-axis
      odom2person_tf.transform.translation.y = detection.bbox.center.position.y; // Set the translation in y-axis
      odom2person_tf.transform.translation.z = detection.bbox.center.position.z; // Set the translation in z-axis

      // Calculate orientation of the robot towards the person
      tf2::Quaternion quaternion; // Create a quaternion for orientation
      quaternion.setRPY(
        // Set the quaternion based on the yaw angle towards the person
        0, 0,
        atan2(detection.bbox.center.position.y, detection.bbox.center.position.x));
      odom2person_tf.transform.rotation.x = quaternion.getX(); // Set the quaternion components
      odom2person_tf.transform.rotation.y = quaternion.getY();
      odom2person_tf.transform.rotation.z = quaternion.getZ();
      odom2person_tf.transform.rotation.w = quaternion.getW();

      // Publish the transformed TF
      tf_broadcaster_->sendTransform(odom2person_tf); // Publish the TF for the person's position

      // Set found_person flag if object detected is "person"
      if (result.hypothesis.class_id == "person") { // Check if the detected object is a person
        found_person = true; // Set the flag to true
        is_person_detected_ = true; // Set the flag for person detection to true
      }
    }
  }

  // If person is not detected, reset the flag
  if (!found_person) { // If no person is found in the detections
    is_person_detected_ = false; // Reset the flag for person detection
  }
}

bool PerceptionNode::isPersonDetected() const // Definition of the method to check if a person is detected
{
  return is_person_detected_; // Return the flag indicating if a person is detected
}

}  // namespace followperson

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


#include "followperson/Detection3DToTFNode.hpp"

namespace followperson
{

using std::placeholders::_1;

Detection3DToTFNode::Detection3DToTFNode()
: Node("detection3d_to_tf_node")
{
  detection_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    "detections_3d", 10, std::bind(&Detection3DToTFNode::detectionCallback, this, _1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void Detection3DToTFNode::detectionCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  for (size_t i = 0; i < msg->detections.size(); ++i) {
    const auto & detection = msg->detections[i];

    if (std::isnan(detection.bbox.center.position.z)) {
      continue;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "camera_link";
    transform.child_frame_id = "person_" + std::to_string(i);

    transform.transform.translation.x = detection.bbox.center.position.x;
    transform.transform.translation.y = detection.bbox.center.position.y;
    transform.transform.translation.z = detection.bbox.center.position.z;

    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transform);
  }
}

}  // namespace followperson

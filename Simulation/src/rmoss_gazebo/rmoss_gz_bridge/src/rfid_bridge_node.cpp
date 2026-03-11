// Copyright 2021 RoboMaster-OSS
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

#include "rmoss_gz_bridge/rfid_bridge_node.hpp"

#include <rmoss_interfaces/msg/detail/rfid_status_array__struct.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>

#include <thread>
#include <memory>
#include <string>

#include "ros_gz_bridge/convert.hpp"

namespace rmoss_gz_bridge
{

RfidBridgeNode::RfidBridgeNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("rfid_bridge", options);
  gz_node_ = std::make_shared<ignition::transport::Node>();
  // parameters
  std::string world_name;
  node_->declare_parameter("world_name", "default");
  node_->declare_parameter("robot_filter", false);
  node_->get_parameter("world_name", world_name);
  std::string gz_blue_supplier_rfid_topic = "/model/blue_supplier/performer_detector/status";
  std::string gz_red_supplier_rfid_topic = "/model/red_supplier/performer_detector/status";
  gz_red_supplier_service_name_ = "/world/" + world_name + "/set_rfid_status";
  gz_blue_supplier_service_name_ = "/world/" + world_name + "/set_rfid_status";
  // get rfid status from ignition gazebo
  gz_node_->Subscribe(gz_blue_supplier_rfid_topic, &RfidBridgeNode::gz_rfid_cb, this);
  gz_node_->Subscribe(gz_red_supplier_rfid_topic, &RfidBridgeNode::gz_rfid_cb, this);

  rfid_pub_ = node_->create_publisher<rmoss_interfaces::msg::RfidStatusArray>(
    "/referee_system/rfid_info", 10);
}

void RfidBridgeNode::gz_rfid_cb(const ignition::msgs::Pose & msg)
{
  std::string robot_name = msg.name().data();
  for (auto const & p : msg.header().data()) {
    if (p.key() == "frame_id") {
      if (p.value().size() > 0) {
        std::string value = p.value().Get(0);
        rmoss_interfaces::msg::RfidStatus rfid_status;
        if (value.find("red_supplier") != std::string::npos) {
          rfid_status.robot_name = robot_name;
          rfid_status.supplier_area_is_triggered = true;
          rfid_status_array.robot_rfid_status.push_back(rfid_status);
          break;
        }
      }
    }
  }
  if (!rfid_status_array.robot_rfid_status.empty()) {
    rfid_pub_->publish(rfid_status_array);
    rfid_status_array.robot_rfid_status.clear();
  }
}

}  // namespace rmoss_gz_bridge
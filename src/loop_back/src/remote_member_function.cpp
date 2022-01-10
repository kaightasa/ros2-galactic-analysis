// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "time_interface/msg/timestamp.hpp"

using std::placeholders::_1;

class LoopbackNode : public rclcpp::Node
{
public:
  LoopbackNode()
  : Node("loopback_node")
  {
    publisher_ = this->create_publisher<time_interface::msg::Timestamp>("topic2", 10);
    subscription_ = this->create_subscription<time_interface::msg::Timestamp>(
      "topic1", 10, std::bind(&LoopbackNode::topic_callback, this, _1));
  }

private:
  void topic_callback(const time_interface::msg::Timestamp::SharedPtr msg) const
  {
    auto message = std::move(msg);
    publisher_->publish(*message);
  }
  rclcpp::Publisher<time_interface::msg::Timestamp>::SharedPtr publisher_;
  rclcpp::Subscription<time_interface::msg::Timestamp>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoopbackNode>());
  rclcpp::shutdown();
  return 0;
}

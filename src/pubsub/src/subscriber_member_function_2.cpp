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
//#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    //subscription_ = this->create_subscription<std_msgs::msg::String>(
    subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "topic2", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  //void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  void topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr array) const
  {
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", "array");
    int sum = 0;
    for (int i = 0; i < 250; i++) {
      sum += array->data[i];
    }
    //RCLCPP_INFO(this->get_logger(), "sum is: '%d'", sum);*/
      
  }
  //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

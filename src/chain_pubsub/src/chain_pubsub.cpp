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

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

 #define QOS_HISTORY_SIZE 10
 #define MESSAGE_ARRAY_SIZE 250;
 #define INTER_NODE_NUM 5

using namespace std::chrono_literals;

using std::placeholders::_1;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(
      std::string pub_topic_name)
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(pub_topic_name, QOS_HISTORY_SIZE);
    timer_ = this->create_wall_timer(
      34ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    std_msgs::msg::Int32MultiArray array;
    array.data.clear();
    int array_size = MESSAGE_ARRAY_SIZE;
    array.data.resize(array_size);
    for (int i = 0; i < array_size; i++) {
      array.data[i]  = i;
    }
    publisher_->publish(array);

  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  size_t count_;
};

class InterNode : public rclcpp::Node
{
public:
  InterNode(
      std::string node_name, std::string sub_topic_name, std::string pub_topic_name)
  : Node(node_name)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(pub_topic_name, QOS_HISTORY_SIZE);
    subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      sub_topic_name, QOS_HISTORY_SIZE, std::bind(&InterNode::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const
  {
    auto array = std::move(msg);
    publisher_->publish(*array);
  }
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(
      std::string sub_topic_name)
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      sub_topic_name, QOS_HISTORY_SIZE, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const
  {
    int sum = 0;
    int array_size = MESSAGE_ARRAY_SIZE;
    for (int i = 0; i < array_size; i++) {
      sum += msg->data[i];
    }
    //RCLCPP_INFO(this->get_logger(), "sum is: '%d'", sum);
  }
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::vector<std::shared_ptr<rclcpp::Node>> nodes;

  nodes.emplace_back(std::make_shared<MinimalPublisher>("/topic1"));
  std::string pub_topic_name;
  std::string sub_topic_name;
  for (int i = 0; i < INTER_NODE_NUM; i++) {
    std::string node_name = "inter_node" + std::to_string(i);
    sub_topic_name = "/topic" + std::to_string(i+1);
    pub_topic_name = "/topic" + std::to_string(i+2);
    nodes.emplace_back(
      std::make_shared<InterNode>(node_name, sub_topic_name, pub_topic_name)
    );
  }
  nodes.emplace_back(std::make_shared<MinimalSubscriber>(pub_topic_name));


  for (auto & node : nodes) {
    executor->add_node(node);
  }

  executor->spin();
  rclcpp::shutdown();

  return 0;
}

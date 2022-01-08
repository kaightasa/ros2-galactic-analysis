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
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "time_interface/msg/timestamp.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<time_interface::msg::Timestamp>("topic1", 10);
    timer_ = this->create_wall_timer(
      34ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = time_interface::msg::Timestamp();
    int array_size = 25000;
    for (int i = 0; i < array_size; i++) {
      message.array.push_back(i);
    }
    timespec ts;
    timespec_get(&ts, TIME_UTC);
    message.stime = static_cast<int64_t>(ts.tv_sec);
    message.ntime = static_cast<int64_t>(ts.tv_nsec);
    publisher_->publish(message);

  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<time_interface::msg::Timestamp>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

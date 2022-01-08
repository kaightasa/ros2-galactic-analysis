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
#include <ctime>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "time_interface/msg/timestamp.hpp"

using std::placeholders::_1;

struct timespec end;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<time_interface::msg::Timestamp>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const time_interface::msg::Timestamp::SharedPtr msg) const
  {
    timespec_get(&end, TIME_UTC);
    time_t start_sec = msg->stime;
    time_t start_nsec = static_cast<long>(msg->ntime);

    long nsec;
    int64_t sec;
    if (start_nsec < end.tv_nsec) {
      nsec = end.tv_nsec - start_nsec;
      sec = end.tv_sec - start_sec;
    } else {
      nsec = 1000*1000*1000 + end.tv_nsec - start_nsec;
      sec = end.tv_sec - start_sec - 1;
    }
    //RCLCPP_INFO(this->get_logger(), "%ld.%ld\n", sec, nsec);
    std::ofstream outputfile("/home/bluebox/ros2-galactic-analysis/result/remote_pubsub/result.txt", std::ios::app);
    outputfile << sec*1000*1000*1000 + nsec << std::endl;
    outputfile.close();
  }
  rclcpp::Subscription<time_interface::msg::Timestamp>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

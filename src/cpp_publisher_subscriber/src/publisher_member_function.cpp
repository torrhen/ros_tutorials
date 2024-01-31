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
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0) {
    // shared pointer to new publisher on /chatter topic
    publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("chatter", 10);
    // create a timer based on wall clock (since start of program) to control callbacks
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    // set the data within the message
    auto message = tutorial_interfaces::msg::Num();
    message.num = this->count_++;
    // console output
    RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.num);
    // publish message on /topic
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  // intialize ROS2 program and C++ client library
  rclcpp::init(argc, argv);
  // block the main thread to process callbacks between publishers and subscribers on /chatter topic (until interrupt or ros::is_ok())
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  // stop ROS2 given a user interrupt with ctrl+c
  rclcpp::shutdown();
  return 0;
}

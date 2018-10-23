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
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher()
  : Node("image_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw");
    timer_ = this->create_wall_timer(
      500ms, std::bind(&ImagePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = sensor_msgs::msg::Image();
    // message.header.stamp = TBD
    msg.header.frame_id = frame_id_;
    msg.width = 100;
    msg.height = 100;
    msg.encoding = "mono8";
    msg.step = msg.width;
    msg.data.resize(msg.width * msg.height);
    publisher_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  // TODO(lucasw) use topic or parameter to set this
  std::string frame_id_ = "map";
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}

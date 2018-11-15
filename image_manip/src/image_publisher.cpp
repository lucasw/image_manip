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
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher(const std::string image_name="")
  : Node("image_publisher"), image_name_(image_name)
  {
    cv_bridge::CvImage cvi;
#if 0
    image_ = cv::imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
    if (image_.empty()) {
      image_ = cv::Mat(100, 100, CV_8UC1);
    }
    cvi.encoding = sensor_msgs::image_encodings::MONO8;
#else
    image_ = cv::imread(image_name, CV_LOAD_IMAGE_COLOR);
    if (image_.empty()) {
      image_ = cv::Mat(100, 100, CV_8UC3);
    }
    cvi.encoding = sensor_msgs::image_encodings::BGR8;

#endif
    cvi.image = image_;
    msg_ = cvi.toImageMsg();
    msg_->header.frame_id = frame_id_;

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw");
    timer_ = this->create_wall_timer(
      500ms, std::bind(&ImagePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // message.header.stamp = TBD
    publisher_->publish(msg_);
  }

  std::string image_name_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  cv::Mat image_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  // TODO(lucasw) use topic or parameter to set this
  std::string frame_id_ = "map";
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  for (int i = 0; i < argc; ++i) {
    std::cout << argv[i] << "\n";
  }
  std::string image_name;
  if (argc > 1) {
    image_name = argv[1];
    std::cout << "using image " << image_name << "\n";
    // RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic.c_str());
  }
  rclcpp::spin(std::make_shared<ImagePublisher>(image_name));
  rclcpp::shutdown();
  return 0;
}

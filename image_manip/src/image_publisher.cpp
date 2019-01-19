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
  ImagePublisher() : Node("image_publisher")
  {
    cv_bridge::CvImage cvi;

    get_parameter_or("image_name", image_name_, image_name_);
    set_parameter_if_not_set("image_name", image_name_);
#if 0
    image_ = cv::imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
    if (image_.empty()) {
      image_ = cv::Mat(100, 100, CV_8UC1);
    }
    cvi.encoding = sensor_msgs::image_encodings::MONO8;
#else
    image_ = cv::imread(image_name_, CV_LOAD_IMAGE_COLOR);
    if (image_.empty()) {
      RCLCPP_INFO(get_logger(), "using blank image because imread failed '%s'",
          image_name_.c_str());
      image_ = cv::Mat(500, 500, CV_8UC3);
    } else {
      RCLCPP_INFO(get_logger(), "Loaded image '%s' %d x %d",
          image_name_.c_str(), image_.rows, image_.cols);
    }
    cvi.encoding = sensor_msgs::image_encodings::BGR8;

#endif
    cvi.image = image_;
    msg_ = cvi.toImageMsg();
    msg_->header.frame_id = frame_id_;

    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    int depth = 5;
    get_parameter_or("qos_depth", depth, depth);
    set_parameter_if_not_set("qos_depth", depth);
    qos.depth = depth;
    // qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", qos);

    double rate = 1.0;
    get_parameter_or("rate", rate, rate);
    set_parameter_if_not_set("rate", rate);

    int period_ms = 1000;
    if (rate > 0.0) {
      period_ms = 1000.0 / rate;
    }
    RCLCPP_INFO(get_logger(), "update rate %f %d", rate, period_ms);

        // rmw_qos_profile_sensor_data);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&ImagePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // message.header.stamp = TBD
    auto t0 = now();
    msg_->header.stamp = t0;
    publisher_->publish(msg_);
    // std::cout << (now() - t0).nanoseconds() / 1e9 << "\n";
  }

  std::string image_name_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  // sensor_msgs::msg::Image::UniquePtr msg_;
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
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}

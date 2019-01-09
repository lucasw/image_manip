/**
 Copyright 2015 Lucas Walter

     This file is part of Vimjay.

    Vimjay is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Vimjay is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Vimjay.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
using std::placeholders::_1;

class Color : public rclcpp::Node
{
protected:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  void pubImage();

  int width_ = 32;
  int height_ = 32;
  int red_ = 255;
  int green_ = 255;
  int blue_= 255;

  double frame_rate_ = 1.0;
  void updateTimer();

  bool dirty_ = true;
  cv::Mat image_;

  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;
  void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
public:
  Color();
};

Color::Color() : Node("color")
{
  set_parameter_if_not_set("red", red_);
  get_parameter_or("red", red_, red_);
  set_parameter_if_not_set("green", green_);
  get_parameter_or("green", green_, green_);
  set_parameter_if_not_set("blue", blue_);
  get_parameter_or("blue", blue_, blue_);
  // this works okay but would rather get width and height from
  // an input image
  set_parameter_if_not_set("width", width_);
  get_parameter_or("width", width_, width_);
  set_parameter_if_not_set("height", height_);
  get_parameter_or("height", height_, height_);
  // TODO(lucasw) get update rate

  pub_ = create_publisher<sensor_msgs::msg::Image>("image");

  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  param_sub_ = parameters_client_->on_parameter_event(
      std::bind(&Color::onParameterEvent, this, _1));

  updateTimer();
  // TODO(lucasw) get width height
  if (frame_rate_ > 0.0) {
    int period_ms = 1000.0 / frame_rate_;
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
        std::bind(&Color::pubImage, this));
  }
}

void Color::updateTimer()
{
  int period_ms = 1000.0 / frame_rate_;
  timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
      std::bind(&Color::pubImage, this));
}

void Color::pubImage()
{
  if (dirty_ || image_.empty()) {
    image_ = cv::Mat(cv::Size(width_, height_), CV_8UC3);
    image_ = cv::Scalar(red_, green_, blue_);
    dirty_ = false;
  }

  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = now();  // or reception time of original message?
  cv_image.image = image_;
  cv_image.encoding = "rgb8";
  pub_->publish(cv_image.toImageMsg());
}

void Color::onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  // auto -> ParameterValue
  for (auto & parameter : event->new_parameters) {
    const std::string name = parameter.name;
    RCLCPP_WARN(get_logger(), "Unexpected %s parameter", name.c_str());
  }
  for (auto & changed_parameter : event->changed_parameters) {
    const std::string name = changed_parameter.name;
    if (changed_parameter.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      if (name == "frame_rate") {
        frame_rate_ = changed_parameter.value.double_value;
        updateTimer();
      } else {
        RCLCPP_WARN(get_logger(), "No '%s' parameter or mismatching type %d",
          name.c_str(), changed_parameter.value.type);
      }
    }
    if (changed_parameter.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      const int value = changed_parameter.value.integer_value;
      dirty_ = true;
      if (name == "red") {
        red_ = value;
      } else if (name == "green") {
        green_ = value;
      } else if (name == "blue") {
        blue_ = value;
      } else if (name == "width") {
        width_ = value;
      } else if (name == "height") {
        height_ = value;
      } else {
        RCLCPP_WARN(get_logger(), "No '%s' parameter or mismatching type %d",
          name.c_str(), changed_parameter.value.type);
        dirty_ = false;
      }
    } else {
      // TODO(lucasw) maybe should rescan controls, or provide that function elsewhere
      RCLCPP_WARN(get_logger(), "No '%s' parameter or mismatching type %d",
          name.c_str(), changed_parameter.value.type);
      continue;
    }  // is expected parameter
  }  // loop through changed parameters
}  // parameter event handler

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  auto color = std::make_shared<Color>();
  // color->init();
  rclcpp::spin(color);
  rclcpp::shutdown();
  return 0;
}

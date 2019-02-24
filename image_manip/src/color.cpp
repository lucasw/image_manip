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
#include <image_manip/color.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
using std::placeholders::_1;

namespace image_manip
{

Color::Color()
{
  // will crash if try to use any null pointers here, because init() hasn't run.
}

Color::~Color()
{
  RCLCPP_INFO(get_logger(), "deconstructor");
}

void Color::postInit(std::shared_ptr<internal_pub_sub::Core> core)
{
  RCLCPP_INFO(get_logger(), "post init");
  Node::postInit(core);
  try {
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
    set_parameter_if_not_set("frame_rate", frame_rate_);
    get_parameter_or("frame_rate", frame_rate_, frame_rate_);
  } catch (rclcpp::ParameterTypeException& ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
  }
  // TODO(lucasw) remapping doesn't work with multiple same type nodes in same process
  // std::string topic = "image";
  // get_parameter_or("image", topic, topic);

  // image_pub_ = create_publisher<sensor_msgs::msg::Image>("image");
  image_pub_ = get_create_internal_publisher("image");

  std::cout << width_ << " x " << height_ << "\n";
#if 0
  this->register_param_change_callback(std::bind(&Color::paramChangeCallback, this, _1));
#endif
#if 1
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  param_sub_ = parameters_client_->on_parameter_event(
      std::bind(&Color::onParameterEvent, this, _1));
#endif

  updateTimer();
}

void Color::updateTimer()
{
  if (frame_rate_ > 0.0) {
    int period_ms = 1000.0 / frame_rate_;
    std::cout << "frame rate " << frame_rate_ << ", period ms " << period_ms << "\n";
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
        std::bind(&Color::pubImage, this));
  } else {
    std::cout << "setting frame rate to 0.0\n";
    timer_ = nullptr;
  }
}

void Color::pubImage()
{
  // RCLCPP_INFO(get_logger(), "timer update");
  if (dirty_ || image_.empty()) {
    get_parameter_or("red", red_, red_);
    get_parameter_or("green", green_, green_);
    get_parameter_or("blue", blue_, blue_);
    get_parameter_or("width", width_, width_);
    get_parameter_or("height", height_, height_);

    image_ = cv::Mat(cv::Size(width_, height_), CV_8UC3);
    image_ = cv::Scalar(red_, green_, blue_);
    dirty_ = false;
  }

  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = now();  // or reception time of original message?
  cv_image.image = image_;
  cv_image.encoding = "rgb8";
  // TODO(lucasw) cache the converted image message and only call toImageMsg() above
  // in if dirty.
  image_pub_->publish(cv_image.toImageMsg());
}

#if 0
rcl_interfaces::msg::SetParametersResult Color::paramChangeCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::cout << parameters.size() << " ";
  for (auto param : parameters) {
    std::cout << param.get_name() << " ";
  }
  rcl_interfaces::msg::SetParametersResult result;
  dirty_ = true;
  result.successful = true;
  return result;
}
#endif

#if 1
void Color::onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  const std::string full_name = std::string(get_namespace()) + std::string(get_name());
  if (event->node != full_name) {
    return;
  }

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
      dirty_ = true;
    } else {
      // TODO(lucasw) maybe should rescan controls, or provide that function elsewhere
      RCLCPP_WARN(get_logger(), "No '%s' parameter or mismatching type %d",
          name.c_str(), changed_parameter.value.type);
      continue;
    }  // is expected parameter
  }  // loop through changed parameters
}  // parameter event handler
#endif

}  // image_manip

#include <class_loader/register_macro.hpp>

CLASS_LOADER_REGISTER_CLASS(image_manip::Color, internal_pub_sub::Node)

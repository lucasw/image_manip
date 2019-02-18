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

#ifndef IMAGE_MANIP_COLOR_HPP
#define IMAGE_MANIP_COLOR_HPP

#include <cv_bridge/cv_bridge.h>
#include <internal_pub_sub/internal_pub_sub.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace image_manip
{

class Color : public internal_pub_sub::Node
{
protected:
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::shared_ptr<internal_pub_sub::Publisher> image_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  void pubImage();

  int width_ = 1024;
  int height_ = 1024;
  int red_ = 255;
  int green_ = 255;
  int blue_= 255;

  double frame_rate_ = 20.0;
  void updateTimer();

  bool dirty_ = true;
  cv::Mat image_;

#if 0
  rcl_interfaces::msg::SetParametersResult paramChangeCallback(std::vector<rclcpp::Parameter> parameters);
#endif
#if 1
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;
  void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
#endif

public:
  Color();
  ~Color();
  virtual void postInit(std::shared_ptr<internal_pub_sub::Core> core);
};

}  // image_manip

#endif  // IMAGE_MANIP_COLOR_HPP

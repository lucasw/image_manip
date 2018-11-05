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

public:
  Color();
};

Color::Color() : Node("color")
{
  get_parameter_or("red", red_, red_);
  get_parameter_or("green", green_, green_);
  get_parameter_or("blue", blue_, blue_);
  // this works okay but would rather get width and height from
  // an input image
  get_parameter_or("width", width_, width_);
  get_parameter_or("height", height_, height_);
  // TODO(lucasw) get update rate

  pub_ = create_publisher<sensor_msgs::msg::Image>("image");

  // TODO(lucasw) get width height
  timer_ = create_wall_timer(std::chrono::seconds(1),
      std::bind(&Color::pubImage, this));
}

void Color::pubImage()
{
  cv::Mat out = cv::Mat(cv::Size(width_, height_), CV_8UC3);
  out = cv::Scalar(red_, green_, blue_);

  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = now();  // or reception time of original message?
  cv_image.image = out;
  cv_image.encoding = "rgb8";
  pub_->publish(cv_image.toImageMsg());
}

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

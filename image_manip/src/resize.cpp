/*
 * Copyright (c) 2017 Lucas Walter
 * June 2017
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cv_bridge/cv_bridge.h>
#include <image_manip/utility.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <string>
using std::placeholders::_1;

namespace image_manip
{

class Resize : public rclcpp::Node
{
public:
Resize() : Node("resize")
{
  dirty_ = false;
  pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_out");

  get_parameter_or("frame_rate", frame_rate_, frame_rate_);
  get_parameter_or("width", width_, width_);
  get_parameter_or("height", height_, height_);

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_in",
      std::bind(&image_manip::Resize::imageCallback, this, _1));
  width_sub_ = this->create_subscription<std_msgs::msg::UInt16>("width",
      std::bind(&image_manip::Resize::widthCallback, this, _1));
  height_sub_ = this->create_subscription<std_msgs::msg::UInt16>("height",
      std::bind(&image_manip::Resize::heightCallback, this, _1));

  if (frame_rate_ > 0)
  {
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<long int>(1000.0 / frame_rate_)),
        std::bind(&Resize::update, this));
    RCLCPP_INFO(get_logger(), "starting timer");
  }
}

~Resize()
{
}

void widthCallback(const std_msgs::msg::UInt16::SharedPtr msg)
{
  width_ = msg->data;
}

void heightCallback(const std_msgs::msg::UInt16::SharedPtr msg)
{
  height_ = msg->data;
}

void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  msg_ = msg;
  dirty_ = true;

  // negative frame_rate is a disable
  // TODO(lucasw) or should that be the other way around?
  if (frame_rate_ < 0)
  {

  }
  else if (frame_rate_ == 0)
  {
    update();
  }
}

void update()
{
  if (!dirty_)
    return;

  if (!msg_)
    return;

  // TODO(lucasw) optionally convert encoding to dr type or keep same
  const sensor_msgs::msg::Image::SharedPtr msg = msg_;
  const std::string encoding = msg->encoding;
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // TBD why converting to BGR8
    cv_ptr = cv_bridge::toCvShare(msg, encoding);
    //, "mono8"); // sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  int width = width_;
  if (width <= 0)
  {
    width = cv_ptr->image.cols;
  }
  int height = height_;
  if (height <= 0)
  {
    height = cv_ptr->image.rows;
  }
  const cv::Size size(width, height);
  cv_bridge::CvImage cv_image;

  try {
  if (mode_ == 0)
  {
    resizeFixAspect(cv_ptr->image, cv_image.image, size, interpolate_mode_);
  }
  if (mode_ == 1)
  {
    resizeFixAspectFill(cv_ptr->image, cv_image.image, size, interpolate_mode_);
  }
  if (mode_ == 2)
  {
    cv::resize(cv_ptr->image, cv_image.image, size, 0, 0, interpolate_mode_);
  }
  }
  catch (cv::Exception& ex)
  {
    return;
  }

  cv_image.header = cv_ptr->header;  // or reception time of original message?
  cv_image.encoding = encoding;
  pub_->publish(cv_image.toImageMsg());

  dirty_ = false;
}

private:
  // sensor_msgs::
  bool dirty_ = false;
  int width_ = 0;
  int height_ = 0;
  float frame_rate_ = 0.0;
  unsigned int mode_ = 0;
  int interpolate_mode_ = cv::INTER_NEAREST;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr width_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr height_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};  // Resize
}  // namespace image_manip


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::spin(std::make_shared<image_manip::Resize>());
  rclcpp::shutdown();
  return 0;
}


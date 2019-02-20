/**
 Copyright 2015-2018 Lucas Walter

    TODO(lucasw) relicense to BSD3
    This file is part of ImageManip.

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

  Save a received image to disk

  TODO(lucasw) want to also have a gate_image node which passes images
  through when triggered- but for now image_deque and this node will
  duplicate that functionality and only save or add to the deque 
  when triggered.
*/

#include <cv_bridge/cv_bridge.h>
#include <image_manip/save_image.hpp>
#include <iomanip>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
using std::placeholders::_1;

namespace image_manip
{

SaveImage::SaveImage()
{
}

void SaveImage::postInit(std::shared_ptr<internal_pub_sub::Core> core)
{
  internal_pub_sub::Node::postInit(core);

  get_parameter_or("prefix", prefix_, prefix_);
  get_parameter_or("capture_continuous", capture_continuous_, capture_continuous_);

  start_time_ = now();
  std::stringstream ss;
  // TODO(lucasw) year month day later
  ss << "_" << int(start_time_.nanoseconds() / 1e9) << "_";
  prefix_ += ss.str();
  RCLCPP_INFO(get_logger(), "prefix %s", prefix_.c_str());

  timer_ = create_wall_timer(std::chrono::milliseconds(20),
      std::bind(&SaveImage::update, this));


  // saved_pub_ = create_publisher<sensor_msgs::msg::Image>("saved_image", true);
  saved_pub_ = get_create_internal_publisher("saved_image");

  single_sub_ = create_subscription<std_msgs::msg::Bool>("single",
      std::bind(&SaveImage::singleCallback, this, _1));
  continuous_sub_ = create_subscription<std_msgs::msg::Bool>("continuous",
      std::bind(&SaveImage::continuousCallback, this, _1));

  // image_sub_ = create_subscription<sensor_msgs::msg::Image>("image",
  image_sub_ = create_internal_subscription("image",
      std::bind(&SaveImage::imageCallback, this, _1));

  RCLCPP_INFO(get_logger(), "save image initialized");
}

SaveImage::~SaveImage()
{
  RCLCPP_INFO(get_logger(), "saved %d images", counter_);
}

void SaveImage::singleCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  capture_single_ = msg->data;
}

void SaveImage::continuousCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  capture_continuous_ = msg->data;
}

void SaveImage::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!(capture_single_ || capture_continuous_))
    return;
  capture_single_ = false;

  std::lock_guard<std::mutex> lock(mutex_);
  image_ = msg;
  dirty_ = true;
}

void SaveImage::update()
{
  sensor_msgs::msg::Image::SharedPtr image;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    image = image_;
    if (!dirty_) {
      return;
    }
  }
  dirty_ = false;

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // TBD why converting to BGR8
    // cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
    // , "mono8"); // sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  std::stringstream ss;
  ss << prefix_ << std::setw(8) << std::setfill('0') << counter_ << ".png";
  RCLCPP_INFO(get_logger(), "saving %s", ss.str().c_str());
  cv::imwrite(ss.str(), cv_ptr->image);
  counter_++;

  saved_pub_->publish(image);
}

}  // namespace image_manip

#include <class_loader/register_macro.hpp>

CLASS_LOADER_REGISTER_CLASS(image_manip::SaveImage, internal_pub_sub::Node)

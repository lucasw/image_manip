/*
 * Copyright (c) 2017 Lucas Walter
 * July 2017
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

#ifndef IMAGE_MANIP_IMAGE_DEQUE_HPP
#define IMAGE_MANIP_IMAGE_DEQUE_HPP

#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <image_manip/utility.h>
#include <internal_pub_sub/internal_pub_sub.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int16.hpp>
using std::placeholders::_1;


namespace image_manip
{
class ImageDeque : public rclcpp::Node
{
public:
  ImageDeque(std::shared_ptr<internal_pub_sub::Core> core = nullptr);
  ~ImageDeque();

  void init();
private:
  std::shared_ptr<internal_pub_sub::Core> core_;

  // send a bool to indicate that an image was saved
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr captured_trigger_pub_;
  // publish the most recent captured image
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr captured_pub_;
  std::shared_ptr<internal_pub_sub::Publisher> captured_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr anim_pub_;
  std::shared_ptr<internal_pub_sub::Publisher> anim_pub_;
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  std::shared_ptr<internal_pub_sub::Subscriber> image_sub_;

  // this is for appending onto the animation output
  sensor_msgs::msg::Image::SharedPtr live_frame_;
  std::deque<sensor_msgs::msg::Image::SharedPtr> images_;

  // TODO(lucasw) some of these ought to be parameters
  float frame_rate_ = 5.0;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr frame_rate_sub_;
  void frameRateCallback(const std_msgs::msg::Float32::SharedPtr msg);

  bool capture_single_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr capture_single_sub_;
  void captureSingleCallback(const std_msgs::msg::Bool::SharedPtr msg);

  unsigned int index_ = 0;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr index_sub_;
  void indexCallback(const std_msgs::msg::UInt16::SharedPtr msg);

  // TODO(lucasw) index_fraction

  unsigned int start_index_ = 0;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr start_index_sub_;
  void startIndexCallback(const std_msgs::msg::UInt16::SharedPtr msg);

  bool use_live_frame_ = true;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr use_live_frame_sub_;
  void useLiveFrameCallback(const std_msgs::msg::Bool::SharedPtr msg);

  bool capture_continuous_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr capture_continuous_sub_;
  void captureContinuousCallback(const std_msgs::msg::Bool::SharedPtr msg);

  unsigned int max_size_ = 1000;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr max_size_sub_;
  void maxSizeCallback(const std_msgs::msg::UInt16::SharedPtr msg);

  bool dirty_ = false;
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::TimerBase::SharedPtr timer_;
  void update();
};  // ImageDeque

}  // image_manip

#endif

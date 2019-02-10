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

#ifndef IMAGE_MANIP_IIR_IMAGE_HPP
#define IMAGE_MANIP_IIR_IMAGE_HPP

#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <image_manip/utility.h>
#include <internal_pub_sub/internal_pub_sub.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>


namespace image_manip
{

class IIRImage : public rclcpp::Node
{
public:
  IIRImage(std::shared_ptr<internal_pub_sub::Core> core = nullptr);
  ~IIRImage();
  void init();
private:
  std::shared_ptr<internal_pub_sub::Core> core_;

  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::shared_ptr<internal_pub_sub::Publisher> image_pub_;
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::shared_ptr<internal_pub_sub::Subscriber> image_sub_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;

  double frame_rate_ = 20;
  std::vector<double> b_coeffs_;
  // faster to leave this empty, which amounts to a_coeffs_ = {1.0};
  std::vector<double> a_coeffs_;

  bool use_time_sequence_ = false;

  rclcpp::TimerBase::SharedPtr timer_;
  void update();

  std::deque<sensor_msgs::msg::Image::SharedPtr> in_images_;
  std::deque<cv_bridge::CvImageConstPtr> in_cv_images_;
  std::deque<cv::Mat> out_frames_;
  bool dirty_ = false;

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void imagesCallback(const sensor_msgs::msg::Image::SharedPtr msg, const size_t index);
};

}  // image_manip

#endif  // IMAGE_MANIP_IIR_IMAGE_HPP

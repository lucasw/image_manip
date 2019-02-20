/**
 * Copyright 2015-2019 Lucas Walter
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
 *
 *  Save a received image to disk
 *
 * TODO(lucasw) want to also have a gate_image node which passes images
 * through when triggered- but for now image_deque and this node will
 * duplicate that functionality and only save or add to the deque
 * when triggered.
 *
 */

#ifndef IMAGE_MANIP_SAVE_IMAGE_HPP
#define IMAGE_MANIP_SAVE_IMAGE_HPP

#include <internal_pub_sub/internal_pub_sub.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>


namespace image_manip
{

class SaveImage : public internal_pub_sub::Node
{
public:
  SaveImage();
  ~SaveImage();
  virtual void postInit(std::shared_ptr<internal_pub_sub::Core> core);
protected:
  // TODO(lucasw) maybe mode to capture N images then stop?
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  std::shared_ptr<internal_pub_sub::Subscriber> image_sub_;
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr saved_pub_;
  std::shared_ptr<internal_pub_sub::Publisher> saved_pub_;

  int counter_ = 0;
  rclcpp::Time start_time_;
  std::string prefix_ = "frame";

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // maybe these should be in base class
  bool capture_single_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr single_sub_;
  void singleCallback(const std_msgs::msg::Bool::SharedPtr msg);

  bool capture_continuous_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr continuous_sub_;
  void continuousCallback(const std_msgs::msg::Bool::SharedPtr msg);

  bool dirty_ = false;
  sensor_msgs::msg::Image::SharedPtr image_;
  std::mutex mutex_;
  void update();
  rclcpp::TimerBase::SharedPtr timer_;
};

}

#endif  // IMAGE_MANIP_SAVE_IMAGE_HPP

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

#ifndef IMAGE_MANIP_IMAGE_DELAY_H
#define IMAGE_MANIP_IMAGE_DELAY_H

#include <deque>
#include <dynamic_reconfigure/server.h>
#include <image_manip/ImageDequeConfig.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <vector>

namespace image_manip
{

class ImageDeque : public nodelet::Nodelet
{
  // send a bool to indicate that an image was saved
  ros::Publisher captured_trigger_pub_;
  // publish the most recent captured image
  ros::Publisher captured_pub_;
  ros::Publisher anim_pub_;
  ros::Subscriber image_sub_;

  unsigned int index_;

  void pubImage(const ros::TimerEvent& e);

  // this is for appending onto the animation output
  sensor_msgs::ImageConstPtr live_frame_;
  std::deque<sensor_msgs::ImageConstPtr> images_;

	// TODO(lucasw) are these needed if dynamic reconfigure handles them?
  bool capture_single_;
  ros::Subscriber single_sub_;
  void singleCallback(const std_msgs::Bool::ConstPtr& msg);

  ros::Subscriber continuous_sub_;
  void continuousCallback(const std_msgs::Bool::ConstPtr& msg);

  unsigned int max_size_;
  ros::Subscriber max_size_sub_;
  void maxSizeCallback(const std_msgs::UInt16::ConstPtr& msg);
  //////

  image_manip::ImageDequeConfig config_;
  typedef dynamic_reconfigure::Server<image_manip::ImageDequeConfig> ReconfigureServer;
  boost::shared_ptr< ReconfigureServer > server_;
  void callback(image_manip::ImageDequeConfig& config,
      uint32_t level);
  boost::recursive_mutex dr_mutex_;

  bool dirty_;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  ros::Timer timer_;
  void update(const ros::TimerEvent& e);

public:
  virtual void onInit();
  ImageDeque();
  ~ImageDeque();
};

}  // namespace image_manip

#endif  // IMAGE_MANIP_IMAGE_DELAY_H

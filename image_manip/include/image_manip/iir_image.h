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

#ifndef IMAGE_MANIP_IIR_IMAGE_H
#define IMAGE_MANIP_IIR_IMAGE_H

#include <deque>
#include <dynamic_reconfigure/server.h>
#include <image_manip/IIRImageConfig.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <vector>

namespace image_manip
{

void updateTimer(ros::Timer& timer, const float frame_rate,
    const float old_frame_rate);

class IIRImage : public nodelet::Nodelet
{
  // image_transport::ImageTransport it_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  // TODO(lucasw) should this be ImageTransport- is it faster?
  std::vector<ros::Subscriber> image_subs_;

  void updateTopics(image_manip::IIRImageConfig& config);

  image_manip::IIRImageConfig config_;
  typedef dynamic_reconfigure::Server<image_manip::IIRImageConfig> ReconfigureServer;
  boost::shared_ptr< ReconfigureServer > server_;
  void callback(image_manip::IIRImageConfig& config,
      uint32_t level);

  boost::recursive_mutex dr_mutex_;

  bool use_time_sequence_;
  std::vector<double> b_coeffs_;
  std::vector<double> a_coeffs_;

  ros::Timer timer_;
  void update(const ros::TimerEvent& e);

  std::deque<sensor_msgs::ImageConstPtr> in_images_;
  // store converted images also, 1:1 with in_images_
  std::deque<cv_bridge::CvImageConstPtr> in_cv_images_;
  // Don't need two deques here
  std::deque<cv::Mat> out_frames_;
  bool dirty_;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void imagesCallback(const sensor_msgs::ImageConstPtr& msg, const size_t index);

public:
  virtual void onInit();
  IIRImage();
  ~IIRImage();
};

}  // namespace image_manip

#endif  // IMAGE_MANIP_IIR_IMAGE_H

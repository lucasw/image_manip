/*
 * Copyright (c) 2016 Lucas Walter
 * March 2016
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

#ifndef IMAGE_MANIP_DISTORT_H
#define IMAGE_MANIP_DISTORT_H

#include <deque>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
// #include <image_manip/DistortConfig.h>
#include <memory>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>

namespace image_manip
{

class Distort : public nodelet::Nodelet
{
#if 0
  image_manip::DistortConfig config_;
  typedef dynamic_reconfigure::Server<image_manip::DistortConfig> ReconfigureServer;
  boost::shared_ptr< ReconfigureServer > server_;
  void callback(image_manip::DistortConfig& config,
      uint32_t level);
  boost::recursive_mutex dr_mutex_;

  bool dirty_;
#endif

  ros::Timer timer_;
  void update(const ros::TimerEvent& e);

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher image_pub_;
  ros::Publisher camera_info_pub_;
  sensor_msgs::CameraInfo camera_info_;
  image_transport::Subscriber image_sub_;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  ros::Subscriber camera_info_sub_;
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  // TODO(lucasw) move to dynamic reconfigure
  bool use_debug_ = false;
  image_transport::Publisher debug_image_pub_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Mat map_1_;
  cv::Mat map_2_;
  bool new_maps_needed_ = true;

public:
  virtual void onInit();
  Distort();
  ~Distort();
};

}  // namespace image_manip

#endif  // IMAGE_MANIP_DISTORT_H

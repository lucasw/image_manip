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

#ifndef IMAGE_MANIP_ROTATE90_H
#define IMAGE_MANIP_ROTATE90_H

#include <deque>
#include <dynamic_reconfigure/server.h>
#include <image_manip/Rotate90Config.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <vector>

namespace image_manip
{

class Rotate90 : public nodelet::Nodelet
{
  // image_transport::ImageTransport it_;
  ros::Publisher pub_;
  ros::Publisher camera_info_pub_;

  ros::Subscriber sub_;
  ros::Subscriber camera_info_sub_;

  image_manip::Rotate90Config config_;
  typedef dynamic_reconfigure::Server<image_manip::Rotate90Config> ReconfigureServer;
  boost::shared_ptr< ReconfigureServer > server_;
  void callback(image_manip::Rotate90Config& config,
      uint32_t level);
  boost::recursive_mutex dr_mutex_;

  std::deque<sensor_msgs::ImageConstPtr> images_;
  bool dirty_;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  ros::Timer timer_;
  void update(const ros::TimerEvent& e);


public:
  virtual void onInit();
  Rotate90();
  ~Rotate90();
};

}  // namespace image_manip

#endif  // IMAGE_MANIP_ROTATE90_H

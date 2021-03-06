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

#ifndef IMAGE_MANIP_ROTO_ZOOM_H
#define IMAGE_MANIP_ROTO_ZOOM_H

#include <deque>
#include <dynamic_reconfigure/server.h>
#include <image_manip/RotoZoomConfig.h>
#include <map>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <string>
#include <vector>

namespace image_manip
{

class RotoZoom : public nodelet::Nodelet
{
  // image_transport::ImageTransport it_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Subscriber background_sub_;

  std::map<std::string, ros::Subscriber> param_sub_;
  std::map<std::string, double*> params_;
  void paramCallback(std_msgs::Float32::ConstPtr msg, const std::string name);

  image_manip::RotoZoomConfig config_;
  typedef dynamic_reconfigure::Server<image_manip::RotoZoomConfig> ReconfigureServer;
  boost::shared_ptr< ReconfigureServer > server_;
  void callback(image_manip::RotoZoomConfig& config,
      uint32_t level);

  boost::recursive_mutex dr_mutex_;

  std::deque<sensor_msgs::ImageConstPtr> images_;
  sensor_msgs::ImageConstPtr background_image_;
  // this contains no image data, just a reference for the height, width, etc.
  sensor_msgs::Image output_image_info_;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void backgroundImageCallback(const sensor_msgs::ImageConstPtr& msg);
  bool dirty_;
  ros::Timer timer_;
  void update(const ros::TimerEvent& e);

public:
  virtual void onInit();
  RotoZoom();
  ~RotoZoom();
};

}  // namespace image_manip

#endif  // IMAGE_MANIP_ROTO_ZOOM_H

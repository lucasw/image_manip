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

#ifndef IMAGE_MANIP_UTILITY_H
#define IMAGE_MANIP_UTILITY_H

#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace image_manip
{
  // TODO(lucasw) need a float parameter to scale between these two options
  // leave black borders
  bool resizeFixAspect(const cv::Mat& tmp0, cv::Mat& tmp1,
      const cv::Size sz, const int mode);
  // fill to edges, chop off edges as necessary
  bool resizeFixAspectFill(const cv::Mat& tmp0, cv::Mat& tmp1,
      const cv::Size sz, const int mode);

  void updateTimer(ros::Timer& timer, const float frame_rate,
      const float old_frame_rate);

  // this just catches exceptions
  bool imageToMat(const sensor_msgs::ImageConstPtr& msg,
      cv_bridge::CvImageConstPtr& cv_ptr,
      const std::string encoding = "");

  bool sameImageType(const sensor_msgs::Image& im1, const sensor_msgs::ImageConstPtr& im2);
  bool sameImageType(const sensor_msgs::ImageConstPtr& im1, const sensor_msgs::ImageConstPtr& im2);
}

#endif  // IMAGE_MANIP_UTILITY_H

/*
 * Copyright (c) 2017 Lucas Walter
 * November 2017
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

#include <cv_bridge/cv_bridge.h>
#include <image_manip/cv_distort_image.h>
#include <image_manip/distort.h>
#include <image_manip/utility.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <sensor_msgs/image_encodings.h>

namespace image_manip
{

Distort::Distort()
{
}

Distort::~Distort()
{
}

void Distort::onInit()
{
  it_ = std::make_shared<image_transport::ImageTransport>(getNodeHandle());

  // TODO(lucasw) use CameraPublisher to sync camera info and image?
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  image_pub_ = it_->advertise("distorted/image", 1, true);

  ros::param::get("~use_debug", use_debug_);
  if (use_debug_) {
    debug_image_pub_ = it_->advertise("debug_image", 1, true);
  }

  camera_info_pub_ = getNodeHandle().advertise<sensor_msgs::CameraInfo>("distorted/camera_info", 1);
  image_sub_ = it_->subscribe("image", 1, &Distort::imageCallback, this);
  camera_info_sub_ = getNodeHandle().subscribe("camera_info", 1, &Distort::cameraInfoCallback, this);
}

// TODO(lucasw) this use of a non synchronized callback is really non-standard
// should use a TimeSynchronizer and expect the camera info to have matching
// headers with the image.
void Distort::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  camera_info_ = *msg;

  if (dist_coeffs_.rows != msg->D.size())
  {
    new_maps_needed_ = true;
    dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
  }

  for (size_t i = 0; i < msg->D.size(); ++i)
  {
    if (dist_coeffs_.at<double>(i, 0) != msg->D[i])
    {
      new_maps_needed_ = true;
      dist_coeffs_.at<double>(i, 0) = msg->D[i];
    }
  }

  int ind = 0;
  for (size_t y = 0; y < camera_matrix_.rows; ++y)
  {
    for (size_t x = 0; x < camera_matrix_.cols; ++x)
    {
      if (ind > msg->K.size())
      {
        // TODO(lucasw) save the old camera_info if the new
        // is bad?
        ROS_ERROR_STREAM(msg->K.size() << " " << camera_matrix_.size() << " " << ind);
        return;
      }

      if (camera_matrix_.at<double>(y, x) != msg->K[ind])
      {
        new_maps_needed_ = true;
        camera_matrix_.at<double>(y, x) = msg->K[ind];
      }
      ++ind;
    }
  }

  // TODO(lucasw) need to convert cv::Mat to cv::Matx33d everywhere to use this
  // cameraInfoToCV(msg, camera_matrix_, dist_coeffs_);

  // ROS_INFO_STREAM(dist_coeffs_);
}

void Distort::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (camera_matrix_.empty())
    return;
  if (dist_coeffs_.empty())
    return;

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // TODO(lucasw) can this be done automatically by cv bridge?
    if (msg->encoding == "rgb8") {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    } else {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    //, "mono8"); // sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cv_ptr->image.empty())
  {
    ROS_ERROR("no converted image");
  }

  cv::Size src_size = cv_ptr->image.size();
  cv::Mat distorted_cv_image;
  if (map_1_.empty() || map_2_.empty() ||
      (map_1_.size() != src_size) ||
      (map_2_.size() != src_size) ||
      new_maps_needed_)
  {
    initDistortMap(camera_matrix_, dist_coeffs_, src_size, map_1_, map_2_);
    new_maps_needed_ = false;
  }
  // Don't ever call distort() directly because it is more efficient to
  // reuse maps.
  // TODO(lucasw) make the interpolation controllable
  cv::remap(cv_ptr->image, distorted_cv_image, map_1_, map_2_, CV_INTER_LINEAR);

  cv_bridge::CvImage distorted_image;
  distorted_image.header = msg->header;
  distorted_image.encoding = msg->encoding;
  distorted_image.image = distorted_cv_image;
  sensor_msgs::ImagePtr image_msg = distorted_image.toImageMsg();
  camera_info_.header = image_msg->header;
  // camera_info_.roi.do_rectify = true;
  image_pub_.publish(image_msg);
  camera_info_pub_.publish(camera_info_);

  if (use_debug_)
  {
    cv::Mat debug_cv_image;
    cv::undistort(distorted_cv_image, debug_cv_image, camera_matrix_, dist_coeffs_);
    cv_bridge::CvImage debug_image;
    debug_image.header = msg->header;
    debug_image.encoding = msg->encoding;
    debug_image.image = debug_cv_image;
    debug_image_pub_.publish(debug_image.toImageMsg());
  }
}

}  // namespace image_manip

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(image_manip::Distort, nodelet::Nodelet)

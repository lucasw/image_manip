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

#include <cv_bridge/cv_bridge.h>
#include <image_manip/pad.h>
#include <image_manip/utility.h>
#include <image_manip/utility_ros.h>
#include <memory>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <string>


namespace image_manip
{

Pad::Pad()
{
}

Pad::~Pad()
{
}

void Pad::imageCb(const sensor_msgs::ImageConstPtr& msg,
                  const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // TODO(lucasw) optionally convert encoding to dr type or keep same
  const std::string encoding = msg->encoding;
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImage cv_image;
  const auto padded_width = msg->width + x_pad_ * 2;
  const auto padded_height = msg->height + y_pad_ * 2;
  const cv::Size padded_size(padded_width, padded_height);

  cv_image.image = cv::Mat(padded_size, cv_ptr->image.type(), cv::Scalar::all(0));
  cv::Mat dst_roi = cv_image.image(cv::Rect(x_pad_, y_pad_, msg->width, msg->height));
  cv_ptr->image.copyTo(dst_roi);

  cv_image.header = cv_ptr->header;
  cv_image.encoding = encoding;
  image_pub_.publish(cv_image.toImageMsg());

  sensor_msgs::CameraInfo padded_camera_info = *info_msg;
  padded_camera_info.width = padded_width;
  padded_camera_info.height = padded_height;
  padded_camera_info.K[2] += x_pad_;
  padded_camera_info.K[5] += y_pad_;
  padded_camera_info.P[2] += x_pad_;
  padded_camera_info.P[6] += y_pad_;
  camera_info_pub_.publish(padded_camera_info);
}

void Pad::onInit()
{
  it_ = std::make_shared<image_transport::ImageTransport>(getNodeHandle());

  image_pub_ = getPrivateNodeHandle().advertise<sensor_msgs::Image>("image_out", 5);
  camera_info_pub_ = getPrivateNodeHandle().advertise<sensor_msgs::CameraInfo>("camera_info", 5);

#if 0
  server_.reset(new ReconfigureServer(dr_mutex_, getPrivateNodeHandle()));
  dynamic_reconfigure::Server<image_manip::PadConfig>::CallbackType cbt =
      boost::bind(&Pad::callback, this, boost::placeholders::_1, boost::placeholders::_2);
  server_->setCallback(cbt);
#endif

  image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
  image_sub_ = it_->subscribeCamera("image_in", 2, &Pad::imageCb, this, hints);
  // image_sub_ = getNodeHandle().subscribe("image_in", 5,
  //     &Pad::imageCallback, this);
}

};  // namespace image_manip

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(image_manip::Pad, nodelet::Nodelet)

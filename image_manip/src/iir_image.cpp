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
#include <image_manip/iir_image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


namespace image_manip
{

IIRImage::IIRImage() :
  use_time_sequence_(true),
  dirty_(false)
{
}

IIRImage::~IIRImage()
{
}

void IIRImage::callback(
    image_manip::IIRImageConfig& config,
    uint32_t level)
{
  config_ = config;

  if (use_time_sequence_)
    b_coeffs_.resize(8);
  if (b_coeffs_.size() > 0)
    b_coeffs_[0] = config_.b0;
  if (b_coeffs_.size() > 1)
    b_coeffs_[1] = config_.b1;
  if (b_coeffs_.size() > 2)
    b_coeffs_[2] = config_.b2;
  if (b_coeffs_.size() > 3)
    b_coeffs_[3] = config_.b3;
  if (b_coeffs_.size() > 4)
    b_coeffs_[4] = config_.b4;
  if (b_coeffs_.size() > 5)
    b_coeffs_[5] = config_.b5;
  if (b_coeffs_.size() > 6)
    b_coeffs_[6] = config_.b6;
  if (b_coeffs_.size() > 7)
    b_coeffs_[7] = config_.b7;

  dirty_ = true;
}

void IIRImage::update(const ros::TimerEvent& e)
{
  if (!dirty_)
    return;

  cv::Mat out_frame;

  for (size_t i = 0; i < in_frames_.size() && i < b_coeffs_.size(); ++i)
  {
    const double bn = b_coeffs_[i];
    if (i == 0)
      out_frame = in_frames_[i] * bn;
    else if ((out_frame.size() == in_frames_[i].size()) &&
             (out_frame.type() == in_frames_[i].type()))
    {
      if (bn > 0)
        out_frame += in_frames_[i] * bn;
      else if (bn < 0)
        out_frame -= in_frames_[i] * -bn;
    }
  }

  // TODO(lucasw) this may be argument for keeping original Image messages around
  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = ros::Time::now();  // or reception time of original message?
  cv_image.image = out_frame;
  cv_image.encoding = "rgb8";
  pub_.publish(cv_image.toImageMsg());
}

void IIRImage::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // TBD why converting to BGR8
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    //, "mono8"); // sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  in_frames_.push_front(cv_ptr->image.clone());

  if (in_frames_.size() > b_coeffs_.size())
  {
    in_frames_.pop_back();
  }

  dirty_ = true;
}

void IIRImage::imagesCallback(const sensor_msgs::ImageConstPtr& msg, const size_t index)
{
  if (index >= in_frames_.size())
    return;

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // TBD why converting to BGR8
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    //, "mono8"); // sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  in_frames_[index] = cv_ptr->image.clone();

  dirty_ = true;
}

void IIRImage::onInit()
{
  dirty_ = false;
  use_time_sequence_ = true;

  pub_ = getNodeHandle().advertise<sensor_msgs::Image>("image_out", 5);

  server_.reset(new ReconfigureServer(dr_mutex_, getPrivateNodeHandle()));
  dynamic_reconfigure::Server<image_manip::IIRImageConfig>::CallbackType cbt =
      boost::bind(&IIRImage::callback, this, _1, _2);
  server_->setCallback(cbt);

  getPrivateNodeHandle().getParam("~use_time_sequence", use_time_sequence_);

  if (!use_time_sequence_)
  {
    // TODO(lucasw) update config from b_coeffs
    getPrivateNodeHandle().getParam("~b_coeffs", b_coeffs_);
    if (b_coeffs_.size() == 0)
    {
      ROS_WARN_STREAM("no b coefficients");
    }
    in_frames_.resize(b_coeffs_.size());

    for (size_t i = 0; i < b_coeffs_.size(); ++i)
    {
      std::stringstream ss;
      ss << "image_" << i;
      ROS_INFO_STREAM("subscribe " << ss.str() << " " << b_coeffs_[i]);
      image_subs_.push_back(getNodeHandle().subscribe<sensor_msgs::Image>(
          ss.str(), 1,
          boost::bind(&IIRImage::imagesCallback, this, _1, i)));
    }
  }
  // getPrivateNodeHandle().getParam("~a_coeffs", a_coeffs_);
  else
  {
    sub_ = getNodeHandle().subscribe("image_in", 1, &IIRImage::imageCallback, this);
  }

  // TODO(lucasw) move to update function
  const float period = 1.0 / config_.frame_rate;
  timer_ = getPrivateNodeHandle().createTimer(ros::Duration(period),
      &IIRImage::update, this);
}

};  // namespace image_manip

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_manip::IIRImage, nodelet::Nodelet)

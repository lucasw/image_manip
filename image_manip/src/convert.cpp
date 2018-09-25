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
#include <image_manip/convert.h>
#include <image_manip/utility.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <string>


namespace image_manip
{

Convert::Convert()
{
}

Convert::~Convert()
{
}

void Convert::callback(
    image_manip::ConvertConfig& config,
    uint32_t level)
{
  updateTimer(timer_, config.frame_rate, config_.frame_rate);
  config_ = config;
}

void Convert::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  images_.push_back(msg);
  dirty_ = true;

  // negative frame_rate is a disable
  // TODO(lucasw) or should that be the other way around?
  if (config_.frame_rate == 0)
  {
    ros::TimerEvent e;
    e.current_real = ros::Time::now();
    update(e);
  }
}

void Convert::update(const ros::TimerEvent& e)
{
  if (!dirty_)
    return;

  if (images_.size() == 0)
    return;

  // TODO(lucasw) if the message encoding is the same as dst_type, then pass it through
  // without conversion.

  const sensor_msgs::ImageConstPtr msg = images_[0];
  images_.clear();
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImage cv_image;

  std::stringstream enc;
  // if the number of channels is 1, 3, or 4 and the type is 8 or 16 unsigned
  // then can preserve the color channel names from original encoding,
  // but only change the 8 to a 16 or the other way.
  // Otherwise have to use opencv type

  // TODO(lucasw) image_encodings ought to do this
  bool abstract = !(sensor_msgs::image_encodings::isColor(msg->encoding) ||
      sensor_msgs::image_encodings::isBayer(msg->encoding) ||
      sensor_msgs::image_encodings::isMono(msg->encoding) ||
      (msg->encoding == "yuv422"));

  const int num_channels = sensor_msgs::image_encodings::numChannels(msg->encoding);
  const int bit_depth = sensor_msgs::image_encodings::bitDepth(msg->encoding);
  if (!abstract)
  {
    std::string prefix;
    if (bit_depth == 8)
    {
      prefix = msg->encoding.substr(0, msg->encoding.size() - 1);
    }
    else if (bit_depth == 16)
    {
      prefix = msg->encoding.substr(0, msg->encoding.size() - 2);
    }
    else
    {
      // there are no rgba32 or bgr64 or mono32 etc.
      abstract = true;
    }
    enc << prefix;
  }

  // TODO(lucasw) can't handle yuv422 yet

  if (config_.dst_type == image_manip::Convert_passthrough)
  {
    cv_ptr->image.convertTo(cv_image.image, -1, config_.scale, config_.offset);
    enc.str("");
    enc << msg->encoding;
  }
  else if (config_.dst_type == Convert_8UC)
  {
    // TODO(lucasw) the number of channels don't matter here
    cv_ptr->image.convertTo(cv_image.image, CV_8UC1, config_.scale, config_.offset);
    if (abstract)
    {
      enc << "8UC" << num_channels;
    }
    else
    {
      enc << "8";
    }
  }
  else if (config_.dst_type == Convert_8SC)
  {
    cv_ptr->image.convertTo(cv_image.image, CV_8SC1, config_.scale, config_.offset);
    if (abstract)
    {
      enc << "8SC" << num_channels;
    }
    else
    {
      enc << "8";
    }
  }
  else if (config_.dst_type == Convert_16UC)
  {
    cv_ptr->image.convertTo(cv_image.image, CV_16UC1, config_.scale, config_.offset);
    if (abstract)
    {
      enc << "16UC" << num_channels;
    }
    else
    {
      enc << "16";
    }
  }
  else if (config_.dst_type == Convert_16SC)
  {
    cv_ptr->image.convertTo(cv_image.image, CV_16SC1, config_.scale, config_.offset);
    if (abstract)
    {
      enc << "16SC" << num_channels;
    }
    else
    {
      enc << "16";
    }
  }
  else if (config_.dst_type == Convert_32SC)
  {
    cv_ptr->image.convertTo(cv_image.image, CV_32FC1, config_.scale, config_.offset);
    // TODO(lucasw) until there is a rgba32 etc.
    abstract = true;
    enc.str("");
    if (abstract)
    {
      enc << "32SC" << num_channels;
    }
    else
    {
      enc << "32";
    }
  }
  else if (config_.dst_type == Convert_32FC)
  {
    cv_ptr->image.convertTo(cv_image.image, CV_32FC1, config_.scale, config_.offset);
    // TODO(lucasw) until there is a rgba32 etc.
    abstract = true;
    enc.str("");
    if (abstract)
    {
      enc << "32FC" << num_channels;
    }
    else
    {
      enc << "32";
    }
  }
  else if (config_.dst_type == Convert_64FC)
  {
    cv_ptr->image.convertTo(cv_image.image, CV_64FC1, config_.scale, config_.offset);
    abstract = true;
    enc.str("");
    if (abstract)
    {
      enc << "64FC" << num_channels;
    }
    else
    {
      enc << "64";
    }
  }
  cv_image.header = cv_ptr->header;  // or reception time of original message?
  cv_image.encoding = enc.str();
  pub_.publish(cv_image.toImageMsg());

  dirty_ = false;
}

void Convert::onInit()
{
  dirty_ = false;
  pub_ = getNodeHandle().advertise<sensor_msgs::Image>("image_out", 5);

  server_.reset(new ReconfigureServer(dr_mutex_, getPrivateNodeHandle()));
  dynamic_reconfigure::Server<image_manip::ConvertConfig>::CallbackType cbt =
      boost::bind(&Convert::callback, this, _1, _2);
  server_->setCallback(cbt);

  sub_ = getNodeHandle().subscribe("image_in", 5,
      &Convert::imageCallback, this);

  timer_ = getPrivateNodeHandle().createTimer(ros::Duration(1.0),
    &Convert::update, this);
  // force timer start by making old frame_rate different
  updateTimer(timer_, config_.frame_rate, config_.frame_rate - 1.0);
}

};  // namespace image_manip

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_manip::Convert, nodelet::Nodelet)

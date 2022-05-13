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
#include <image_manip/rotate90.h>
#include <image_manip/utility.h>
#include <image_manip/utility_ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <string>


namespace image_manip
{

Rotate90::Rotate90()
{
}

Rotate90::~Rotate90()
{
}

void Rotate90::callback(
    image_manip::Rotate90Config& config,
    uint32_t level)
{
  updateTimer(timer_, config.frame_rate, config_.frame_rate);
  if (config.frame_rate != config_.frame_rate) {
    ROS_INFO_STREAM("frame rate " << config.frame_rate);
  }
  if (config.rotation_mode != config_.rotation_mode) {
    ROS_INFO_STREAM("rotation mode " << config.rotation_mode);
  }
  config_ = config;
}

void Rotate90::imageCallback(const sensor_msgs::ImageConstPtr& msg)
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

void Rotate90::update(const ros::TimerEvent& e)
{
  if (!config_.enable)
    return;

  if (!dirty_)
    return;

  if (images_.size() == 0)
    return;

  // TODO(lucasw) optionally convert encoding to dr type or keep same
  const sensor_msgs::ImageConstPtr msg = images_[0];
  const std::string encoding = msg->encoding;
  images_.clear();
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // TBD why converting to BGR8
    cv_ptr = cv_bridge::toCvShare(msg, encoding);
    //, "mono8"); // sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImage cv_image;

  cv_image.header = cv_ptr->header;
  // TODO(lucasw) generate and publish the frame here for convenience?
  // otherwise user needs to set up static transform publisher that needs
  // to use the right input and output frame_ids
  cv_image.header.frame_id += "_rotated";
  cv_image.encoding = encoding;

  bool do_publish = true;
  switch (config_.rotation_mode)
  {
    case 0:
      pub_.publish(msg);
      do_publish = false;
      break;
    case 1:
      cv::rotate(cv_ptr->image, cv_image.image, cv::ROTATE_90_CLOCKWISE);
      break;
    case 2:
      cv::rotate(cv_ptr->image, cv_image.image, cv::ROTATE_180);
      break;
    case 3:
      cv::rotate(cv_ptr->image, cv_image.image, cv::ROTATE_90_COUNTERCLOCKWISE);
      break;
    // TODO(lucasw) add flip and transpose modes
  }

  if (do_publish)
  {
    pub_.publish(cv_image.toImageMsg());
  }

  dirty_ = false;
}

void Rotate90::onInit()
{
  dirty_ = false;
  pub_ = getNodeHandle().advertise<sensor_msgs::Image>("image_out", 5);

  server_.reset(new ReconfigureServer(dr_mutex_, getPrivateNodeHandle()));
  dynamic_reconfigure::Server<image_manip::Rotate90Config>::CallbackType cbt =
      boost::bind(&Rotate90::callback, this, boost::placeholders::_1, boost::placeholders::_2);
  server_->setCallback(cbt);

  sub_ = getNodeHandle().subscribe("image_in", 5,
      &Rotate90::imageCallback, this);

  timer_ = getPrivateNodeHandle().createTimer(ros::Duration(1.0),
    &Rotate90::update, this);
  // force timer start by making old frame_rate different
  updateTimer(timer_, config_.frame_rate, config_.frame_rate - 1.0);
}

};  // namespace image_manip

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_manip::Rotate90, nodelet::Nodelet)

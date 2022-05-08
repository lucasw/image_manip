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
#include <image_manip/image_delay.h>
#include <image_manip/utility.h>
#include <image_manip/utility_ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


namespace image_manip
{

ImageDelay::ImageDelay()
{
}

ImageDelay::~ImageDelay()
{
}

void ImageDelay::callback(
    image_manip::DelayConfig& config,
    uint32_t level)
{
  updateTimer(timer_, config.frame_rate, config_.frame_rate);
  config_ = config;
}

void ImageDelay::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (images_.size() < config_.max_size)
    images_.push_back(msg);
  // ros::Timer timer = gePrivateNodeHandle().createTimer(
  //    ros::Duration(delay_), ImageDelay::update, true);
  // TODO(lucasw) need to add this timer to a map and have it get
  // remove once it is called.

  dirty_ = true;
  if (config_.frame_rate == 0)
  {
    ros::TimerEvent e;
    e.current_real = ros::Time::now();
    update(e);
  }
}

void ImageDelay::update(const ros::TimerEvent& e)
{
  bool has_published = false;
  if (images_.size() > 0)
  {
    if (images_[0]->header.stamp < e.current_real - ros::Duration(config_.delay))
    {
      if (!has_published)
      {
        const sensor_msgs::ImageConstPtr msg = images_[0];
        pub_.publish(msg);
        has_published = true;
      }
      // throw away other messages
      images_.pop_front();
    }
    else
    {
      // no other old messages to process
      return;
    }
  }
}

void ImageDelay::onInit()
{
  pub_ = getNodeHandle().advertise<sensor_msgs::Image>("image_out", 5);

  server_.reset(new ReconfigureServer(dr_mutex_, getPrivateNodeHandle()));
  dynamic_reconfigure::Server<image_manip::DelayConfig>::CallbackType cbt =
      boost::bind(&ImageDelay::callback, this, boost::placeholders::_1, boost::placeholders::_2);
  server_->setCallback(cbt);

  // if the input frame rate is too big and the delay too long, and queue_size
  // too small, then images get dropped, which may be desirable.
  // TODO(lucasw) put the queue_size and delay into dr cfg

  sub_ = getNodeHandle().subscribe("image_in", 5,
      &ImageDelay::imageCallback, this);

  timer_ = getPrivateNodeHandle().createTimer(ros::Duration(1.0),
      &ImageDelay::update, this);
  // force init
  updateTimer(timer_, config_.frame_rate, config_.frame_rate - 1.0);
}

};  // namespace image_manip

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_manip::ImageDelay, nodelet::Nodelet)

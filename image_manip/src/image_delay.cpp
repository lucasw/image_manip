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

#if 0
void ImageDelay::callback(
    image_manip::ImageDelayConfig& config,
    uint32_t level)
{
  config_ = config;
}
#endif

void ImageDelay::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  pub_.publish(msg);
  // ros::Timer timer = gePrivateNodeHandle().createTimer(
  //    ros::Duration(delay_), ImageDelay::update, true);
  // TODO(lucasw) need to add this timer to a map and have it get
  // remove once it is called.
}

#if 0
void ImageDelay::update(const sensor_msgs::ImageConstPtr& msg)
{
  pub_.publish(msg);
}
#endif

void ImageDelay::onInit()
{
  pub_ = getNodeHandle().advertise<sensor_msgs::Image>("image_out", 5);

  #if 0
  server_.reset(new ReconfigureServer(dr_mutex_, getPrivateNodeHandle()));
  dynamic_reconfigure::Server<image_manip::ImageDelayConfig>::CallbackType cbt =
      boost::bind(&ImageDelay::callback, this, _1, _2);
  server_->setCallback(cbt);
  #endif

  // if the input frame rate is too big and the delay too long, and queue_size
  // too small, then images get dropped, which may be desirable.
  // TODO(lucasw) put the queue_size and delay into dr cfg
  int queue_size = 10;
  getPrivateNodeHandle().getParam("queue_size", queue_size);
  ROS_INFO_STREAM("queue_size " << queue_size);
  delay_ = 2.0;
  getPrivateNodeHandle().getParam("delay", delay_);

  sub_ = getNodeHandle().subscribe("image_in", queue_size,
      &ImageDelay::imageCallback, this);
}

};  // namespace image_manip

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_manip::ImageDelay, nodelet::Nodelet)

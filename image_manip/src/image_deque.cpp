/*
 * Copyright (c) 2017 Lucas Walter
 * July 2017
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
#include <image_manip/image_deque.h>
#include <image_manip/utility.h>
#include <image_manip/utility_ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


namespace image_manip
{

ImageDeque::ImageDeque() :
  dirty_(false)
{
}

ImageDeque::~ImageDeque()
{
}

void ImageDeque::callback(
    image_manip::ImageDequeConfig& config,
    uint32_t level)
{
  if (level & 1)
  {
    if (config.capture_single)
    {
      capture_single_ = true;
      config.capture_single = false;
    }
    updateTimer(timer_, config.frame_rate, config_.frame_rate);
  }
  if (level & 4)
  {
    // TODO(lucasw) update config on every pubImage with latest index_?
    if (config_.index != config.index)
    {
      if (images_.size() > 0)
      {
        config.index_fraction = static_cast<double>(config.index) /
            static_cast<double>(images_.size());
      }
    }
    else if (config_.index_fraction != config.index_fraction)
    {
      config.index = config.index_fraction * images_.size();
    }
    index_ = config.index;
  }

  dirty_ = true;
  config_ = config;
}

void ImageDeque::maxSizeCallback(const std_msgs::UInt16::ConstPtr& msg)
{
  // TODO(lucasw) update dr, also keep dr updated with current size
  config_.max_size = msg->data;
  server_->updateConfig(config_);
}

void ImageDeque::singleCallback(const std_msgs::Bool::ConstPtr& msg)
{
  capture_single_ = msg->data;
  server_->updateConfig(config_);
}

void ImageDeque::continuousCallback(const std_msgs::Bool::ConstPtr& msg)
{
  // TODO(lucasw) update dr
  config_.capture_continuous = msg->data;
  server_->updateConfig(config_);
}

void ImageDeque::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (!config_.use_live_frame && !(capture_single_ || config_.capture_continuous))
    return;

  live_frame_ = msg;

  if (!(capture_single_ || config_.capture_continuous))
    return;

  images_.push_back(msg);

  // TODO(lucasw) make this optional
  // save the image with unique timestamp from init time + counter
  // Or have node that just saves any image it receives?

  if (capture_single_)
  {
    // should capture single capture the one already received, rather
    // than here it captures the next one received?
    ROS_INFO_STREAM("capturing single");
    capture_single_ = false;
  }

  if (config_.restrict_size)
  {
    while (images_.size() > config_.max_size)
      // TODO(lucasw) also could have mode where it fills and then doesn't accept any more
      images_.pop_front();
  }

  // TODO(lucasw) could put this in separate thread.
  // publish the exact same message received
  captured_pub_.publish(msg);
  std_msgs::Bool trigger;
  trigger.data = true;
  captured_trigger_pub_.publish(trigger);

  // TODO(lucasw) was this disabled earlier to work with the stop motion launch?
  dirty_ = true;
}

void ImageDeque::indexCallback(const std_msgs::UInt16::ConstPtr& msg)
{
  // if index is incrementing slower than the capture rate,
  // then odd backwards motion may result.
  if (msg->data != index_)
    dirty_ = true;
  index_ = msg->data;
  // TODO(lwalter) get rid of index_, just use config_.index
  config_.index = index_;
  server_->updateConfig(config_);

  if (config_.frame_rate == 0)
  {
    ros::TimerEvent e;
    e.current_real = ros::Time::now();
    update(e);
  }
}

void ImageDeque::update(const ros::TimerEvent& e)
{
  if (!dirty_)
    return;

  ROS_DEBUG_STREAM(index_ << " " << images_.size());
  // TEMP code to show output of frames
  // TODO(lucasw) have index be controlled by topic
  if (index_ <= images_.size())
  {
    if (index_ < images_.size())
      anim_pub_.publish(images_[index_]);
    else if (live_frame_)
      // preview the live frame at the end of the saved animation
      anim_pub_.publish(live_frame_);
    // TODO(lwalter) else publish something else?
    // index_++;
  }

  // ROS_INFO_STREAM(images_.size() << " " << index_);

  // TODO(lucasw) maybe if config_.start_index changes
  // it should force index_ to it, instead of waiting to cycle.
  #if 0
  if ((config_.use_live_frame && (index_ > images_.size())) ||
      (!config_.use_live_frame) && (index_ >= images_.size()))
  {
    index_ = config_.start_index;
  }
  #endif

  dirty_ = false;
}

void ImageDeque::onInit()
{
  captured_trigger_pub_ = getNodeHandle().advertise<std_msgs::Bool>("captured_image_trigger", 1);
  captured_pub_ = getNodeHandle().advertise<sensor_msgs::Image>("captured_image", 1, true);
  anim_pub_ = getNodeHandle().advertise<sensor_msgs::Image>("anim", 1);
  image_sub_ = getNodeHandle().subscribe("image", 1, &ImageDeque::imageCallback, this);
  index_sub_ = getNodeHandle().subscribe<std_msgs::UInt16>("index", 1,
                &ImageDeque::indexCallback, this);
  single_sub_ = getNodeHandle().subscribe<std_msgs::Bool>("single", 1,
                &ImageDeque::singleCallback, this);
  continuous_sub_ = getNodeHandle().subscribe<std_msgs::Bool>("continuous", 1,
                    &ImageDeque::continuousCallback, this);
  max_size_sub_ = getNodeHandle().subscribe<std_msgs::UInt16>("max_size", 1,
                  &ImageDeque::maxSizeCallback, this);

  server_.reset(new ReconfigureServer(dr_mutex_, getPrivateNodeHandle()));
  dynamic_reconfigure::Server<image_manip::ImageDequeConfig>::CallbackType cbt =
      boost::bind(&ImageDeque::callback, this, boost::placeholders::_1, boost::placeholders::_2);
  server_->setCallback(cbt);

  timer_ = getPrivateNodeHandle().createTimer(ros::Duration(1.0),
      &ImageDeque::update, this);
  // force init
  updateTimer(timer_, config_.frame_rate, config_.frame_rate - 1.0);
}

};  // namespace image_manip

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(image_manip::ImageDeque, nodelet::Nodelet)

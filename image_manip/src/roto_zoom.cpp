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
#include <image_manip/roto_zoom.h>
#include <image_manip/utility.h>
#include <image_manip/utility_ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <vector>


namespace image_manip
{

RotoZoom::RotoZoom()
{
}

RotoZoom::~RotoZoom()
{
}

void RotoZoom::callback(
    image_manip::RotoZoomConfig& config,
    uint32_t level)
{
  updateTimer(timer_, config.frame_rate, config_.frame_rate);
  config_ = config;
  dirty_ = true;

  if (config_.frame_rate == 0)
  {
    ros::TimerEvent e;
    update(e);
  }
}

void RotoZoom::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // need mutex around image_ uses
  images_.push_back(msg);
  while (images_.size() > 1)
    images_.pop_front();
  dirty_ = true;

  if (config_.frame_rate == 0)
  {
    ros::TimerEvent e;
    update(e);
  }
}

void RotoZoom::backgroundImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // need mutex around image_ uses
  if (!background_image_)
    ROS_INFO_STREAM("Using background image " << msg->width << " " << msg->height);
  background_image_ = msg;
  dirty_ = true;

  if (config_.frame_rate == 0)
  {
    ros::TimerEvent e;
    update(e);
  }
}

void RotoZoom::update(const ros::TimerEvent& e)
{
  if (!dirty_)
    return;
  dirty_ = false;

  // make a copy now of the pointer, if it is overwritten later this should notice
  // but probably still need mutex around this one line
  sensor_msgs::ImageConstPtr background_image = background_image_;

  if (images_.size() == 0)
  {
    if (!background_image)
      return;

    pub_.publish(background_image);
    return;
  }

  const sensor_msgs::ImageConstPtr msg = images_[0];
  cv_bridge::CvImageConstPtr cv_ptr;
  if (!imageToMat(msg, cv_ptr))
    return;

  // TODO(lucasw) should this be the background_image width and height if available?
  const float wd = msg->width;
  const float ht = msg->height;
  if ((wd == 0) || (ht == 0))
  {
    // ROS_ERROR_STREAM(wd << " " << ht);
    return;
  }

  float bg_wd = wd;
  float bg_ht = ht;
  if (background_image)
  {
    bg_wd = background_image->width;
    bg_ht = background_image->height;
  }

  float phi = config_.phi;
  float theta = config_.theta;
  float psi = config_.psi;

  cv::Point3f center;
  // TODO(lucasw) this doesn't work as expected
  center.x = config_.center_x;
  center.y = config_.center_y;

  float off_x = config_.off_x;
  float off_y = config_.off_y;

  const float z = config_.z;
  const float z_scale = config_.z_scale;

  cv::Mat transform;
  getPerspectiveTransform(wd, ht, phi, theta, psi, off_x, off_y,
      z, z_scale, center, transform);

  sensor_msgs::ImagePtr output_image;
  cv::Size dst_size;
  // TODO(lucasw) create a ImagePtr with memory allocated for dst_size,
  // then use toCvShare to get the cv::Mat version of it and operate
  // warpPerspective on it to avoid teh toImageMsg() image copy.
  cv_bridge::CvImageConstPtr bg_cv_ptr;
  if (background_image)
  {
    // change background image encoding to foreground image
    if (!imageToMat(background_image, bg_cv_ptr, msg->encoding))
    {
      ROS_ERROR_STREAM("background image couldn't be converted to mat pointer");
      return;
    }
    cv_bridge::CvImage cv_image;
    cv_image.image = bg_cv_ptr->image;
    cv_image.encoding = background_image->encoding;
    // this copies the image, don't need to clone above
    // Can't avoid this image copy, otherwise the next iteration
    // will have a background image that already has a warped foreground image
    // on it.
    // TODO(lucasw) could try to get smart and only clone if the input
    // update rate is very slow- but that seems unreliable.
    output_image = cv_image.toImageMsg();
    // TODO(lucasw) is there a more direct Image copy than above?
    // output_image_.reset(new sensor_msgs::ImagePtr(background_image));
    dst_size = cv::Size(output_image->width, output_image->height);
  }
  else
  {
    // No background image, need to allocate output image that is same size
    // as input image.
    if (!sameImageType(output_image_info_, msg))
    {
      dst_size = cv_ptr->image.size();
      cv_bridge::CvImage cv_image;
      // TODO(lucasw) this is creating a mat then immediately copying it-
      // would rather create the image just once,
      // but probably need to construct Image on own?
      cv_image.image = cv::Mat(dst_size, cv_ptr->image.type(), cv::Scalar::all(0));
      cv_image.encoding = msg->encoding;
      output_image = cv_image.toImageMsg();
    }
    else
    {
      // TODO(lucasw) what is copied here
      *output_image = output_image_info_;
      // TODO(lucasw) zero out data
    }
  }

  const int type = cv_bridge::getCvType(output_image->encoding);
  // this is safe because the size and type won't be changed
  cv::Mat out = cv::Mat(dst_size, type, static_cast<uchar*>(&output_image->data[0]),
      output_image->step);
  // TBD make inter_nearest changeable
  cv::warpPerspective(cv_ptr->image, out, transform,
                      dst_size,
                      cv::INTER_NEAREST, config_.border);

  output_image->header.stamp = ros::Time::now();  // or reception time of original message?
  output_image->header.frame_id = msg->header.frame_id;

  output_image_info_.height = output_image->height;
  output_image_info_.width = output_image->width;
  output_image_info_.encoding = output_image->encoding;
  output_image_info_.is_bigendian = output_image->is_bigendian;
  output_image_info_.step = output_image->step;
  output_image_info_.data.resize(output_image->data.size());

  pub_.publish(output_image);
}

void RotoZoom::paramCallback(std_msgs::Float32::ConstPtr msg, const std::string name)
{
  if (params_.count(name) > 0) {
    *params_[name] = msg->data;
  } else {
    ROS_ERROR_STREAM("no name " << name << " in params");
  }
}

void RotoZoom::onInit()
{
  pub_ = getNodeHandle().advertise<sensor_msgs::Image>("image_out", 5);

  server_.reset(new ReconfigureServer(dr_mutex_, getPrivateNodeHandle()));
  dynamic_reconfigure::Server<image_manip::RotoZoomConfig>::CallbackType cbt =
      boost::bind(&RotoZoom::callback, this, boost::placeholders::_1, boost::placeholders::_2);
  server_->setCallback(cbt);

  params_["phi"] = &config_.phi;
  params_["theta"] = &config_.theta;
  params_["psi"] = &config_.psi;
  params_["center_x"] = &config_.center_x;
  params_["center_y"] = &config_.center_y;
  params_["off_x"] = &config_.off_x;
  params_["off_y"] = &config_.off_y;
  params_["z"] = &config_.z;
  params_["z_scale"] = &config_.z_scale;

  sub_ = getNodeHandle().subscribe("image_in", 5,
      &RotoZoom::imageCallback, this);

  std::vector<std::string> params = {"phi", "theta", "psi",
    "center_x", "center_y", "off_x", "off_y", "z", "z_scale"};
  for (const auto& param : params) {
    param_sub_[param] = getPrivateNodeHandle().subscribe<std_msgs::Float32>(
        param, 1, boost::bind(&RotoZoom::paramCallback, this, boost::placeholders::_1, param));
  }

  background_sub_ = getNodeHandle().subscribe("background_image", 5,
      &RotoZoom::backgroundImageCallback, this);

  timer_ = getPrivateNodeHandle().createTimer(ros::Duration(1.0),
    &RotoZoom::update, this);
  // force timer start by making old frame_rate different
  updateTimer(timer_, config_.frame_rate, config_.frame_rate - 1.0);
}

};  // namespace image_manip

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(image_manip::RotoZoom, nodelet::Nodelet)

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

  float scale = 1.0;

  const bool nrm_px = true;

  cv::Point3f center;
  // TODO(lucasw) this doesn't work as expected
  center.x = config_.center_x;
  center.y = config_.center_y;
  // TODO(lucasw) this has no effect
  center.z = 0.1;  // config_.center_z;

  float off_x = config_.off_x;
  float off_y = config_.off_y;
  float off_z = 0.0;  // config_.off_z;

  if (nrm_px)
  {
    center.x = center.x * wd;  // + wd * 3 / 4;
    center.y = center.y * ht;  // + ht * 3 / 4;
    // center.z *= ht;

    off_x = off_x * wd + wd / 2;
    off_y = off_y * ht + ht / 2;
    // off_z *= ht;
  }

  cv::Mat in_p = (cv::Mat_<float>(3, 4) <<
                  0, wd, wd, 0,
                  0, 0,  ht, ht,
                  0, 0, 0, 0);

  cv::Mat in_roi = in_p.t()(cv::Rect(0, 0, 2, 4));  // ).clone();
  in_roi = in_roi.clone();
  cv::Mat out_roi;

  {
    // This implements a standard rotozoom
    // move the image prior to rotation
    cv::Mat offset = (cv::Mat_<float>(3, 4) <<
                      off_x, off_x, off_x, off_x,
                      off_y, off_y, off_y, off_y,
                      off_z, off_z, off_z, off_z);

    // shift the image after rotation
    cv::Mat center_m = (cv::Mat_<float>(3, 4) <<
                        center.x, center.x, center.x, center.x,
                        center.y, center.y, center.y, center.y,
                        center.z, center.z, center.z, center.z);

    // Rotation matrices
    cv::Mat rotz = (cv::Mat_<float>(3, 3) <<
                    cos(phi), -sin(phi), 0,
                    sin(phi),  cos(phi), 0,
                    0, 0, 1);

    cv::Mat roty = (cv::Mat_<float>(3, 3) <<
                    cos(theta),  0, sin(theta),
                    0, 1, 0,
                    -sin(theta), 0, cos(theta));

    cv::Mat rotx = (cv::Mat_<float>(3, 3) <<
                    1,  0,        0,
                    0,  cos(psi), sin(psi),
                    0, -sin(psi), cos(psi));

    // TBD reformat the matrices so all the transposes aren't necessary

    // Transform into ideal coords
    // float fx = getSignal("fx");
    cv::Mat out_p = (in_p - offset).t() * rotx.t() * roty.t() * rotz.t() * scale;  // + center_m.t();
    out_roi = out_p(cv::Rect(0, 0, 2, 4)).clone();

    // this moves the image away from the 0 plane
    // TODO(lucasw) this has no effect
    const float z = config_.z;
    const float z_scale = config_.z_scale;
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 2; j++)
      {
        // this makes the projection non-orthographic
        const float out_p_z = out_p.at<float>(i, 2);
        out_roi.at<float>(i, j) = out_p.at<float>(i, j) / (out_p_z * z_scale + z) +
            center_m.at<float>(j, i) + offset.at<float>(j, i);
      }
    }
  }

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
  cv::Mat transform = cv::getPerspectiveTransform(in_roi, out_roi);
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

void RotoZoom::onInit()
{
  pub_ = getNodeHandle().advertise<sensor_msgs::Image>("image_out", 5);

  server_.reset(new ReconfigureServer(dr_mutex_, getPrivateNodeHandle()));
  dynamic_reconfigure::Server<image_manip::RotoZoomConfig>::CallbackType cbt =
      boost::bind(&RotoZoom::callback, this, _1, _2);
  server_->setCallback(cbt);

  sub_ = getNodeHandle().subscribe("image_in", 5,
      &RotoZoom::imageCallback, this);

  background_sub_ = getNodeHandle().subscribe("background_image", 5,
      &RotoZoom::backgroundImageCallback, this);

  timer_ = getPrivateNodeHandle().createTimer(ros::Duration(1.0),
    &RotoZoom::update, this);
  // force timer start by making old frame_rate different
  updateTimer(timer_, config_.frame_rate, config_.frame_rate - 1.0);
}

};  // namespace image_manip

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_manip::RotoZoom, nodelet::Nodelet)

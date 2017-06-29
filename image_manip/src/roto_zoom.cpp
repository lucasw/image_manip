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
  config_ = config;
}

void RotoZoom::imageCallback(const sensor_msgs::ImageConstPtr& msg)
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

  const float wd = msg->width;
  const float ht = msg->height;
  if ((wd == 0) || (ht == 0))
  {
    // ROS_ERROR_STREAM(wd << " " << ht);
    return;
  }

  float phi = config_.phi;
  float theta = config_.theta;
  float psi = config_.psi;

  float scale = 1.0;

  const bool nrm_px = false;

  cv::Point3f center;
  center.x = wd/2;
  center.y = ht/2;
  center.z = 0;

  float off_x = 0;
  float off_y = 0;
  float off_z = 0;

  if (nrm_px)
  {
    center.x *= wd;
    center.y *= ht;
    center.z *= ht;

    off_x *= wd;
    off_y *= ht;
    off_z *= ht;
  }

  cv::Mat in_p = (cv::Mat_<float>(3, 4) <<
                  0, wd, wd, 0,
                  0, 0,  ht, ht,
                  0, 0, 0, 0);

  cv::Mat in_roi = in_p.t()(cv::Rect(0, 0, 2, 4));  // ).clone();
  in_roi = in_roi.clone();
  cv::Mat out_roi = in_roi.clone();

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
    const float z = 1.0;  // TODO(lucasw) config_
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 2; j++)
      {
        out_roi.at<float>(i, j) = z * out_p.at<float>(i, j) / (out_p.at<float>(i, 2) + z) + center_m.at<float>(j, i);
      }
    }
  }

  cv::Mat out;
  // TBD make inter_nearest changeable
  ROS_INFO_STREAM(in_roi);
  ROS_INFO_STREAM(out_roi);
  cv::Mat transform = cv::getPerspectiveTransform(in_roi, out_roi);
  cv::warpPerspective(cv_ptr->image, out, transform,
                      cv_ptr->image.size(),
                      cv::INTER_NEAREST, cv::BORDER_REFLECT_101);
                      // getModeType(), getBorderType());

  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = ros::Time::now();  // or reception time of original message?
  cv_image.image = out;
  cv_image.encoding = "rgb8";
  pub_.publish(cv_image.toImageMsg());
}

#if 0
void RotoZoom::update(const sensor_msgs::ImageConstPtr& msg)
{
  pub_.publish(msg);
}
#endif

void RotoZoom::onInit()
{
  pub_ = getNodeHandle().advertise<sensor_msgs::Image>("image_out", 5);

  server_.reset(new ReconfigureServer(dr_mutex_, getPrivateNodeHandle()));
  dynamic_reconfigure::Server<image_manip::RotoZoomConfig>::CallbackType cbt =
      boost::bind(&RotoZoom::callback, this, _1, _2);
  server_->setCallback(cbt);

  sub_ = getNodeHandle().subscribe("image_in", 5,
      &RotoZoom::imageCallback, this);
}

};  // namespace image_manip

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_manip::RotoZoom, nodelet::Nodelet)
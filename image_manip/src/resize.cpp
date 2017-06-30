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
#include <image_manip/resize.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


namespace image_manip
{

/// resize the source tmp0 mat to fit inside tmp1 with borders
/// tmp0 and tmp1 have to be initialized already
/// TBD add another mode which chops off the edges so there
/// are no borders?
bool resizeFixAspect(const cv::Mat& tmp0, cv::Mat& tmp1,
    const cv::Size sz, const int mode)
{
  tmp1 = cv::Mat(sz, tmp0.type(), cv::Scalar::all(0));

  const float aspect_0 = (float)tmp0.cols / (float)tmp0.rows;
  const float aspect_1 = (float)tmp1.cols / (float)tmp1.rows;

  // this is the subimage that has to fit within tmp1
  // it will be shrunk down as necessary and border offset
  // values adjusted
  cv::Size tmp_sz = tmp1.size();
  int off_x = 0;
  int off_y = 0;

  // TBD could have epsilon defined by 1 pixel width
  if (aspect_0 > aspect_1)
  {
    // have to have a border on top
    tmp_sz.height = tmp_sz.width / aspect_0;
    off_y = (sz.height - tmp_sz.height) / 2;
  }
  else if (aspect_0 < aspect_1)
  {
    // have a border on the sides
    tmp_sz.width = tmp_sz.height * aspect_0;
    off_x = (sz.width - tmp_sz.width) / 2;
  }

  #if 0
  ROS_DEBUG_STREAM_COND(log_level > 2, "fix aspect " << aspect_0 << " " << aspect_1 << ", "
                        << off_x << " " << off_y << ", "
                        << tmp_sz.width << " " << tmp_sz.height << ", "
                        << sz.width << " " << sz.height);
  #endif

  // the source image with the right aspect ratio and size
  // to fit within the dest image
  cv::Mat tmp_aspect;
  cv::resize(tmp0, tmp_aspect, tmp_sz, 0, 0, mode);

  // TBD put offset so image is centered
  cv::Mat tmp1_roi = tmp1(cv::Rect(off_x, off_y, tmp_sz.width, tmp_sz.height));
  tmp_aspect.copyTo(tmp1_roi);

  return true;
}

/// another mode which chops off the edges so there
/// are no borders
bool resizeFixAspectFill(const cv::Mat& tmp0, cv::Mat& tmp1,
    const cv::Size sz, const int mode)
{
  tmp1 = cv::Mat(sz, tmp0.type(), cv::Scalar::all(0));
  // width/height
  const float aspect_0 = (float)tmp0.cols / (float)tmp0.rows;
  const float aspect_1 = (float)tmp1.cols / (float)tmp1.rows;

  const cv::Size src_sz = tmp0.size();

  // this is the subimage that has to fit within tmp1
  // it will be shrunk down as necessary and border offset
  // values adjusted
  cv::Size tmp_sz = sz;
  int off_x = 0;
  int off_y = 0;

  // TBD could have epsilon defined by 1 pixel width
  if (aspect_0 > aspect_1)
  {
    // lose the edges off the sides
    tmp_sz.width = sz.height * aspect_0;
    off_x = (tmp_sz.width - sz.width) / 2;
  }
  else if (aspect_0 < aspect_1)
  {
    // lose the edges of the top and bottom
    tmp_sz.height = sz.width / aspect_0;
    off_y = (tmp_sz.height - sz.height) / 2;
  }

  #if 0
  ROS_DEBUG_STREAM_COND(log_level > 2, "fix aspect fill "
                        << "src " << src_sz.width << " " << src_sz.height << ", "
                        << "tmp " << off_x << " " << off_y << " "
                        << tmp_sz.width << " " << tmp_sz.height << ", "
                        << "dst " << sz.width << " " << sz.height);
  #endif

  // resize the source image so that it is equal to the dest
  // image in at least one dimension,
  cv::Mat tmp_aspect;
  cv::resize(tmp0, tmp_aspect, tmp_sz, 0, 0, mode);

  // now take just the subimage so the aspect is preserved
  // while the resolution is dst
  cv::Mat tmp1_roi = tmp_aspect(cv::Rect(off_x, off_y, sz.width, sz.height));
  tmp1_roi.copyTo(tmp1);

  return true;
}

Resize::Resize()
{
}

Resize::~Resize()
{
}

void Resize::callback(
    image_manip::ResizeConfig& config,
    uint32_t level)
{
  config_ = config;
}

void Resize::imageCallback(const sensor_msgs::ImageConstPtr& msg)
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

  cv_bridge::CvImage cv_image;
  const cv::Size size(config_.width, config_.height);

	if (config_.mode == 0)
  {
    resizeFixAspect(cv_ptr->image, cv_image.image, size, config_.interpolate_mode);
  }
	if (config_.mode == 1)
  {
    resizeFixAspectFill(cv_ptr->image, cv_image.image, size, config_.interpolate_mode);
  }
	if (config_.mode == 2)
  {
    cv::resize(cv_ptr->image, cv_image.image, size, 0, 0, config_.interpolate_mode);
  }

  cv_image.header = cv_ptr->header;  // or reception time of original message?
  cv_image.encoding = msg->encoding;
  pub_.publish(cv_image.toImageMsg());
}

#if 0
void Resize::update(const sensor_msgs::ImageConstPtr& msg)
{
  pub_.publish(msg);
}
#endif

void Resize::onInit()
{
  pub_ = getNodeHandle().advertise<sensor_msgs::Image>("image_out", 5);

  server_.reset(new ReconfigureServer(dr_mutex_, getPrivateNodeHandle()));
  dynamic_reconfigure::Server<image_manip::ResizeConfig>::CallbackType cbt =
      boost::bind(&Resize::callback, this, _1, _2);
  server_->setCallback(cbt);

  sub_ = getNodeHandle().subscribe("image_in", 5,
      &Resize::imageCallback, this);
}

};  // namespace image_manip

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_manip::Resize, nodelet::Nodelet)

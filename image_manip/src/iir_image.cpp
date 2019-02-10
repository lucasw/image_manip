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
#include <deque>
#include <image_manip/iir_image.hpp>
#include <image_manip/utility.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/float64.hpp>
using std::placeholders::_1;


namespace image_manip
{

IIRImage::IIRImage(std::shared_ptr<internal_pub_sub::Core> core) :
    Node("iir_image"), core_(core)
{
}

IIRImage::~IIRImage()
{
}

void IIRImage::update()
{
  if (!dirty_)
    return;

  cv::Mat out_frame;

  for (size_t i = 0; i < in_images_.size() && i < b_coeffs_.size(); ++i)
  {
    if (!in_images_[i])
      continue;

    // convert the image on demand
    if (!in_cv_images_[i])
    {
      try
      {
        // TODO(lucasw) don't want to convert the same images over and over,
        // so store both the sensor_msg and convert and save on demand?
        // TBD why converting to BGR8-
        // this will incur image data copy if not the native format
        in_cv_images_[i] =
            cv_bridge::toCvShare(in_images_[i],
                sensor_msgs::image_encodings::RGB8);
        //, "mono8"); // sensor_msgs::image_encodings::MONO8);
      }
      catch (cv_bridge::Exception& e)
      {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        continue;
      }
    }

    const double bn = b_coeffs_[i];
    if (out_frame.empty())
      out_frame = in_cv_images_[i]->image * bn;
    else if ((out_frame.size() == in_cv_images_[i]->image.size()) &&
             (out_frame.type() == in_cv_images_[i]->image.type()))
    {
      // TODO(lucasw) if size/type mismatch have optional mode
      // to convert the cv_ptr image.
      if (bn > 0)
        out_frame += in_cv_images_[i]->image * bn;
      else if (bn < 0)
        out_frame -= in_cv_images_[i]->image * -bn;
    }
  }
  if (out_frame.empty())
    return;
  // since the current frame hasn't been pushed yet,
  // i-1 is actually index i for out_frames_.
  for (size_t i = 1; (i - 1) < out_frames_.size() && i < a_coeffs_.size(); ++i)
  {
    if ((out_frame.size() == out_frames_[i - 1].size()) &&
        (out_frame.type() == out_frames_[i - 1].type()))
    {
      const float an = -a_coeffs_[i];
      if (an > 0)
        out_frame += out_frames_[i - 1] * an;
      else if (an < 0)
        out_frame -= out_frames_[i - 1] * -an;
    }
  }
  if (!out_frame.empty() && (a_coeffs_.size() > 0) && (a_coeffs_[0] != 1.0))
  {
    out_frame *= 1.0 / a_coeffs_[0];
  }

  // TODO(lucasw) this may be argument for keeping original Image messages around
  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = now();  // or reception time of original message?
  cv_image.image = out_frame;
  cv_image.encoding = "rgb8";
  sensor_msgs::msg::Image::SharedPtr out_image(cv_image.toImageMsg());
  image_pub_->publish(out_image);

  out_frames_.push_front(out_frame);
  if (out_frames_.size() > a_coeffs_.size())
    out_frames_.pop_back();

  // don't care if dirty_ would have become true again
  // had it been set false immediately after testing it.
  dirty_ = false;
}

void IIRImage::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  in_images_.push_front(msg);
  in_cv_images_.push_front(cv_bridge::CvImageConstPtr());

  if (in_images_.size() > b_coeffs_.size())
  {
    in_images_.pop_back();
    in_cv_images_.pop_back();
  }

  dirty_ = true;
}

void IIRImage::imagesCallback(const sensor_msgs::msg::Image::SharedPtr msg, const size_t index)
{
  if (index >= in_images_.size())
    return;

  in_images_[index] = msg;
  in_cv_images_[index] = cv_bridge::CvImageConstPtr();

  dirty_ = true;
}

void IIRImage::init()
{
  // pub_ = create_publisher<sensor_msgs::msg::Image>("image_out");
  image_pub_ = core_->get_create_publisher("image_out", shared_from_this());

  get_parameter_or("use_time_sequence", use_time_sequence_, use_time_sequence_);

  // TODO(lucasw) update config from b_coeffs
  {
    int num_b = 0;
    get_parameter_or("num_b", num_b, num_b);
    b_coeffs_.resize(num_b);
    const double div = 1.0 / static_cast<double>(num_b);
    for (size_t i = 0; i < b_coeffs_.size(); ++i)
    {
      get_parameter_or("b" + std::to_string(i), b_coeffs_[i], div);
    }
    int num_a = 0;
    get_parameter_or("num_a", num_a, num_a);
    a_coeffs_.resize(num_a);
    for (size_t i = 0; i < a_coeffs_.size(); ++i)
    {
      get_parameter_or("a" + std::to_string(i), a_coeffs_[i], (i > 0) ? 0.0 : 1.0);
    }
  }

  if (!use_time_sequence_)
  {
    in_images_.resize(b_coeffs_.size());
    in_cv_images_.resize(b_coeffs_.size());

    for (size_t i = 0; i < b_coeffs_.size(); ++i)
    {
      std::stringstream ss;
      ss << "image_" << i;
      RCLCPP_INFO(get_logger(), "subscribe %s %f", ss.str().c_str(), b_coeffs_[i]);

      std::function<void(std::shared_ptr<sensor_msgs::msg::Image>)> fnc;
      fnc = std::bind(&IIRImage::imagesCallback, this, _1, i);
      image_subs_.push_back(core_->create_subscription(
          ss.str(), fnc, shared_from_this()));
    }
  }
  else
  {
    // sub_ = create_subscription<sensor_msgs::msg::Image>("image_in",
    //     std::bind(&IIRImage::imageCallback, this, _1));
    image_sub_ = core_->create_subscription("image_in",
        std::bind(&IIRImage::imageCallback, this, _1),
        shared_from_this());
  }

  get_parameter_or("frame_rate", frame_rate_, frame_rate_);
  const int period_ms = 1000.0 / frame_rate_;
  timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
      std::bind(&IIRImage::update, this));

  RCLCPP_INFO(get_logger(), "iir image initialized");
}

}  // namespace image_manip

#include <class_loader/register_macro.hpp>

CLASS_LOADER_REGISTER_CLASS(image_manip::IIRImage, rclcpp::Node)

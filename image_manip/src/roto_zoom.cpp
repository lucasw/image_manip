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
#include <image_manip/utility.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <string>
using std::placeholders::_1;

namespace image_manip
{

class RotoZoom : public rclcpp::Node
{
public:
RotoZoom() : Node("rotozoom")
{
  dirty_ = false;
  pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_out");

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_in",
      std::bind(&image_manip::RotoZoom::imageCallback, this, _1));
  phi_sub_ = this->create_subscription<std_msgs::msg::Float32>("phi",
      std::bind(&image_manip::RotoZoom::phiCallback, this, _1));
  theta_sub_ = this->create_subscription<std_msgs::msg::Float32>("theta",
      std::bind(&image_manip::RotoZoom::thetaCallback, this, _1));

  // timer_ = getPrivateNodeHandle().createTimer(ros::Duration(1.0),
  //  &update, this);
}

~RotoZoom()
{
}

void phiCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (phi_ != msg->data) {
    phi_ = msg->data;
    dirty_ = true;

    if (frame_rate_ == 0)
    {
      update();
    }
  }
}

void thetaCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (theta_ != msg->data) {
    theta_ = msg->data;
    dirty_ = true;

    if (frame_rate_ == 0)
    {
      update();
    }
  }
}

void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  msg_ = msg;
  dirty_ = true;

  // negative frame_rate is a disable
  // TODO(lucasw) or should that be the other way around?
  if (frame_rate_ == 0)
  {
    update();
  }
}

void update()
{
  if (!dirty_)
    return;

  if (!msg_)
    return;

  const sensor_msgs::msg::Image::SharedPtr msg = msg_;

  const float wd = msg->width;
  const float ht = msg->height;
  if ((wd == 0) || (ht == 0))
  {
    // ROS_ERROR_STREAM(wd << " " << ht);
    return;
  }

  // TODO(lucasw) optionally convert encoding to dr type or keep same
  const std::string encoding = msg->encoding;
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // TBD why converting to BGR8
    cv_ptr = cv_bridge::toCvShare(msg, encoding);
    //, "mono8"); // sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImage cv_image;

  cv::Size dst_size = cv_ptr->image.size();
  const float phi = phi_;
  const float theta = theta_;
  const float psi = 0.0;
  cv::Point3f center(0, 0, 0);
  float off_x = 0;
  float off_y = 0;
  const float z = 1.0;
  const float z_scale = 0.005;

  cv::Mat transform;
  // std::cout << "phi theta psi " << phi << " " << theta << " " << psi << "\n";
  getPerspectiveTransform(wd, ht, phi, theta, psi, off_x, off_y,
      z, z_scale, center, transform);

  const int border = cv::BORDER_REFLECT;  // TRANSPARENT;
  // TODO(lucasw) don't waste time creating an empty image if the border
  // type will overwrite it all anyhow
  cv_image.image = cv::Mat(dst_size, cv_ptr->image.type(), cv::Scalar::all(0));
  cv::warpPerspective(cv_ptr->image, cv_image.image, transform,
                      dst_size,
                      cv::INTER_NEAREST, border);

  cv_image.header = cv_ptr->header;  // or reception time of original message?
  cv_image.encoding = encoding;
  // TODO(lucasw) the image copy in toImageMsg ought to be avoided
  // (assuming it does copy as it does in ros1)
  pub_->publish(cv_image.toImageMsg());

  dirty_ = false;
}

private:
  // sensor_msgs::
  bool dirty_ = false;
  float phi_ = 0.0;
  float theta_ = 0.0;
  float frame_rate_ = 0.0;
  unsigned int mode_ = 0;
  sensor_msgs::msg::Image::SharedPtr msg_;
  sensor_msgs::msg::Image::SharedPtr background_image_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr background_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr phi_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr theta_sub_;
};
}  // namespace image_manip


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::spin(std::make_shared<image_manip::RotoZoom>());
  rclcpp::shutdown();
  return 0;
}


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
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <map>
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

  for (auto it = controls_.begin(); it != controls_.end(); ++it) {
    std::string name = it->first;
    // auto doesn't work
    std::function<void(std::shared_ptr<std_msgs::msg::Float32>)> fnc;
    fnc = std::bind(&image_manip::RotoZoom::controlCallback, this, _1, name);
    control_subs_[name] = create_subscription<std_msgs::msg::Float32>(name, fnc);
  }

  for (auto it = int_controls_.begin(); it != int_controls_.end(); ++it) {
    std::string name = it->first;
    // auto doesn't work
    std::function<void(std::shared_ptr<std_msgs::msg::Int32>)> fnc;
    fnc = std::bind(&image_manip::RotoZoom::intControlCallback, this, _1, name);
    int_subs_[name] = create_subscription<std_msgs::msg::Int32>(name, fnc);
  }

  set_parameter_if_not_set("frame_rate", frame_rate_);
  get_parameter_or("frame_rate", frame_rate_, frame_rate_);
  updateTimer();

  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  param_sub_ = parameters_client_->on_parameter_event(
      std::bind(&RotoZoom::onParameterEvent, this, _1));
}

~RotoZoom()
{
}

void updateTimer()
{
  if (frame_rate_ <= 0) {
    RCLCPP_INFO(get_logger(), "Only updating when controls change %f", frame_rate_);
    timer_ = nullptr;
    return;
  }
  const long int period_ms = 1000.0 / frame_rate_;
  RCLCPP_INFO(get_logger(), "Updating at fixed rate %f %d", frame_rate_, period_ms);
  timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&RotoZoom::update, this));
}

// TODO(lucasw) could replace with generic callback
void controlCallback(const std_msgs::msg::Float32::SharedPtr msg,
    std::string name)
{
  // this should never happen
  if (controls_.count(name) < 1) {
    RCLCPP_ERROR(get_logger(), "unknown control %s", name.c_str());
    return;
  }
  if (controls_[name] != msg->data) {
    controls_[name] = msg->data;
    dirty_ = true;

    if (frame_rate_ <= 0) {
      update();
    }
  }
}

void intControlCallback(const std_msgs::msg::Int32::SharedPtr msg,
    std::string name)
{
  // this should never happen
  if (int_controls_.count(name) < 1) {
    RCLCPP_ERROR(get_logger(), "unknown control %s", name.c_str());
    return;
  }
  if (int_controls_[name] != msg->data) {
    int_controls_[name] = msg->data;
    dirty_ = true;

    if (frame_rate_ <= 0) {
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
  if (frame_rate_ <= 0) {
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
  // TODO(lucasw) this probably messes up roto centering
  if (int_controls_["width"] > 0) {
    dst_size.width = int_controls_["width"];
  }
  if (int_controls_["height"] > 0) {
    dst_size.height = int_controls_["height"];
  }
  const float phi = controls_["phi"];
  const float theta = controls_["theta"];
  const float psi = controls_["psi"];
  cv::Point3f center(
      controls_["center_x"], controls_["center_y"], controls_["center_z"]);
  float off_x = controls_["off_x"];
  float off_y = controls_["off_y"];
  const float z = controls_["z"];
  const float z_scale = controls_["z_scale"];

  cv::Mat transform;
  // std::cout << "phi theta psi " << phi << " " << theta << " " << psi << "\n";
  getPerspectiveTransform(dst_size.width, dst_size.height, phi, theta, psi, off_x, off_y,
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

  std::map<std::string, int> int_controls_ = {
    {"width", 0},
    {"height", 0},
  };

  std::map<std::string, float> controls_ = {
    {"phi", 0.0},
    {"theta", 0.0},
    {"psi", 0.0},
    {"off_x", 0.0},
    {"off_y", 0.0},
    {"center_x", 0.0},
    {"center_y", 0.0},
    {"center_z", 0.0},
    {"z", 1.0},
    {"z_scale", 0.005},
  };

  unsigned int mode_ = 0;
  sensor_msgs::msg::Image::SharedPtr msg_;
  sensor_msgs::msg::Image::SharedPtr background_image_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr background_sub_;

  std::map<std::string, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> control_subs_;
  std::map<std::string, rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> int_subs_;

  double frame_rate_ = 0.0;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;

  void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {
    // auto -> ParameterValue
    for (auto & changed_parameter : event->changed_parameters) {
      const std::string name = changed_parameter.name;
      if (name == "frame_rate")
      {
        if (changed_parameter.value.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
          RCLCPP_WARN(get_logger(), "Wrong type %s %d",
              name, changed_parameter.value.type);
          continue;
        }
        frame_rate_ = changed_parameter.value.double_value;
        dirty_ = true;
        updateTimer();
      } else {
        // TODO(lucasw) maybe should rescan controls, or provide that function elsewhere
        RCLCPP_WARN(get_logger(), "No %s in controls", name.c_str());
        continue;
      }  // is expected parameter
    }  // loop through changed parameters
  }  // parameter event handler

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

